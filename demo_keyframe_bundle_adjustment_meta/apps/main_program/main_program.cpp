/**
LIMO 2022
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
\author Hojun Ji
**/

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_preproc/gamma_corrector.h>
#include <image_preproc/image_preproc_params.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <iterator>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "tracklets_depth/tracklet_depth_module.h"
#include "monolidar_fusion/DepthEstimatorParameters.h"
#include "utility.h"
#include "viso_feature_tracking/viso_feature_tracker_module.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "limo_deterministic_run");
  // Nodehandle for parameters.
  ros::NodeHandle node_handle("~");

  /*****************************************************************
                          Read the parameters
  ******************************************************************/
  std::string dataset_folder, sequence_number, output_bag_file;
  // Dataset is assumed to follow the Kitti convention and format.
  node_handle.getParam("dataset_folder", dataset_folder);
  node_handle.getParam("sequence_number", sequence_number);
  std::cout << "Reading sequence " << sequence_number << " from "
            << dataset_folder << '\n';
  bool to_bag;
  node_handle.getParam("to_bag", to_bag);
  if (to_bag) node_handle.getParam("output_bag_file", output_bag_file);

  // Read the viso tracking configuration
  std::string visoConfigFileName;
  if (!node_handle.getParam("viso_config", visoConfigFileName)) {
    throw std::runtime_error("No viso configuration specified");
  }
  // Read the tracklet depth configuration
  std::string trackletDepthConfigFileName;
  if (!node_handle.getParam("tracklet_depth_config",
                            trackletDepthConfigFileName)) {
    throw std::runtime_error("No tracklet depth configuration specified");
  }

  std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
  std::ifstream timestamp_file(dataset_folder + timestamp_path,
                               std::ifstream::in);

  std::string ground_truth_path = "results/" + sequence_number + ".txt";
  std::ifstream ground_truth_file(dataset_folder + ground_truth_path,
                                  std::ifstream::in);

  /*****************************************************************
                          Module Instantiation
  ******************************************************************/
  // Preprocessor module.
  image_preproc::GammaCorrector corrector;

  viso_feature_tracking::VisoFeatureTrackerModule feature_tracker_module(
      viso_feature_tracking::VisoFeatureTrackingParameters(visoConfigFileName)
          .get_config(),
      viso_feature_tracking::VisoFeatureTrackingParameters(visoConfigFileName)
          .get_matcher_parameters());

  tracklets_depth::TrackletDepthParameters tracklet_depth_param;
  tracklet_depth_param.fromFile(trackletDepthConfigFileName);

  nav_msgs::Odometry odomGT;
  odomGT.header.frame_id = "/camera_init";
  odomGT.child_frame_id = "/ground_truth";

  nav_msgs::Path pathGT;
  pathGT.header.frame_id = "/camera_init";

// Add camera calibration data.

  // Prepare the pose rotation that converts the world frame into the camera
  // optical frame.
  Eigen::Matrix3d R_transform;
  R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Quaterniond q_transform(R_transform);

  // If saving the result into a bag file, save it into the output file.
  rosbag::Bag bag_out;
  if (to_bag) bag_out.open(output_bag_file, rosbag::bagmode::Write);

  std::string line;
  std::size_t line_num = 0;
  ros::Rate r(10.0);
  while (std::getline(timestamp_file, line) && ros::ok()) {
    float timestamp = stof(line);
    ros::Time timestamp_ros = ros::Time().fromSec(timestamp);

    // Read the images.
    std::stringstream left_image_path, right_image_path;
    left_image_path << dataset_folder
                    << "sequences/" + sequence_number + "/image_0/"
                    << std::setfill('0') << std::setw(6) << line_num << ".png";
    cv::Mat left_image =
        cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
    right_image_path << dataset_folder
                     << "sequences/" + sequence_number + "/image_1/"
                     << std::setfill('0') << std::setw(6) << line_num << ".png";
    cv::Mat right_image =
        cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);

    cv::imshow("Original", left_image);

    /*****************************************************************
                        Image Brightness Preprocessing
    ******************************************************************/
    cv::Mat left_processed;
    corrector.processImage(left_image, left_processed);

    cv::imshow("Processed", left_processed);

    /*****************************************************************
                        Feature Computation and Matching
    ******************************************************************/
    feature_tracker_module.process(left_processed, timestamp_ros);
    matches_msg_ros::MatchesMsg match_msg =
        feature_tracker_module.get_matches_msg();
    feature_tracker_module.reduce();

    // Read the lidar point cloud.
    std::stringstream lidar_data_path;
    lidar_data_path << dataset_folder
                    << "sequences/" + sequence_number + "/velodyne/"
                    << std::setfill('0') << std::setw(6) << line_num << ".bin";

    std::vector<Eigen::Vector3d> lidar_points;
    std::vector<float> lidar_intensities;
    pcl::PointCloud<pcl::PointXYZI> laser_cloud;
    if (!read_lidar_data(lidar_data_path.str(), lidar_points, lidar_intensities,
                         laser_cloud)) {
      printf("Failed to read the lidar data from the path %s\n",
             lidar_data_path.str().c_str());
      return -1;
    }
    sensor_msgs::PointCloud2 laser_cloud_msg;
    pcl::toROSMsg(laser_cloud, laser_cloud_msg);
    laser_cloud_msg.header.stamp = timestamp_ros;
    laser_cloud_msg.header.frame_id = "/camera_init";

    /*****************************************************************
                        Depth Estimation
    ******************************************************************/

    /*****************************************************************
                        Match Outlier Filtering
    ******************************************************************/

    /*****************************************************************
                        Semantic Segmentation
    ******************************************************************/

    /*****************************************************************
                        Bundle Adjustment
    ******************************************************************/

    cv::waitKey(1);
    std::getline(ground_truth_file, line);

    if (to_bag) {
      // Read the ground truth pose data from the file.
      if (!read_gt_pose(line, q_transform, timestamp, odomGT, pathGT)) {
        printf("Failed to read the ground truth pose data\n");
        return -1;
      }

      sensor_msgs::ImagePtr image_left_msg =
          cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image)
              .toImageMsg();
      sensor_msgs::ImagePtr image_right_msg =
          cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image)
              .toImageMsg();

      bag_out.write("/image_left", ros::Time::now(), image_left_msg);
      bag_out.write("/image_right", ros::Time::now(), image_right_msg);
      bag_out.write("/kitti/velo/pointcloud", ros::Time::now(),
                    laser_cloud_msg);
      bag_out.write("/path_gt", ros::Time::now(), pathGT);
      bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
    }

    line_num++;
    r.sleep();
  }
  bag_out.close();

  return 0;
}
