#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>
#include <string>
#include <vector>

bool read_lidar_data(const std::string lidar_data_path,
                     std::vector<Eigen::Vector3d> &lidar_points,
                     std::vector<float> &lidar_intensities,
                     pcl::PointCloud<pcl::PointXYZI> &laser_cloud) {
  std::ifstream lidar_data_file(lidar_data_path,
                                std::ifstream::in | std::ifstream::binary);
  if (!lidar_data_file.is_open()) {
    return false;
  }
  lidar_data_file.seekg(0, std::ios::end);
  const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
  lidar_data_file.seekg(0, std::ios::beg);

  std::vector<float> lidar_data_buffer(num_elements);
  lidar_data_file.read(reinterpret_cast<char *>(&lidar_data_buffer[0]),
                       num_elements * sizeof(float));

  for (std::size_t i = 0; i < lidar_data_buffer.size(); i += 4) {
    pcl::PointXYZI point;
    point.x = lidar_data_buffer[i];
    point.y = lidar_data_buffer[i + 1];
    point.z = lidar_data_buffer[i + 2];
    point.intensity = lidar_data_buffer[i + 3];
    laser_cloud.push_back(point);

    lidar_points.emplace_back(point.x, point.y, point.z);
    lidar_intensities.push_back(point.intensity);
  }
  return true;
}

bool read_gt_pose(const std::string &value_string,
                  const Eigen::Quaterniond &q_transform, float timestamp,
                  nav_msgs::Odometry &odomGT, nav_msgs::Path &pathGT) {
  std::stringstream pose_stream(value_string);
  std::string s;
  // Eigen::Matrix<double, 3, 4> gt_pose;
  Eigen::Affine3d gt_pose;
  geometry_msgs::PoseStamped poseGT;

  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 4; ++j) {
      std::getline(pose_stream, s, ' ');
      if (s.empty()) return false;
      gt_pose.matrix()(i, j) = stof(s);
    }
  }

  Eigen::Quaterniond q_w_i(gt_pose.matrix().topLeftCorner<3, 3>());
  Eigen::Quaterniond q = q_transform * q_w_i;
  q.normalize();
  Eigen::Vector3d t = q_transform * gt_pose.matrix().topRightCorner<3, 1>();

  odomGT.header.stamp = ros::Time().fromSec(timestamp);
  odomGT.pose.pose.orientation.x = q.x();
  odomGT.pose.pose.orientation.y = q.y();
  odomGT.pose.pose.orientation.z = q.z();
  odomGT.pose.pose.orientation.w = q.w();
  odomGT.pose.pose.position.x = t(0);
  odomGT.pose.pose.position.y = t(1);
  odomGT.pose.pose.position.z = t(2);

  poseGT.header = odomGT.header;
  poseGT.pose = odomGT.pose.pose;
  pathGT.header.stamp = odomGT.header.stamp;
  pathGT.poses.push_back(poseGT);
  return true;
}