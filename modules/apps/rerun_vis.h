#pragma once
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <rerun.hpp>


#include <Eigen/Dense>

namespace rr_vis {


    class RerunVisualizer{
        public:
        RerunVisualizer(const std::string& name);

        // Log an image as a 3D textured plane using camera intrinsics (K) and extrinsics (Tr)
        void log(const std::string& name, const cv::Mat& image, const Eigen::Matrix3d& K, const Eigen::Matrix4d& Tr);

        // Log a pointcloud (XYZ or XYZI)
        void log(const std::string& name, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

    private:
        rerun::RecordingStream recording_stream_;
    };



}
