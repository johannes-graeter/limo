#pragma once
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace rr_vis {
    void log_image(const std::string& name, const cv::Mat& image);

    void log_pointcloud(const std::string& name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
}
