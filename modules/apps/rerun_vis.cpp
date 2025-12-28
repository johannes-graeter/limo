#include "rerun_vis.h"

#include <rerun.hpp>

namespace rr_vis {
    void log_image(const std::string& name, const cv::Mat& image) {
        rerun::Image image_rr(image.data, image.cols, image.rows, rerun::ImageFormat::U8, image.channels());
        rerun::log_image(name.c_str(), image_rr);
    }

    void log_pointcloud(const std::string& name, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
        std::vector<rerun::Point3D> points;
        for (const auto& pt : cloud->points) {
            points.emplace_back(pt.x, pt.y, pt.z);
        }
        rerun::log_point_cloud(name.c_str(), points);
    }
}
