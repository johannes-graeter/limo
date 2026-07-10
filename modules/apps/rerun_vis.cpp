
#include "rerun_vis.h"
#include <rerun.hpp>
#include <vector>

namespace rr_vis {

RerunVisualizer::RerunVisualizer(const std::string& name) : rerun::RecordingStream(name)
{
    rec.spawn().exit_on_failure();
}
    // Log an image as a 3D textured plane using camera intrinsics and extrinsics
    void RerunVisualizer::log(const std::string& name, const cv::Mat& image, const Eigen::Matrix3d& K, const Eigen::Matrix4d& Tr) {
        int w = image.cols, h = image.rows;
        // 4 corners in image pixel coordinates
        std::vector<Eigen::Vector3f> img_corners = {
            (K.inverse() * Eigen::Vector3d(0,   0,   1)).cast<float>(), // top-left
            (K.inverse() * Eigen::Vector3d(w,   0,   1)).cast<float>(), // top-right
            (K.inverse() * Eigen::Vector3d(w,   h,   1)).cast<float>(), // bottom-right
            (K.inverse() * Eigen::Vector3d(0,   h,   1)).cast<float>()  // bottom-left
        };
        // Transform corners to world
        std::vector<Eigen::Vector3f> world_corners;
        for (const auto& c : img_corners) {
            Eigen::Vector4d cam_pt; cam_pt << c.cast<double>(), 1.0;
            Eigen::Vector4d world_pt = Tr * cam_pt;
            world_corners.emplace_back(world_pt.x(), world_pt.y(), world_pt.z());
        }
        // Log as a Mesh3D with texture
        recording_stream_.log(name,
            rerun::Mesh3D(world_corners)
            .with_indices({0, 1, 2, 0, 2, 3})
            .with_texture(image.data, w, h, image.channels()));
    }

    void RerunVisualizer::log(const std::string& name, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
        std::vector<Eigen::Vector3f> points;
        std::vector<float> intensities;
        for (const auto& pt : cloud->points) {
            points.emplace_back(pt.x, pt.y, pt.z);
            intensities.push_back(pt.intensity);
        }
        recording_stream_.log(name,
            rerun::Points3D(points)
            .with_colors_rgba(std::vector<uint8_t>(points.size() * 4, 255)) // white
            .with_radii(std::vector<float>(points.size(), 0.05f)));
    }
}
