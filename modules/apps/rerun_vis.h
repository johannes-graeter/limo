#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rerun.hpp>

namespace rr_vis {

/// Visualization sink for the LIMO pipeline, backed by rerun.
///
/// Sink mode is chosen at construction:
///   "spawn"          spawn a local rerun viewer and stream to it
///   "connect"        connect to a viewer at the default gRPC url
///   "connect:<url>"  connect to a viewer at <url>
///   "save:<path>"    write a .rrd recording to <path>
///   "off"            all log calls become no-ops
class RerunVisualizer {
public:
    RerunVisualizer(const std::string& app_id, const std::string& mode);

    bool enabled() const {
        return stream_.has_value();
    }

    /// Stamp all subsequent log calls with the given frame index and sequence time.
    void setFrame(int64_t frame_index, uint64_t timestamp_ns) const;

    /// Log a cv::Mat image (CV_8UC3 is interpreted as BGR, CV_8UC1 as grayscale).
    void logImage(std::string_view entity, const cv::Mat& image) const;

    /// Log a pinhole camera model with full intrinsics K and image resolution.
    void logCamera(std::string_view entity, const Eigen::Matrix3d& intrinsics, int width, int height) const;

    /// Log the pose of an entity in the world frame.
    void logPose(std::string_view entity, const Eigen::Isometry3d& world_from_entity) const;

    /// Log a trajectory as a single 3d line strip.
    void logTrajectory(std::string_view entity, const std::vector<Eigen::Vector3d>& positions) const;

    /// Log 3d points with one rgba color each. If colors is empty, points are white.
    void logPoints(std::string_view entity,
                   const std::vector<Eigen::Vector3d>& points,
                   const std::vector<std::array<uint8_t, 4>>& colors_rgba = {},
                   float radius = 0.15f) const;

    /// Log a lidar point cloud, colored by intensity. If a transform is given, the points are
    /// mapped through it first (e.g. velodyne -> camera to log under the camera entity).
    void logPointCloud(std::string_view entity,
                       const pcl::PointCloud<pcl::PointXYZI>& cloud,
                       const Eigen::Affine3d& transform = Eigen::Affine3d::Identity()) const;

private:
    std::optional<rerun::RecordingStream> stream_;
};

} // namespace rr_vis
