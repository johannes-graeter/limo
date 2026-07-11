#include "rerun_vis.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace {

/// Column-major float array from a row-major Eigen 3x3 matrix.
std::array<float, 9> toFlatColumns(const Eigen::Matrix3d& m) {
    std::array<float, 9> cols{};
    for (int c = 0; c < 3; ++c) {
        for (int r = 0; r < 3; ++r) {
            cols[static_cast<size_t>(c * 3 + r)] = static_cast<float>(m(r, c));
        }
    }
    return cols;
}

} // namespace

namespace rr_vis {

RerunVisualizer::RerunVisualizer(const std::string& app_id, const std::string& mode) {
    if (mode == "off") {
        return;
    }

    stream_.emplace(app_id);
    if (mode == "spawn") {
        stream_->spawn().exit_on_failure();
    } else if (mode == "connect") {
        stream_->connect_grpc().exit_on_failure();
    } else if (mode.starts_with("connect:")) {
        stream_->connect_grpc(mode.substr(std::string_view{"connect:"}.size())).exit_on_failure();
    } else if (mode.starts_with("save:")) {
        stream_->save(mode.substr(std::string_view{"save:"}.size())).exit_on_failure();
    } else {
        throw std::invalid_argument("Unknown rerun mode '" + mode +
                                    "', expected spawn|connect[:<url>]|save:<path>|off");
    }

    // KITTI world frame == camera frame of the first keyframe: x right, y down, z forward.
    stream_->log_static("world", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);
}

void RerunVisualizer::setFrame(int64_t frame_index, uint64_t timestamp_ns) const {
    if (!stream_) {
        return;
    }
    stream_->set_time_sequence("frame", frame_index);
    stream_->set_time_duration_nanos("time", static_cast<int64_t>(timestamp_ns));
}

void RerunVisualizer::logImage(std::string_view entity, const cv::Mat& image) const {
    if (!stream_ || image.empty()) {
        return;
    }
    const cv::Mat contiguous = image.isContinuous() ? image : image.clone();
    const auto width = static_cast<uint32_t>(contiguous.cols);
    const auto height = static_cast<uint32_t>(contiguous.rows);
    const size_t num_bytes = contiguous.total() * contiguous.elemSize();

    switch (contiguous.type()) {
    case CV_8UC3:
        stream_->log(entity,
                     rerun::Image(rerun::borrow(contiguous.data, num_bytes),
                                  {width, height},
                                  rerun::ColorModel::BGR));
        break;
    case CV_8UC1:
        stream_->log(entity,
                     rerun::Image(rerun::borrow(contiguous.data, num_bytes),
                                  {width, height},
                                  rerun::ColorModel::L));
        break;
    default:
        throw std::invalid_argument("logImage supports CV_8UC3 (BGR) and CV_8UC1 images only");
    }
}

void RerunVisualizer::logCamera(std::string_view entity,
                                const Eigen::Matrix3d& intrinsics,
                                int width,
                                int height) const {
    if (!stream_) {
        return;
    }
    stream_->log(entity,
                 rerun::Pinhole(rerun::components::PinholeProjection(toFlatColumns(intrinsics)))
                     .with_resolution(static_cast<float>(width), static_cast<float>(height))
                     .with_image_plane_distance(2.0f));
}

void RerunVisualizer::logPose(std::string_view entity, const Eigen::Isometry3d& world_from_entity) const {
    if (!stream_) {
        return;
    }
    const Eigen::Vector3d& t = world_from_entity.translation();
    stream_->log(entity,
                 rerun::Transform3D(
                     rerun::components::Translation3D(
                         static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z())),
                     rerun::components::TransformMat3x3(toFlatColumns(world_from_entity.rotation()))));
}

void RerunVisualizer::logTrajectory(std::string_view entity,
                                    const std::vector<Eigen::Vector3d>& positions) const {
    if (!stream_ || positions.empty()) {
        return;
    }
    std::vector<rerun::datatypes::Vec3D> strip;
    strip.reserve(positions.size());
    for (const auto& p : positions) {
        strip.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }
    stream_->log(entity,
                 rerun::LineStrips3D(rerun::components::LineStrip3D(strip))
                     .with_colors(rerun::Color(0, 200, 255))
                     .with_radii(0.08f));
}

void RerunVisualizer::logPoints(std::string_view entity,
                                const std::vector<Eigen::Vector3d>& points,
                                const std::vector<std::array<uint8_t, 4>>& colors_rgba,
                                float radius) const {
    if (!stream_ || points.empty()) {
        return;
    }
    std::vector<rerun::Position3D> positions;
    positions.reserve(points.size());
    for (const auto& p : points) {
        positions.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
    }

    auto archetype = rerun::Points3D(positions).with_radii(radius);
    if (!colors_rgba.empty()) {
        std::vector<rerun::Color> colors;
        colors.reserve(colors_rgba.size());
        for (const auto& c : colors_rgba) {
            colors.emplace_back(c[0], c[1], c[2], c[3]);
        }
        archetype = std::move(archetype).with_colors(colors);
    }
    stream_->log(entity, archetype);
}

void RerunVisualizer::logPointCloud(std::string_view entity,
                                    const pcl::PointCloud<pcl::PointXYZI>& cloud) const {
    if (!stream_ || cloud.empty()) {
        return;
    }
    std::vector<rerun::Position3D> positions;
    std::vector<rerun::Color> colors;
    positions.reserve(cloud.size());
    colors.reserve(cloud.size());
    for (const auto& pt : cloud.points) {
        positions.emplace_back(pt.x, pt.y, pt.z);
        // KITTI intensities are in [0, 1]; map to a gray ramp with a visible floor.
        const auto gray = static_cast<uint8_t>(55.f + 200.f * std::clamp(pt.intensity, 0.f, 1.f));
        colors.emplace_back(gray, gray, gray);
    }
    stream_->log(entity, rerun::Points3D(positions).with_colors(colors).with_radii(0.03f));
}

} // namespace rr_vis
