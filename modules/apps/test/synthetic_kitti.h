#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace synthetic_kitti {

/// Cityscapes label ids used by the generated scene.
inline constexpr uint8_t kLabelRoad = 7;
inline constexpr uint8_t kLabelBuilding = 11;
inline constexpr uint8_t kLabelVegetation = 21;

/// Configuration of the generated sequence.
struct Options {
    int frame_count{25};
    int image_width{640};
    int image_height{192};
    double focal_length{320.};
    double forward_motion_per_frame{0.8}; ///< camera motion along z per frame, meters
    double yaw_rate_per_frame{0.};        ///< rotation around y per frame, radians
    unsigned int seed{42};
    bool with_semantic_images{false};     ///< also write semantic_2/ label images (cityscapes ids)
};

/// Generates a synthetic KITTI-odometry-format sequence (image_2/*.png, velodyne/*.bin,
/// calib.txt, times.txt, poses.txt) into the given directory.
///
/// The scene is a corridor (ground plane + two side walls) covered with high-contrast dots that
/// the viso feature tracker can follow; the velodyne clouds sample the same surfaces densely.
/// The camera moves with the configured constant motion; the world frame is the camera frame of
/// the first image.
///
/// @return ground-truth poses (pose of frame i in the frame of frame 0)
std::vector<Eigen::Isometry3d> generateSequence(const std::filesystem::path& sequence_dir, const Options& options);

} // namespace synthetic_kitti
