#include "synthetic_kitti.h"

#include <fstream>
#include <random>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace synthetic_kitti {

namespace {

struct ScenePoint {
    Eigen::Vector3d position; ///< world frame (= camera frame of the first image)
    uint8_t gray;             ///< dot intensity for rendering
    bool is_dot;              ///< dots are rendered into the images; all points go into the cloud
    uint8_t label;            ///< cityscapes label id of the surface the point belongs to
};

/// Corridor: ground plane at y = 1.5 (road), a vegetation wall left and a building wall right at
/// x = +-6, extending along z.
std::vector<ScenePoint> makeScene(const Options& options) {
    std::mt19937 rng(options.seed);
    std::uniform_int_distribution<int> intensity(0, 255);
    std::uniform_real_distribution<double> jitter(-0.15, 0.15);

    const double z_max = options.frame_count * options.forward_motion_per_frame + 40.;
    std::vector<ScenePoint> scene;

    const auto addSurface =
        [&](auto&& to_position, double u_min, double u_max, double dot_step, double lidar_step, uint8_t label) {
            for (double z = 1.; z < z_max; z += dot_step) {
                for (double u = u_min; u <= u_max; u += dot_step) {
                    scene.push_back({to_position(u + jitter(rng), z + jitter(rng)),
                                     static_cast<uint8_t>(intensity(rng)),
                                     true,
                                     label});
                }
            }
            for (double z = 1.; z < z_max; z += lidar_step) {
                for (double u = u_min; u <= u_max; u += lidar_step) {
                    scene.push_back({to_position(u, z), 128, false, label});
                }
            }
        };

    // Ground plane (u = x), left and right wall (u = y).
    addSurface([](double u, double z) { return Eigen::Vector3d(u, 1.5, z); }, -6., 6., 0.8, 0.35, kLabelRoad);
    addSurface([](double u, double z) { return Eigen::Vector3d(-6., u, z); }, -2.5, 1.5, 0.7, 0.35, kLabelVegetation);
    addSurface([](double u, double z) { return Eigen::Vector3d(6., u, z); }, -2.5, 1.5, 0.7, 0.35, kLabelBuilding);

    return scene;
}

/// KITTI-style transform mapping camera points into the velodyne frame and its inverse: the
/// velodyne x axis points along the camera z axis, velodyne z is up (-y camera).
Eigen::Affine3d makeCameraToVelodyne() {
    Eigen::Matrix3d rotation;
    rotation << 0., 0., 1., -1., 0., 0., 0., -1., 0.; // velo <- cam
    Eigen::Affine3d transform(rotation);
    transform.translation() = Eigen::Vector3d(0.27, 0., -0.08);
    return transform;
}

void writeCalib(const std::filesystem::path& file_path, const Options& options, const Eigen::Affine3d& velo_to_cam) {
    std::ofstream out(file_path);
    const double f = options.focal_length;
    const double cu = options.image_width / 2.;
    const double cv = options.image_height / 2.;
    for (const auto* name : {"P0:", "P1:", "P2:", "P3:"}) {
        out << name << " " << f << " 0 " << cu << " 0"
            << " 0 " << f << " " << cv << " 0"
            << " 0 0 1 0\n";
    }
    out << "Tr:";
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 4; ++col) {
            out << " " << velo_to_cam.matrix()(row, col);
        }
    }
    out << "\n";
}

} // namespace

std::vector<Eigen::Isometry3d> generateSequence(const std::filesystem::path& sequence_dir, const Options& options) {
    std::filesystem::create_directories(sequence_dir / "image_2");
    std::filesystem::create_directories(sequence_dir / "velodyne");
    if (options.with_semantic_images) {
        std::filesystem::create_directories(sequence_dir / "semantic_2");
    }

    const auto scene = makeScene(options);
    const Eigen::Affine3d cam_to_velo = makeCameraToVelodyne();
    writeCalib(sequence_dir / "calib.txt", options, cam_to_velo.inverse());

    const double f = options.focal_length;
    const double cu = options.image_width / 2.;
    const double cv = options.image_height / 2.;

    std::vector<Eigen::Isometry3d> poses;
    std::ofstream times(sequence_dir / "times.txt");
    std::ofstream poses_file(sequence_dir / "poses.txt");
    poses_file << std::setprecision(12);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); // pose of frame i in frame 0
    for (int frame = 0; frame < options.frame_count; ++frame) {
        poses.push_back(pose);

        times << frame * 0.1 << "\n";
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 4; ++col) {
                poses_file << pose.matrix()(row, col) << (row == 2 && col == 3 ? "" : " ");
            }
        }
        poses_file << "\n";

        const Eigen::Isometry3d world_to_camera = pose.inverse();

        // Render the dots visible from the current camera pose; the semantic image gets the
        // surface label of every scene point with slightly larger stamps so features detected on
        // dot edges are labeled too.
        cv::Mat image(options.image_height, options.image_width, CV_8UC3, cv::Scalar(90, 90, 90));
        cv::Mat semantic(options.image_height, options.image_width, CV_8UC1, cv::Scalar(0));
        for (const auto& point : scene) {
            const Eigen::Vector3d p_cam = world_to_camera * point.position;
            if (p_cam.z() < 0.5) {
                continue;
            }
            const double u = f * p_cam.x() / p_cam.z() + cu;
            const double v = f * p_cam.y() / p_cam.z() + cv;
            if (u < 2. || u >= options.image_width - 2. || v < 2. || v >= options.image_height - 2.) {
                continue;
            }
            const cv::Point pixel(static_cast<int>(std::lround(u)), static_cast<int>(std::lround(v)));
            const int radius = std::max(1, static_cast<int>(std::lround(6. / p_cam.z() * 4.)));
            if (point.is_dot) {
                const auto gray = point.gray;
                cv::circle(image, pixel, radius, cv::Scalar(gray, gray, gray), cv::FILLED);
            }
            cv::circle(semantic, pixel, radius + 3, cv::Scalar(point.label), cv::FILLED);
        }

        char name[16];
        std::snprintf(name, sizeof(name), "%06d", frame);
        cv::imwrite((sequence_dir / "image_2" / (std::string(name) + ".png")).string(), image);
        if (options.with_semantic_images) {
            cv::imwrite((sequence_dir / "semantic_2" / (std::string(name) + ".png")).string(), semantic);
        }

        // Velodyne cloud: all scene points in front of the camera, in the velodyne frame.
        std::ofstream cloud_file(sequence_dir / "velodyne" / (std::string(name) + ".bin"), std::ios::binary);
        for (const auto& point : scene) {
            const Eigen::Vector3d p_cam = world_to_camera * point.position;
            if (p_cam.z() < 0.5 || p_cam.z() > 120.) {
                continue;
            }
            const Eigen::Vector3d p_velo = cam_to_velo * p_cam;
            const float values[4] = {static_cast<float>(p_velo.x()),
                                     static_cast<float>(p_velo.y()),
                                     static_cast<float>(p_velo.z()),
                                     0.5f};
            cloud_file.write(reinterpret_cast<const char*>(values), sizeof(values));
        }

        // Advance the camera by the constant per-frame motion.
        Eigen::Isometry3d motion = Eigen::Isometry3d::Identity();
        motion.translate(Eigen::Vector3d(0., 0., options.forward_motion_per_frame));
        motion.rotate(Eigen::AngleAxisd(options.yaw_rate_per_frame, Eigen::Vector3d::UnitY()));
        pose = pose * motion;
    }

    return poses;
}

} // namespace synthetic_kitti
