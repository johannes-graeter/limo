#pragma once

#include <coroutine>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/// One synchronized KITTI odometry frame.
struct KittiFrame {
    Eigen::Matrix3d K;                              ///< intrinsics of the used camera (P2)
    Eigen::Affine3d transform_velodyne_to_camera;   ///< maps velodyne points into that camera's frame
    cv::Mat image;                                  ///< color image (image_2)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;     ///< velodyne scan
    uint64_t timestamp_ns;                          ///< from times.txt (frame_index * 0.1s if missing)
    size_t index;
};

/// Reads a KITTI odometry sequence directory (image_2/, velodyne/, calib.txt, times.txt and
/// optionally the ground-truth poses file).
class KittiDataReader {
public:
    explicit KittiDataReader(const std::string& sequence_dir);

    struct FrameGenerator {
        struct promise_type {
            KittiFrame current_frame;
            std::suspend_always yield_value(KittiFrame frame) {
                current_frame = std::move(frame);
                return {};
            }
            FrameGenerator get_return_object() {
                return FrameGenerator{std::coroutine_handle<promise_type>::from_promise(*this)};
            }
            std::suspend_always initial_suspend() {
                return {};
            }
            std::suspend_always final_suspend() noexcept {
                return {};
            }
            void return_void() {
            }
            void unhandled_exception() {
                throw;
            }
        };
        using handle_type = std::coroutine_handle<promise_type>;
        handle_type coro;
        explicit FrameGenerator(handle_type h) : coro(h) {
        }
        ~FrameGenerator() {
            if (coro) {
                coro.destroy();
            }
        }
        FrameGenerator(const FrameGenerator&) = delete;
        FrameGenerator& operator=(const FrameGenerator&) = delete;
        FrameGenerator(FrameGenerator&& other) noexcept : coro(other.coro) {
            other.coro = nullptr;
        }
        FrameGenerator& operator=(FrameGenerator&& other) noexcept {
            if (this != &other) {
                if (coro) {
                    coro.destroy();
                }
                coro = other.coro;
                other.coro = nullptr;
            }
            return *this;
        }
        bool next() {
            coro.resume();
            return !coro.done();
        }
        KittiFrame& value() {
            return coro.promise().current_frame;
        }
    };

    /// Lazily yields the frames of the sequence in order.
    FrameGenerator frames();

    size_t frameCount() const {
        return image_files_.size();
    }

    const Eigen::Matrix3d& intrinsics() const {
        return K_;
    }

    /// Transform mapping velodyne points into the frame of camera 2 (Tr with the P2 baseline
    /// offset applied).
    const Eigen::Affine3d& transformVelodyneToCamera() const {
        return transform_velodyne_to_camera_;
    }

    /// Ground-truth poses (camera 0, pose of frame i in the frame of frame 0), if a poses file
    /// `poses.txt` or `<sequence>.txt` exists next to the sequence data.
    const std::vector<Eigen::Isometry3d>& groundTruthPoses() const {
        return ground_truth_poses_;
    }

    static cv::Mat readImage(const std::string& image_path);
    static pcl::PointCloud<pcl::PointXYZI>::Ptr readPointcloud(const std::string& bin_path);

private:
    void scanFiles();
    void readCalibration();
    void readTimes();
    void readGroundTruthPoses();

    std::filesystem::path sequence_dir_;
    std::vector<std::string> image_files_;
    std::vector<std::string> cloud_files_;
    std::vector<uint64_t> timestamps_ns_;
    std::vector<Eigen::Isometry3d> ground_truth_poses_;
    Eigen::Matrix3d K_;
    Eigen::Affine3d transform_velodyne_to_camera_;
};
