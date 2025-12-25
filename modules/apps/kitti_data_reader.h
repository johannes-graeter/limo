#pragma once
#include <string>
#include <vector>
#include <filesystem>
#include <coroutine>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

struct KittiFrame {
    Eigen::Matrix3d K; // Intrinsic
    Eigen::Matrix4d Tr; // Extrinsic
    cv::Mat image;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    size_t index;
};

class KittiDataReader {
public:
    KittiDataReader(const std::string& base_path);
    
    struct FrameGenerator {
        struct promise_type {
            KittiFrame current_frame;
            std::suspend_always yield_value(KittiFrame frame) {
                current_frame = std::move(frame);
                return {};
            }
            FrameGenerator get_return_object() { return FrameGenerator{std::coroutine_handle<promise_type>::from_promise(*this)}; }
            std::suspend_always initial_suspend() { return {}; }
            std::suspend_always final_suspend() noexcept { return {}; }
            void return_void() {}
            void unhandled_exception() { std::terminate(); }
        };
        using handle_type = std::coroutine_handle<promise_type>;
        handle_type coro;
        FrameGenerator(handle_type h) : coro(h) {}
        ~FrameGenerator() { if (coro) coro.destroy(); }
        FrameGenerator(const FrameGenerator&) = delete;
        FrameGenerator& operator=(const FrameGenerator&) = delete;
        FrameGenerator(FrameGenerator&& other) noexcept : coro(other.coro) { other.coro = nullptr; }
        FrameGenerator& operator=(FrameGenerator&& other) noexcept { if (this != &other) { if (coro) coro.destroy(); coro = other.coro; other.coro = nullptr; } return *this; }
        bool next() { coro.resume(); return !coro.done(); }
        KittiFrame& value() { return coro.promise().current_frame; }
    };

    FrameGenerator frames();

    static Eigen::Matrix3d read_intrinsics(const std::string& calib_file);
    static Eigen::Matrix4d read_extrinsics(const std::string& calib_file);
    static cv::Mat read_image(const std::string& img_path);
    static pcl::PointCloud<pcl::PointXYZI>::Ptr read_pointcloud(const std::string& bin_path);

private:
    std::filesystem::path base_path_;
    std::vector<std::string> image_files_;
    std::vector<std::string> cloud_files_;
    Eigen::Matrix3d K_;
    Eigen::Matrix4d Tr_;
    void scan_files();
};
