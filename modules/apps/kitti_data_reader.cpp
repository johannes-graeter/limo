#include "kitti_data_reader.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iterator>

KittiDataReader::KittiDataReader(const std::string& base_path) : base_path_(base_path) {
    scan_files();
    // Assume calibration files are in base_path_/calib.txt
    K_ = read_intrinsics((base_path_ / "calib.txt").string());
    Tr_ = read_extrinsics((base_path_ / "calib.txt").string());
}

void KittiDataReader::scan_files() {
    auto img_dir = base_path_ / "image_2";
    auto pc_dir = base_path_ / "velodyne";
    for (const auto& entry : std::filesystem::directory_iterator(img_dir)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg")
            image_files_.push_back(entry.path().string());
    }
    for (const auto& entry : std::filesystem::directory_iterator(pc_dir)) {
        if (entry.path().extension() == ".bin")
            cloud_files_.push_back(entry.path().string());
    }
    std::sort(image_files_.begin(), image_files_.end());
    std::sort(cloud_files_.begin(), cloud_files_.end());
}

Eigen::Matrix3d KittiDataReader::read_intrinsics(const std::string& calib_file) {
    std::ifstream in(calib_file);
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("P2:", 0) == 0) {
            std::istringstream iss(line.substr(3));
            double vals[12];
            for (double& v : vals) iss >> v;
            Eigen::Matrix3d K;
            K << vals[0], vals[1], vals[2],
                 vals[4], vals[5], vals[6],
                 vals[8], vals[9], vals[10];
            return K;
        }
    }
    throw std::runtime_error("No P2 line in calibration file");
}

Eigen::Matrix4d KittiDataReader::read_extrinsics(const std::string& calib_file) {
    std::ifstream in(calib_file);
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("Tr:", 0) == 0 || line.rfind("Tr_velo_to_cam:", 0) == 0) {
            std::istringstream iss(line.substr(line.find(":")+1));
            double vals[12];
            for (double& v : vals) iss >> v;
            Eigen::Matrix4d Tr = Eigen::Matrix4d::Identity();
            Tr.block<3,4>(0,0) = Eigen::Map<Eigen::Matrix<double,3,4,Eigen::RowMajor>>(vals);
            return Tr;
        }
    }
    throw std::runtime_error("No Tr or Tr_velo_to_cam line in calibration file");
}

cv::Mat KittiDataReader::read_image(const std::string& img_path) {
    return cv::imread(img_path, cv::IMREAD_UNCHANGED);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KittiDataReader::read_pointcloud(const std::string& bin_path) {
    std::ifstream input(bin_path, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open pointcloud file");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    while (input) {
        float data[4];
        input.read(reinterpret_cast<char*>(data), sizeof(data));
        if (input.gcount() < sizeof(data)) break;
        pcl::PointXYZI pt;
        pt.x = data[0]; pt.y = data[1]; pt.z = data[2]; pt.intensity = data[3];
        cloud->push_back(pt);
    }
    return cloud;
}

KittiDataReader::FrameGenerator KittiDataReader::frames() {
    struct Awaitable {
        size_t idx = 0;
        KittiDataReader* reader;
        bool await_ready() const noexcept { return false; }
        void await_suspend(std::coroutine_handle<FrameGenerator::promise_type> h) {
            if (idx < reader->image_files_.size() && idx < reader->cloud_files_.size()) {
                KittiFrame frame;
                frame.K = reader->K_;
                frame.Tr = reader->Tr_;
                frame.image = read_image(reader->image_files_[idx]);
                frame.cloud = read_pointcloud(reader->cloud_files_[idx]);
                frame.index = idx;
                h.promise().yield_value(std::move(frame));
                ++idx;
                h.resume();
            }
        }
        void await_resume() const noexcept {}
    };
    for (size_t i = 0; i < std::min(image_files_.size(), cloud_files_.size()); ++i) {
        co_yield KittiFrame{K_, Tr_, read_image(image_files_[i]), read_pointcloud(cloud_files_[i]), i};
    }
}
