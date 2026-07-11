#include "kitti_data_reader.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

std::array<double, 12> parseCalibLine(const std::string& values) {
    std::istringstream iss(values);
    std::array<double, 12> result{};
    for (double& v : result) {
        if (!(iss >> v)) {
            throw std::runtime_error("KittiDataReader: malformed calibration line: " + values);
        }
    }
    return result;
}

} // namespace

KittiDataReader::KittiDataReader(const std::string& sequence_dir) : sequence_dir_(sequence_dir) {
    if (!std::filesystem::is_directory(sequence_dir_)) {
        throw std::runtime_error("KittiDataReader: not a directory: " + sequence_dir_.string());
    }
    scanFiles();
    readCalibration();
    readTimes();
    readGroundTruthPoses();
}

void KittiDataReader::scanFiles() {
    const auto image_dir = sequence_dir_ / "image_2";
    const auto cloud_dir = sequence_dir_ / "velodyne";
    if (!std::filesystem::is_directory(image_dir)) {
        throw std::runtime_error("KittiDataReader: missing " + image_dir.string());
    }
    if (!std::filesystem::is_directory(cloud_dir)) {
        throw std::runtime_error("KittiDataReader: missing " + cloud_dir.string());
    }

    for (const auto& entry : std::filesystem::directory_iterator(image_dir)) {
        if (entry.path().extension() == ".png" || entry.path().extension() == ".jpg") {
            image_files_.push_back(entry.path().string());
        }
    }
    for (const auto& entry : std::filesystem::directory_iterator(cloud_dir)) {
        if (entry.path().extension() == ".bin") {
            cloud_files_.push_back(entry.path().string());
        }
    }
    std::sort(image_files_.begin(), image_files_.end());
    std::sort(cloud_files_.begin(), cloud_files_.end());

    if (image_files_.empty()) {
        throw std::runtime_error("KittiDataReader: no images in " + image_dir.string());
    }
    if (image_files_.size() != cloud_files_.size()) {
        throw std::runtime_error("KittiDataReader: image/velodyne count mismatch: " +
                                 std::to_string(image_files_.size()) + " vs " + std::to_string(cloud_files_.size()));
    }
}

void KittiDataReader::readCalibration() {
    const auto calib_file = sequence_dir_ / "calib.txt";
    std::ifstream in(calib_file);
    if (!in) {
        throw std::runtime_error("KittiDataReader: cannot open " + calib_file.string());
    }

    std::optional<std::array<double, 12>> p2;
    std::optional<std::array<double, 12>> tr;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("P2:", 0) == 0) {
            p2 = parseCalibLine(line.substr(3));
        } else if (line.rfind("Tr:", 0) == 0) {
            tr = parseCalibLine(line.substr(3));
        } else if (line.rfind("Tr_velo_to_cam:", 0) == 0) {
            tr = parseCalibLine(line.substr(std::string("Tr_velo_to_cam:").size()));
        }
    }
    if (!p2.has_value() || !tr.has_value()) {
        throw std::runtime_error("KittiDataReader: calib.txt needs P2 and Tr entries");
    }

    K_ << (*p2)[0], (*p2)[1], (*p2)[2], (*p2)[4], (*p2)[5], (*p2)[6], (*p2)[8], (*p2)[9], (*p2)[10];

    Eigen::Affine3d velo_to_cam0 = Eigen::Affine3d::Identity();
    velo_to_cam0.matrix().block<3, 4>(0, 0) = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(tr->data());

    // P2 projects points given in the rectified cam0 frame; its fourth column K*t encodes the
    // baseline offset of camera 2. Apply it so the transform maps velodyne points directly into
    // the frame of the camera whose images we use.
    const Eigen::Vector3d t_cam2((*p2)[3] / K_(0, 0), (*p2)[7] / K_(1, 1), 0.);
    transform_velodyne_to_camera_ = Eigen::Translation3d(t_cam2) * velo_to_cam0;
}

void KittiDataReader::readTimes() {
    const auto times_file = sequence_dir_ / "times.txt";
    std::ifstream in(times_file);
    if (in) {
        double seconds = 0.;
        while (in >> seconds) {
            timestamps_ns_.push_back(static_cast<uint64_t>(seconds * 1e9));
        }
    }
    if (timestamps_ns_.size() < image_files_.size()) {
        // No (or truncated) times.txt: KITTI records at 10 Hz.
        timestamps_ns_.resize(image_files_.size());
        for (size_t i = 0; i < timestamps_ns_.size(); ++i) {
            timestamps_ns_[i] = static_cast<uint64_t>(i) * 100000000ull;
        }
    }
}

void KittiDataReader::readGroundTruthPoses() {
    // Sequence dirs are usually `<dataset>/sequences/XX`; the benchmark ships poses as
    // `<dataset>/poses/XX.txt`. Also accept a plain `poses.txt` inside the sequence dir.
    std::vector<std::filesystem::path> candidates{sequence_dir_ / "poses.txt"};
    const auto sequence_name = sequence_dir_.filename();
    candidates.push_back(sequence_dir_.parent_path().parent_path() / "poses" / (sequence_name.string() + ".txt"));

    for (const auto& candidate : candidates) {
        std::ifstream in(candidate);
        if (!in) {
            continue;
        }
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty()) {
                continue;
            }
            const auto values = parseCalibLine(line);
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.matrix().block<3, 4>(0, 0) =
                Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(values.data());
            ground_truth_poses_.push_back(pose);
        }
        break;
    }
}

cv::Mat KittiDataReader::readImage(const std::string& image_path) {
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (image.empty()) {
        throw std::runtime_error("KittiDataReader: cannot read image " + image_path);
    }
    return image;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KittiDataReader::readPointcloud(const std::string& bin_path) {
    std::ifstream input(bin_path, std::ios::binary);
    if (!input) {
        throw std::runtime_error("KittiDataReader: cannot open pointcloud " + bin_path);
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    float data[4];
    while (input.read(reinterpret_cast<char*>(data), sizeof(data))) {
        pcl::PointXYZI point;
        point.x = data[0];
        point.y = data[1];
        point.z = data[2];
        point.intensity = data[3];
        cloud->push_back(point);
    }
    return cloud;
}

KittiDataReader::FrameGenerator KittiDataReader::frames() {
    for (size_t i = 0; i < image_files_.size(); ++i) {
        co_yield KittiFrame{K_,
                            transform_velodyne_to_camera_,
                            readImage(image_files_[i]),
                            readPointcloud(cloud_files_[i]),
                            timestamps_ns_[i],
                            i};
    }
}
