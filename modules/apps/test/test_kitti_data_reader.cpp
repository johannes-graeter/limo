#include <cstdlib>
#include <filesystem>

#include <gtest/gtest.h>

#include "../kitti_data_reader.h"
#include "synthetic_kitti.h"

namespace {

class KittiDataReaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        sequence_dir_ = std::filesystem::temp_directory_path() / "limo_test_kitti_reader";
        std::filesystem::remove_all(sequence_dir_);

        synthetic_kitti::Options options;
        options.frame_count = 3;
        ground_truth_ = synthetic_kitti::generateSequence(sequence_dir_, options);
    }

    void TearDown() override {
        std::filesystem::remove_all(sequence_dir_);
    }

    std::filesystem::path sequence_dir_;
    std::vector<Eigen::Isometry3d> ground_truth_;
};

TEST_F(KittiDataReaderTest, readsCalibrationTimesAndGroundTruth) {
    KittiDataReader reader(sequence_dir_.string());

    EXPECT_EQ(reader.frameCount(), 3u);
    EXPECT_DOUBLE_EQ(reader.intrinsics()(0, 0), 320.);
    EXPECT_DOUBLE_EQ(reader.intrinsics()(0, 2), 320.);

    ASSERT_EQ(reader.groundTruthPoses().size(), 3u);
    EXPECT_TRUE(reader.groundTruthPoses()[0].isApprox(ground_truth_[0], 1e-9));
    EXPECT_TRUE(reader.groundTruthPoses()[2].isApprox(ground_truth_[2], 1e-9));
}

TEST_F(KittiDataReaderTest, yieldsAllFramesWithImageAndCloud) {
    KittiDataReader reader(sequence_dir_.string());

    size_t count = 0;
    for (auto gen = reader.frames(); gen.next();) {
        const KittiFrame& frame = gen.value();
        EXPECT_EQ(frame.index, count);
        EXPECT_FALSE(frame.image.empty());
        EXPECT_EQ(frame.image.cols, 640);
        EXPECT_EQ(frame.image.rows, 192);
        EXPECT_GT(frame.cloud->size(), 100u);
        EXPECT_EQ(frame.timestamp_ns, count * 100000000ull);
        ++count;
    }
    EXPECT_EQ(count, 3u);
}

TEST_F(KittiDataReaderTest, velodyneCloudProjectsIntoImage) {
    // The velodyne->camera transform must map cloud points in front of the camera; project a few
    // and check they land inside the image.
    KittiDataReader reader(sequence_dir_.string());
    auto gen = reader.frames();
    ASSERT_TRUE(gen.next());
    const KittiFrame& frame = gen.value();

    int in_image = 0;
    int total = 0;
    for (size_t i = 0; i < frame.cloud->size(); i += 50) {
        const auto& point = frame.cloud->points[i];
        const Eigen::Vector3d p_cam =
            frame.transform_velodyne_to_camera * Eigen::Vector3d(point.x, point.y, point.z);
        ++total;
        if (p_cam.z() <= 0.) {
            continue;
        }
        const double u = frame.K(0, 0) * p_cam.x() / p_cam.z() + frame.K(0, 2);
        const double v = frame.K(1, 1) * p_cam.y() / p_cam.z() + frame.K(1, 2);
        if (u >= 0. && u < frame.image.cols && v >= 0. && v < frame.image.rows) {
            ++in_image;
        }
    }
    // The synthetic corridor is mostly in front of the camera.
    EXPECT_GT(in_image, total / 4);
}

TEST(KittiDataReader, throwsOnMissingDirectory) {
    EXPECT_THROW(KittiDataReader("/nonexistent/sequence"), std::runtime_error);
}

} // namespace
