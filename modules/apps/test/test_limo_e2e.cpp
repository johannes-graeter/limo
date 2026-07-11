#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "synthetic_kitti.h"

namespace {

std::vector<Eigen::Isometry3d> readKittiPoses(const std::filesystem::path& file_path) {
    std::ifstream in(file_path);
    std::vector<Eigen::Isometry3d> poses;
    std::array<double, 12> values{};
    while (in >> values[0] >> values[1] >> values[2] >> values[3] >> values[4] >> values[5] >> values[6] >>
           values[7] >> values[8] >> values[9] >> values[10] >> values[11]) {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.matrix().block<3, 4>(0, 0) =
            Eigen::Map<const Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(values.data());
        poses.push_back(pose);
    }
    return poses;
}

/// End-to-end: run the real limo_with_kitti binary on a synthetic sequence with known motion and
/// check the recovered trajectory — including metric scale, which must come from the lidar depth.
TEST(LimoEndToEnd, recoversTrajectoryOfSyntheticSequence) {
    const auto base_dir = std::filesystem::temp_directory_path() / "limo_e2e";
    const auto sequence_dir = base_dir / "sequence";
    const auto output_dir = base_dir / "output";
    std::filesystem::remove_all(base_dir);

    synthetic_kitti::Options options;
    options.frame_count = 25;
    const auto ground_truth = synthetic_kitti::generateSequence(sequence_dir, options);

    const std::string command = std::string(LIMO_WITH_KITTI_BINARY) + " " + sequence_dir.string() +
                                " --rerun off --output " + output_dir.string() + " > " +
                                (base_dir / "log.txt").string() + " 2>&1";
    ASSERT_EQ(std::system(command.c_str()), 0) << "pipeline failed, see " << (base_dir / "log.txt");

    const auto estimated = readKittiPoses(output_dir / "poses.txt");
    ASSERT_EQ(estimated.size(), ground_truth.size());

    const double trajectory_length =
        (ground_truth.back().translation() - ground_truth.front().translation()).norm();
    ASSERT_GT(trajectory_length, 15.);

    // Both trajectories start at the origin; every pose must stay close to ground truth. The
    // final-position bound doubles as a scale check (a mono pipeline without depth would be off
    // by an arbitrary factor).
    for (size_t i = 0; i < estimated.size(); ++i) {
        const double position_error = (estimated[i].translation() - ground_truth[i].translation()).norm();
        EXPECT_LT(position_error, 0.8) << "at frame " << i;
    }
    const double final_error = (estimated.back().translation() - ground_truth.back().translation()).norm();
    EXPECT_LT(final_error, 0.03 * trajectory_length);

    std::filesystem::remove_all(base_dir);
}

} // namespace
