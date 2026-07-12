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

/// Runs the real limo_with_kitti binary on a synthetic sequence with known motion and checks the
/// recovered trajectory — including metric scale, which must come from the lidar depth.
void runAndCheckTrajectory(const std::string& extra_args,
                           const std::string& test_dir_name,
                           double per_pose_tolerance = 0.8,
                           double final_error_fraction = 0.03) {
    const auto base_dir = std::filesystem::temp_directory_path() / test_dir_name;
    const auto sequence_dir = base_dir / "sequence";
    const auto output_dir = base_dir / "output";
    std::filesystem::remove_all(base_dir);

    synthetic_kitti::Options options;
    options.frame_count = 25;
    options.with_semantic_images = true;
    const auto ground_truth = synthetic_kitti::generateSequence(sequence_dir, options);

    const std::string command = std::string(LIMO_WITH_KITTI_BINARY) + " " + sequence_dir.string() +
                                " --rerun off --output " + output_dir.string() + " " + extra_args + " > " +
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
        EXPECT_LT(position_error, per_pose_tolerance) << "at frame " << i;
    }
    const double final_error = (estimated.back().translation() - ground_truth.back().translation()).norm();
    EXPECT_LT(final_error, final_error_fraction * trajectory_length);

    std::filesystem::remove_all(base_dir);
}

TEST(LimoEndToEnd, recoversTrajectoryOfSyntheticSequence) {
    runAndCheckTrajectory("", "limo_e2e");
}

/// Same sequence with the semantic input: labels weight the landmarks (vegetation wall down)
/// and the ground plane comes from the labeled road pixels instead of RANSAC.
TEST(LimoEndToEnd, recoversTrajectoryWithSemanticImages) {
    runAndCheckTrajectory("--semantic " +
                              (std::filesystem::temp_directory_path() / "limo_e2e_semantic" / "sequence" /
                               "semantic_2")
                                  .string(),
                          "limo_e2e_semantic");
}

/// Camera-only: no lidar at all. The metric scale must come from the camera height (1.5m in the
/// synthetic scene) over the semantically labeled ground plane — looser tolerances, since the
/// ground plane carries the scale alone.
TEST(LimoEndToEnd, recoversTrajectoryMonoWithSemanticImages) {
    runAndCheckTrajectory("--mono --camera-height 1.5 --semantic " +
                              (std::filesystem::temp_directory_path() / "limo_e2e_mono" / "sequence" / "semantic_2")
                                  .string(),
                          "limo_e2e_mono",
                          /*per_pose_tolerance=*/2.0,
                          /*final_error_fraction=*/0.10);
}

} // namespace
