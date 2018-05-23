// google test docs
// wiki page: https://code.google.com/p/googletest/w/list
// primer: https://code.google.com/p/googletest/wiki/V1_7_Primer
// FAQ: https://code.google.com/p/googletest/wiki/FAQ
// advanced guide: https://code.google.com/p/googletest/wiki/V1_7_AdvancedGuide
// samples: https://code.google.com/p/googletest/wiki/V1_7_Samples
//
// List of some basic tests fuctions:
// Fatal assertion                      Nonfatal assertion                   Verifies / Description
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// ASSERT_EQ(expected, actual);         EXPECT_EQ(expected, actual);         expected == actual
// ASSERT_NE(val1, val2);               EXPECT_NE(val1, val2);               val1 != val2
// ASSERT_LT(val1, val2);               EXPECT_LT(val1, val2);               val1 < val2
// ASSERT_LE(val1, val2);               EXPECT_LE(val1, val2);               val1 <= val2
// ASSERT_GT(val1, val2);               EXPECT_GT(val1, val2);               val1 > val2
// ASSERT_GE(val1, val2);               EXPECT_GE(val1, val2);               val1 >= val2
//
// ASSERT_FLOAT_EQ(expected, actual);   EXPECT_FLOAT_EQ(expected, actual);   the two float values
// are almost equal (4 ULPs)
// ASSERT_DOUBLE_EQ(expected, actual);  EXPECT_DOUBLE_EQ(expected, actual);  the two double values
// are almost equal (4 ULPs)
// ASSERT_NEAR(val1, val2, abs_error);  EXPECT_NEAR(val1, val2, abs_error);  the difference between
// val1 and val2 doesn't exceed the given absolute error
//
// Note: more information about ULPs can be found here:
// http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm
//
// Example of two unit test:
// TEST(Math, Add) {
//    ASSERT_EQ(10, 5+ 5);
//}
//
// TEST(Math, Float) {
//	  ASSERT_FLOAT_EQ((10.0f + 2.0f) * 3.0f, 10.0f cpp:1411* 3.0f + 2.0f * 3.0f)
//}
//=======================================================================================================================================================
//#include "bundler.hpp"
#include "bundle_adjuster_keyframes.hpp"
#include "gtest/gtest.h"
#include "internal/cost_functors_ceres.hpp"
#include "internal/local_parameterizations.hpp"
#include "internal/motion_model_regularization.hpp"
#include "internal/triangulator.hpp"

//#include "internal/indexed_histogram.hpp"
//#include "internal/voxel_grid.hpp"

#include <fstream>
#include <keyframe_selection_schemes.hpp>
#include <keyframe_selector.hpp>
#include <random>
#include <Eigen/Eigen>

TEST(Triangulator, process) {
    Eigen::Matrix<double, 3, 1> p;
    p << 1., 1., 3.;

    Eigen::Transform<double, 3, Eigen::Isometry> t, id;
    id.setIdentity();
    t.setIdentity();

    t.translation() << 1., -1., 0.;

    Eigen::Matrix<double, 3, 1> v1 = p / p.norm();
    Eigen::Matrix<double, 3, 1> v2 = t.inverse() * p;
    v2.normalize();

    auto id_p = std::make_shared<Eigen::Isometry3d>(id);
    auto t_p = std::make_shared<Eigen::Isometry3d>(t);

    keyframe_bundle_adjustment::Triangulator<double> triang;
    Eigen::Matrix<double, 3, 1> p_triang = triang.triangulate_rays({std::make_pair(id_p, v1), std::make_pair(t_p, v2)});

    std::cout << "triangulated point=" << p_triang.transpose() << std::endl;

    ASSERT_NEAR((p_triang - p).norm(), 0., 1e-5);
}

TEST(Triangulator, process2) {
    // define groudntruth point
    Eigen::Matrix<double, 3, 1> p_gt(0.5, -1., 3.);

    // define poses
    Eigen::Transform<double, 3, Eigen::Isometry> t1, t0;
    t0.setIdentity();

    t0.translation() << 1., -0.1, 0.5;
    //    t0.rotate(Eigen::AngleAxisd(0.1, Eigen::Vector3d(0., 1., 0.)));

    t1 = t0;
    t1.translate(Eigen::Vector3d(0.5, -0.05, 0.25));
    //    t1.rotate(Eigen::AngleAxisd(0.2, Eigen::Vector3d(0., 1., 0.)));

    std::cout << "pose in 0=\n" << t0.matrix() << std::endl;
    std::cout << "pose in 1=\n" << t1.matrix() << std::endl;

    // calc lines of sight
    Eigen::Vector3d v0 = t0.inverse() * p_gt;
    std::cout << "point in pose 0=" << v0.transpose() << std::endl;
    v0.normalize();

    Eigen::Vector3d v1 = t1.inverse() * p_gt;
    std::cout << "point in pose 1=" << v1.transpose() << std::endl;
    v1.normalize();

    auto id_p = std::make_shared<Eigen::Isometry3d>(t0);
    auto t_p = std::make_shared<Eigen::Isometry3d>(t1);

    // triangulated points are located in the cos of poses (where they have id)
    {
        keyframe_bundle_adjustment::Triangulator<double> triang;
        Eigen::Matrix<double, 3, 1> p = triang.triangulate_rays({std::make_pair(id_p, v0), std::make_pair(t_p, v1)});

        std::cout << "triangulated point=" << p.transpose() << std::endl;
        std::cout << "gt point=" << p_gt.transpose() << std::endl;

        ASSERT_NEAR((p - p_gt).norm(), 0., 1e-5);
        std::cout << "----------------------------" << std::endl;
    }
}
TEST(CostFunctor, get_error_point_ray) {
    std::vector<double> point{1., 1., 10.};
    // cost functor
    double focal_length = 600.;
    double cu = 200.;
    double cv = 100.;

    Eigen::Matrix3d intrin;
    intrin << focal_length, 0., cu, 0., focal_length, cv, 0., 0., 1.;

    double observed_x = 260.;
    double observed_y = 160.;

    auto functor = keyframe_bundle_adjustment::cost_functors_ceres::ReprojectionErrorWithQuaternions(
        observed_x, observed_y, focal_length, cu, cv, {{1., 0., 0., 0., 0., 0.}});

    std::vector<double> pose{1., 0., 0., 0., 0., 0., 0.};

    std::vector<double> res(2);
    bool success = functor(pose.data(), point.data(), res.data());
    ASSERT_EQ(success, true);

    ASSERT_NEAR(res[0], 0., 1e-5);
    ASSERT_NEAR(res[1], 0., 1e-5);
    // do the same with pose unequal zero
    // functor uses ceres, test uses eigen to transform
    // get pose
    std::vector<double> euler_angles{0.1, 0.05, 0.2};
    keyframe_bundle_adjustment::local_parameterizations::EulerAnglesToQuaternion(euler_angles.data(), pose.data());
    std::vector<double> transl{0.01, -0.01, 0.01};
    std::copy(transl.cbegin(), transl.cend(), pose.begin() + 4);

    std::cout << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3] << " " << pose[4] << " " << pose[5]
              << " " << pose[6] << std::endl;
    Eigen::Isometry3d isom = Eigen::Isometry3d::Identity();
    isom.translate(Eigen::Vector3d(pose[4], pose[5], pose[6]));
    isom.rotate(Eigen::Quaterniond(pose[0], pose[1], pose[2], pose[3]));
    std::cout << isom.rotation() << std::endl;

    Eigen::Vector3d p_eigen(point.data());
    std::cout << p_eigen.transpose() << std::endl;
    Eigen::Vector3d transformed_p = isom * p_eigen;
    std::cout << transformed_p.transpose() << std::endl;

    transformed_p /= transformed_p[2];
    std::cout << transformed_p.transpose() << std::endl;

    Eigen::Vector3d proj = intrin * transformed_p;
    std::cout << proj.transpose() << std::endl;
    auto functor2 = keyframe_bundle_adjustment::cost_functors_ceres::ReprojectionErrorWithQuaternions(
        proj[0], proj[1], focal_length, cu, cv, {{1., 0., 0., 0., 0., 0., 0.}});

    success = functor2(pose.data(), point.data(), res.data());

    // rounding errors are pretty big!
    ASSERT_NEAR(res[0], 0., 1e-2);
    ASSERT_NEAR(res[1], 0., 1e-2);
}

namespace {

using namespace keyframe_bundle_adjustment;
double add_noise(double v, double noise) {
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0., noise);

    v += distribution(generator);

    return v;
}


Eigen::Vector3d add_noise(Eigen::Vector3d v, std::tuple<double, double, double> noise) {
    std::default_random_engine generator;
    double noise_x, noise_y, noise_z;
    std::tie(noise_x, noise_y, noise_z) = noise;
    std::normal_distribution<double> distribution_x(0., noise_x);
    std::normal_distribution<double> distribution_y(0., noise_y);
    std::normal_distribution<double> distribution_z(0., noise_z);

    v[0] += distribution_x(generator);
    v[1] += distribution_y(generator);
    v[2] += distribution_z(generator);

    return v;
}

Eigen::Vector2d add_noise(Eigen::Vector2d v, std::tuple<double, double> noise) {
    std::default_random_engine generator;
    double noise_x, noise_y;
    std::tie(noise_x, noise_y) = noise;
    std::normal_distribution<double> distribution_x(0., noise_x);
    std::normal_distribution<double> distribution_y(0., noise_y);

    v[0] += distribution_x(generator);
    v[1] += distribution_y(generator);

    return v;
}

Eigen::Vector2d project(Eigen::Vector3d p, Eigen::Matrix3d intrinsics) {
    Eigen::Vector3d proj = intrinsics * p;
    proj /= proj[2];

    return Eigen::Vector2d(proj[0], proj[1]);
}

Eigen::Vector3d projectKeepDepth(Eigen::Vector3d p, Eigen::Matrix3d intrinsics) {
    Eigen::Vector3d proj = intrinsics * p;
    proj /= proj[2];

    return Eigen::Vector3d(proj[0], proj[1], p.z());
}

std::map<TimestampNSec, Eigen::Isometry3d> getPoses(double noise_angle,
                                                    std::tuple<double, double, double> noise_transl,
                                                    std::vector<TimestampNSec> stamps) {
    std::map<TimestampNSec, Eigen::Isometry3d> noisy_poses;
    noisy_poses[stamps[0]] = Eigen::Isometry3d::Identity();
    noisy_poses[stamps[1]] = noisy_poses[stamps[0]];
    noisy_poses[stamps[1]].translate(Eigen::Vector3d(-1.5, 0., -2.));
    noisy_poses[stamps[1]].rotate(Eigen::AngleAxisd(-0.05, Eigen::Vector3d(0., 0., 1.)));
    noisy_poses[stamps[2]] = noisy_poses[stamps[1]];
    noisy_poses[stamps[2]].translate(add_noise(Eigen::Vector3d(-2.0, 0., 0.), noise_transl));
    noisy_poses[stamps[2]].rotate(Eigen::AngleAxisd(add_noise(-0.05, noise_angle), Eigen::Vector3d(0., 0., 1.)));
    noisy_poses[stamps[3]] = noisy_poses[stamps[2]];
    noisy_poses[stamps[3]].translate(add_noise(Eigen::Vector3d(-1.5, -0.1, 0.), noise_transl));
    noisy_poses[stamps[4]] = noisy_poses[stamps[3]];
    noisy_poses[stamps[4]].translate(add_noise(Eigen::Vector3d(-2.9, -0., 0.), noise_transl));

    return noisy_poses;
}
std::map<TimestampNSec, Eigen::Isometry3d> getPosesOnCircle(double noise_angle,
                                                            std::tuple<double, double, double> noise_transl,
                                                            std::vector<TimestampNSec> stamps) {
    std::map<TimestampNSec, Eigen::Isometry3d> noisy_poses;
    noisy_poses[stamps[0]] = Eigen::Isometry3d::Identity();
    noisy_poses[stamps[1]] = noisy_poses[stamps[0]];
    //        noisy_poses[stamps[1]].translate(add_noise(Eigen::Vector3d(1.5, 0., 0.),
    //        noise_transl));
    noisy_poses[stamps[1]].translate(Eigen::Vector3d(0., 0., 1.5));
    //    noisy_poses[stamps[1]].rotate(Eigen::AngleAxisd(0.05, Eigen::Vector3d(0., 1., 0.)));
    noisy_poses[stamps[2]] = noisy_poses[stamps[1]];
    //    noisy_poses[stamps[2]].rotate(
    //        Eigen::AngleAxisd(add_noise(0.05, noise_angle), Eigen::Vector3d(0., 1., 0.)));
    noisy_poses[stamps[2]].translate(add_noise(Eigen::Vector3d(0., 0., 2.0), noise_transl));
    noisy_poses[stamps[3]] = noisy_poses[stamps[2]];
    noisy_poses[stamps[3]].translate(add_noise(Eigen::Vector3d(0., 0.0, 1.5), noise_transl));
    noisy_poses[stamps[4]] = noisy_poses[stamps[3]];
    noisy_poses[stamps[4]].translate(add_noise(Eigen::Vector3d(0.0, 0., 0.5), noise_transl));

    return noisy_poses;
}

std::array<double, 7> convert(const Eigen::Isometry3d& p) {
    Eigen::Quaterniond q(p.rotation());

    std::array<double, 7> pose;
    pose[0] = q.w();
    pose[1] = q.x();
    pose[2] = q.y();
    pose[3] = q.z();

    pose[4] = p.translation()[0];
    pose[5] = p.translation()[1];
    pose[6] = p.translation()[2];

    return pose;
}

keyframe_bundle_adjustment::Tracklets makeTracklets(const std::map<KeyframeId, Eigen::Isometry3d>& poses_gt,
                                                    const std::vector<Eigen::Vector3d>& lms_origin,
                                                    std::map<CameraId, Camera::Ptr> cameras,
                                                    std::tuple<double, double> noise_lms = std::make_tuple(0., 0.),
                                                    const std::map<LandmarkId, CameraIds>& landmark_to_cameras =
                                                        std::map<LandmarkId, keyframe_bundle_adjustment::CameraIds>(),
                                                    std::vector<LandmarkId> lm_ids = std::vector<LandmarkId>(),
                                                    std::vector<TimestampNSec> stamps = std::vector<TimestampNSec>()) {
    Tracklets ts;
    ts.tracks = std::vector<Tracklet>(lms_origin.size());

    if (lm_ids.size() == 0) {
        for (int i = 0; i < int(lms_origin.size()); ++i) {
            lm_ids.push_back(i);
        }
    }

    ts.stamps = stamps;

    if (stamps.size() == 0) {
        for (int i = 0; i < int(poses_gt.size()); ++i) {
            stamps.push_back(i);
        }
    }
    ts.stamps = stamps;

    for (int i = 0; i < int(lm_ids.size()); ++i) {
        ts.tracks[i].id = lm_ids[i];
    }

    for (const auto& p : poses_gt) {
        Eigen::Isometry3d pose_keyframe_origin = p.second;

        //            Eigen::Isometry3d transf = cur_cam_ext_inv * p.second.inverse() *
        //            cur_cam_ext;

        assert(ts.tracks.size() == lms_origin.size());
        assert(ts.tracks.size() == lm_ids.size());
        for (int i = 0; i < int(ts.tracks.size()); ++i) {
            CameraIds cam_ids;
            if (landmark_to_cameras.size() == 0) {
                cam_ids = CameraIds{0};
            } else {
                cam_ids = landmark_to_cameras.at(lm_ids.at(i));
            }
            assert(cam_ids.size() == 1);

            EigenPose pose_camera_keyframe = cameras.at(cam_ids[0])->getEigenPose();
            Eigen::Isometry3d pose_cam_origin = pose_camera_keyframe * pose_keyframe_origin;
            Eigen::Vector3d lm_cam = pose_cam_origin * lms_origin[i];

            //            std::cout << "calib="
            //                      <<
            //                      Eigen::Quaterniond(pose_camera_keyframe.rotation()).coeffs().transpose()
            //                      << " " << pose_camera_keyframe.translation().transpose() <<
            //                      std::endl;
            //            std::cout << "keyframe_pose="
            //                      <<
            //                      Eigen::Quaterniond(pose_keyframe_origin.rotation()).coeffs().transpose()
            //                      << " " << pose_keyframe_origin.translation().transpose() <<
            //                      std::endl;

            ts.tracks[i].feature_points.push_back(
                add_noise(project(lm_cam, cameras.at(cam_ids[0])->getIntrinsicMatrix()), noise_lms));
        }
    }

    return ts;
}


keyframe_bundle_adjustment::Tracklets makeTrackletsDepth(
    const std::map<KeyframeId, Eigen::Isometry3d>& poses_gt,
    const std::vector<Eigen::Vector3d>& lms_origin,
    std::map<CameraId, Camera::Ptr> cameras,
    std::tuple<double, double, double> noise_lms = std::make_tuple(0., 0., 0.),
    const std::map<LandmarkId, CameraIds>& landmark_to_cameras =
        std::map<LandmarkId, keyframe_bundle_adjustment::CameraIds>(),
    std::vector<LandmarkId> lm_ids = std::vector<LandmarkId>(),
    std::vector<TimestampNSec> stamps = std::vector<TimestampNSec>()) {
    Tracklets ts;
    ts.tracks = std::vector<Tracklet>(lms_origin.size());

    if (lm_ids.size() == 0) {
        for (int i = 0; i < int(lms_origin.size()); ++i) {
            lm_ids.push_back(i);
        }
    }

    ts.stamps = stamps;

    if (stamps.size() == 0) {
        for (int i = 0; i < int(poses_gt.size()); ++i) {
            stamps.push_back(i);
        }
    }
    ts.stamps = stamps;

    for (int i = 0; i < int(lm_ids.size()); ++i) {
        ts.tracks[i].id = lm_ids[i];
    }

    for (const auto& p : poses_gt) {
        Eigen::Isometry3d pose_keyframe_origin = p.second;

        //            Eigen::Isometry3d transf = cur_cam_ext_inv * p.second.inverse() *
        //            cur_cam_ext;

        assert(ts.tracks.size() == lms_origin.size());
        assert(ts.tracks.size() == lm_ids.size());
        for (int i = 0; i < int(ts.tracks.size()); ++i) {
            CameraIds cam_ids;
            if (landmark_to_cameras.size() == 0) {
                cam_ids = CameraIds{0};
            } else {
                cam_ids = landmark_to_cameras.at(lm_ids.at(i));
            }
            assert(cam_ids.size() == 1);

            EigenPose pose_camera_keyframe = cameras.at(cam_ids[0])->getEigenPose();
            Eigen::Isometry3d pose_cam_origin = pose_camera_keyframe * pose_keyframe_origin;
            Eigen::Vector3d lm_cam = pose_cam_origin * lms_origin[i];

            ts.tracks[i].feature_points.push_back(
                add_noise(projectKeepDepth(lm_cam, cameras.at(cam_ids[0])->getIntrinsicMatrix()), noise_lms));
        }
    }

    return ts;
}

void evaluate_bundle_adjustment(std::tuple<double, double> noise_lms,
                                std::tuple<double, double, double, double> noise_poses,
                                double acceptance_thres,
                                std::vector<Eigen::Isometry3d> poses_cameras_keyframe) {
    using namespace keyframe_bundle_adjustment;
    //    // define extrinsics
    //    std::map<keyframe_bundle_adjustment::KeyframeId, Eigen::Isometry3d> cam_ext;
    //    cam_ext[0] = Eigen::Isometry3d::Identity();
    //    cam_ext[0].translate(Eigen::Vector3d(1.3, -0.1, 1.65));
    //    cam_ext[0].rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(-1, 0., 0.)) *
    //                      Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(0., 1., 0.)));

    double focal_length = 600.;
    Eigen::Vector2d principal_point{200., 100.};

    Eigen::Matrix3d intrinsics;
    intrinsics << focal_length, 0., principal_point[0], 0., focal_length, principal_point[1], 0., 0., 1.;

    double noise_angle = std::get<0>(noise_poses);
    std::tuple<double, double, double> noise_transl =
        std::make_tuple(std::get<1>(noise_poses), std::get<2>(noise_poses), std::get<3>(noise_poses));

    std::vector<TimestampNSec> stamps{0, 1, 2, 3, 4};
    // define groudntruth poses
    std::map<KeyframeId, Eigen::Isometry3d> poses_gt = getPoses(0., std::make_tuple(0., 0., 0.), stamps);

    std::map<KeyframeId, Eigen::Isometry3d> noisy_poses = getPoses(noise_angle, noise_transl, stamps);
    for (int i = 2; i < int(noisy_poses.size()); ++i) {
        auto& pos = noisy_poses[i];
        pos.translation().normalize();
    }

    // define measurements
    std::vector<Eigen::Vector3d> lms_origin{Eigen::Vector3d(10., 3., 5.5),
                                            Eigen::Vector3d(11., 1., 6.5),
                                            Eigen::Vector3d(14., -5., 6.),
                                            Eigen::Vector3d(9., 1., 5.),
                                            Eigen::Vector3d(16., -1., 4.)};

    std::map<CameraId, Camera::Ptr> extrinsics_camera_kf;
    for (int i = 0; i < int(poses_cameras_keyframe.size()); ++i) {
        extrinsics_camera_kf[i] = std::make_shared<Camera>(focal_length, principal_point, poses_cameras_keyframe[i]);
    }

    std::vector<LandmarkId> lm_ids{0, 1, 2, 3, 4};

    std::map<LandmarkId, CameraIds> landmark_to_cameras;
    for (int i = 0; i < int(lm_ids.size()); ++i) {
        CameraId cam_id = i % poses_cameras_keyframe.size();
        landmark_to_cameras[lm_ids[i]] = CameraIds{cam_id};
    }

    auto ts = makeTracklets(poses_gt, lms_origin, extrinsics_camera_kf, noise_lms, landmark_to_cameras, lm_ids, stamps);

    // output
    std::cout << "gt_poses:" << std::endl;
    for (const auto& pos : poses_gt) {
        std::cout << "pose id=" << pos.first << "\n" << pos.second.matrix() << std::endl;
    }

    std::cout << "noisy_poses:" << std::endl;
    for (const auto& pos : noisy_poses) {
        std::cout << "pose id=" << pos.first << "\n" << pos.second.matrix() << std::endl;
    }

    // add landmark selection scheme: all
    BundleAdjusterKeyframes b;
    b.set_solver_time(20.);
    //    BundleAdjusterKeyframes b(
    //        LandmarkSelectionSchemeRandom::createConst(std::numeric_limits<size_t>::max()));

    // add measurements to keyframes
    if (poses_cameras_keyframe.size() == 1) {
        // test mono interface
        Camera::Ptr camera = std::make_shared<Camera>(focal_length, principal_point, poses_cameras_keyframe[0]);

        ASSERT_EQ(camera->getEigenPose().matrix().isApprox(poses_cameras_keyframe[0].matrix()), true);

        b.push(Keyframe(stamps.at(0), ts, camera, noisy_poses.at(stamps.at(0)), Keyframe::FixationStatus::Pose));
        b.push(Keyframe(stamps.at(1), ts, camera, noisy_poses.at(stamps.at(1)), Keyframe::FixationStatus::Scale));

        for (int i = 2; i < int(stamps.size()); ++i) {
            b.push(Keyframe(stamps.at(i), ts, camera, noisy_poses.at(stamps.at(i))));
        }
    } else {
        // test multi_cam interface
        b.push(Keyframe(stamps.at(0),
                        ts,
                        extrinsics_camera_kf,
                        landmark_to_cameras,
                        noisy_poses.at(stamps.at(0)),
                        Keyframe::FixationStatus::Pose));
        b.push(Keyframe(stamps.at(1),
                        ts,
                        extrinsics_camera_kf,
                        landmark_to_cameras,
                        noisy_poses.at(stamps.at(1)),
                        Keyframe::FixationStatus::Scale));
        for (int i = 2; i < int(stamps.size()); ++i) {
            b.push(Keyframe(stamps.at(i), ts, extrinsics_camera_kf, landmark_to_cameras, noisy_poses.at(stamps.at(i))));
        }
    }

    for (int i = 0; i < int(lms_origin.size()); ++i) {
        ASSERT_EQ(lms_origin.size(), b.landmarks_.size());
        Eigen::Vector3d rec_lm(b.landmarks_.at(lm_ids.at(i))->pos.data());
        ASSERT_NEAR((lms_origin[i] - rec_lm).norm(), 0., 1e-1);
    }


    // remember non optimized poses for later
    std::vector<std::array<double, 7>> poses_before_bundling;
    for (const auto& kf : b.keyframes_) {
        poses_before_bundling.push_back(kf.second->pose_);
    }
    // ////////////////////////////////////////////////////
    // test landmark selector
    //    std::vector<double> angles;
    for (int i = 0; i < int(lms_origin.size()); ++i) {
        const auto& lm = lms_origin[i];
        CameraIds cam_ids = landmark_to_cameras[lm_ids[i]];
        assert(cam_ids.size() == 1);
        EigenPose pose_camera_keyframe = extrinsics_camera_kf.at(cam_ids[0])->getEigenPose();
        Eigen::Isometry3d pose_cam_origin_0 = pose_camera_keyframe * poses_gt[0];
        Eigen::Isometry3d pose_cam_origin_4 = pose_camera_keyframe * poses_gt[4];

        Eigen::Vector3d first = pose_cam_origin_0 * lm;
        first.normalize();

        Eigen::Vector3d last = pose_cam_origin_4 * lm;
        last.normalize();

        std::cout << "angle=" << std::acos(first.dot(last)) << std::endl;
    }
    //    keyframe_bundle_adjustment::LandmarkSelectionSchemeObservability::Parameters p;
    //    keyframe_bundle_adjustment::LandmarkSelectionSchemeObservability obs_scheme(p);
    //    auto selected_landmark_ids = obs_scheme.getSelection(b.landmarks_, b.keyframes_);


    // ////////////////////////////////////////////////////
    // test bundle adjuster
    std::string summary = b.solve();

    std::cout << summary << std::endl;

    //    std::cout << "before bundling:" << std::endl;
    //    for (const auto& p : poses_before_bundling) {
    //        std::cout << Eigen::Matrix<double, 1, 7>(p.data()) << std::endl;
    //    }

    //    // output
    //    std::cout << "after bundling:" << std::endl;
    //    for (const auto& kf : b.keyframes_) {
    //        std::cout << Eigen::Matrix<double, 1, 7>(kf.second.pose.data()) << std::endl;
    //    }
    //    std::cout << "gt:" << std::endl;
    //    for (const auto& pos : poses_gt) {
    //        std::cout << Eigen::Matrix<double, 1, 7>(convert(pos.second).data()) << std::endl;
    //    }

    // print stuff
    {
        std::cout << "before bundling | after bundling | gt" << std::endl;
        auto poses_before_bundling_iter = poses_before_bundling.cbegin();
        auto b_keyframes_iter = b.keyframes_.cbegin();
        auto poses_gt_iter = poses_gt.cbegin();
        for (; poses_before_bundling_iter != poses_before_bundling.cend() && b_keyframes_iter != b.keyframes_.cend() &&
               poses_gt_iter != poses_gt.cend();
             ++poses_before_bundling_iter, ++b_keyframes_iter, ++poses_gt_iter) {

            auto converted_gt = convert(poses_gt_iter->second);
            for (int i = 0; i < int(7); ++i) {
                std::cout << (*poses_before_bundling_iter)[i] << "|" << b_keyframes_iter->second->pose_[i] << "|"
                          << converted_gt[i] << "\n";
            }

            std::cout << std::endl;
        }
    }

    // compare before and after optimization
    auto poses_gt_iter = poses_gt.cbegin();
    auto poses_iter = b.keyframes_.cbegin();
    //    if (poses_gt.size() != b.keyframes_.size()) {
    //        throw std::runtime_error("poses_gt.size()=" + std::to_string(poses_gt.size()) +
    //                                 " != poses.size()=" + std::to_string(b.keyframes_.size()));
    //    }
    for (; poses_gt_iter != poses_gt.cend() && poses_iter != b.keyframes_.cend(); ++poses_gt_iter, ++poses_iter) {
        ASSERT_EQ(poses_iter->second->getEigenPose().isApprox(poses_gt_iter->second, acceptance_thres), true);
    }
}
}


TEST(KeyframeSelector, process) {
    using namespace keyframe_bundle_adjustment;
    // This object selects keyframes for you
    KeyframeSelector kf_selector;

    // choose selection scheme
    double time_difference_sec = 0.5; // time lap between frames

    // add it to selector
    KeyframeSparsificationSchemeBase::ConstPtr scheme0 =
        std::make_shared<KeyframeSparsificationSchemeTime>(time_difference_sec);
    kf_selector.addScheme(scheme0);

    // dummy frames, here you have to put the new data
    std::map<KeyframeId, Keyframe::Ptr> last_frames;
    last_frames[0] = Keyframe::Ptr(new Keyframe(0, {}, Camera::Ptr(), Eigen::Isometry3d::Identity()));
    last_frames[1] = Keyframe::Ptr(new Keyframe(10000, {}, Camera::Ptr(), Eigen::Isometry3d::Identity()));

    TimestampNSec ts1{10000 + convert(TimestampSec(2. * time_difference_sec))};
    Keyframe::Ptr new_frame0(new Keyframe(ts1, {}, Camera::Ptr(), Eigen::Isometry3d::Identity()));

    TimestampNSec ts2{10000 + convert(TimestampSec(time_difference_sec / 2.))};
    Keyframe::Ptr new_frame1(new Keyframe(ts2, {}, Camera::Ptr(), Eigen::Isometry3d::Identity()));

    // test scheme
    ASSERT_TRUE(scheme0->isUsable(new_frame0, last_frames));
    ASSERT_FALSE(scheme0->isUsable(new_frame1, last_frames));

    // test selector
    // do selection of frames, by comparing to internal frame buffer
    std::set<Keyframe::Ptr> selected_keyframes = kf_selector.select({new_frame0, new_frame1}, last_frames);

    ASSERT_EQ(selected_keyframes.size(), 1);
    ASSERT_EQ((*selected_keyframes.begin())->timestamp_, ts1);
}

TEST(LandmarkSelector, base) {
    using namespace keyframe_bundle_adjustment;


    // calc data on keyframes
    std::vector<Eigen::Vector3d> lms_origin{Eigen::Vector3d(0.5, 3., 5.5),
                                            Eigen::Vector3d(0., 1., -20.),
                                            Eigen::Vector3d(1., -5., 4.),
                                            Eigen::Vector3d(2.0, 1., 1.5),
                                            Eigen::Vector3d(-2.0, -1., 10.)};
    std::map<LandmarkId, Landmark::ConstPtr> lms_arr_origin;
    int count = 0;
    for (const auto& el : lms_origin) {
        lms_arr_origin[count] = std::make_shared<const Landmark>(el);
        count++;
    }

    std::map<keyframe_bundle_adjustment::TimestampNSec, Eigen::Isometry3d> poses =
        getPoses(0., std::make_tuple(0., 0., 0.), {0, 1, 2, 3, 4});


    Camera cam(600, {300, 200}, Eigen::Isometry3d::Identity());
    Camera::Ptr cam_ptr = std::make_shared<Camera>(cam);
    auto ts = makeTrackletsDepth(poses, lms_origin, std::map<CameraId, Camera::Ptr>{{0, cam_ptr}});

    std::map<KeyframeId, Keyframe::ConstPtr> kf_const_ptrs;
    int count2 = 0;
    for (const auto& el : poses) {
        kf_const_ptrs[count2] = std::make_shared<const Keyframe>(Keyframe(count2, ts, cam_ptr, el.second));
        count2++;
    }

    // test random scheme
    std::cout << "test random scheme" << std::endl;
    {
        LandmarkSelector selector;

        selector.addScheme(LandmarkSelectionSchemeRandom::create(6));

        auto selected_lms = selector.select(lms_arr_origin, kf_const_ptrs);
        ASSERT_EQ(selected_lms.size(), lms_arr_origin.size());
    }
    {
        LandmarkSelector selector;

        selector.addScheme(LandmarkSelectionSchemeRandom::create(3));

        auto selected_lms = selector.select(lms_arr_origin, kf_const_ptrs);
        ASSERT_EQ(selected_lms.size(), 3);
    }

    // test cheirality
    std::cout << "test cheirality scheme" << std::endl;
    {
        LandmarkSelector selector;

        selector.addScheme(LandmarkSelectionSchemeCheirality::create());

        auto selected_lms = selector.select(lms_arr_origin, kf_const_ptrs);
        ASSERT_EQ(selected_lms.size(), 3);
        // landmark with id 1 is behin image plane

        selector.addScheme(LandmarkSelectionSchemeRandom::create(2));
        selected_lms = selector.select(lms_arr_origin, kf_const_ptrs);
        ASSERT_EQ(selected_lms.size(), 2);
    }

    // test observability
    {
        LandmarkSelector selector;

        // delete one measruement on one of the keyframes for far field test
        Keyframe cur_kf = *kf_const_ptrs.at(0);
        cur_kf.measurements_.erase(cur_kf.measurements_.find(4));
        kf_const_ptrs[0] = std::make_shared<const Keyframe>(cur_kf); // why is this necessary

        LandmarkSelectionSchemeObservability::Parameters p;
        p.bin_params_.max_num_landmarks_far = 1;
        p.bin_params_.max_num_landmarks_near = 1;
        p.bin_params_.max_num_landmarks_middle = 1;
        selector.addScheme(LandmarkSelectionSchemeObservability::create(p));

        auto selected_lms = selector.select(lms_arr_origin, kf_const_ptrs);
        ASSERT_LE(selected_lms.size(), 3);

        // test if categorizer interface in observability works
        ASSERT_GT(selector.getLandmarkCategories().size(), 0);

        for (const auto& el : selected_lms) {
            ASSERT_NE(el, 4);
            ASSERT_NE(el, 0);
        }
    }
}

TEST(BundleAdjusterKeyframes, deactivateKeyframes) {
    using namespace keyframe_bundle_adjustment;


    // calc data on keyframes
    std::vector<Eigen::Vector3d> lms_origin{Eigen::Vector3d(0.5, 3., 5.5),
                                            Eigen::Vector3d(0., 1., -4.),
                                            Eigen::Vector3d(0., 3., -4.),
                                            Eigen::Vector3d(1., -5., 4.),
                                            Eigen::Vector3d(1., -5., 5.),
                                            Eigen::Vector3d(2.0, 1., 1.5),
                                            Eigen::Vector3d(-2.0, -1., 10.)};
    std::map<LandmarkId, Landmark> lms_arr_origin;
    int count = 0;
    for (const auto& el : lms_origin) {
        lms_arr_origin[count] = Landmark(el);
        count++;
    }

    std::vector<TimestampNSec> stamps{convert(TimestampSec(0.1)),
                                      convert(TimestampSec(0.2)),
                                      convert(TimestampSec(0.3)),
                                      convert(TimestampSec(0.4)),
                                      convert(TimestampSec(0.5))};
    std::map<keyframe_bundle_adjustment::TimestampNSec, Eigen::Isometry3d> poses =
        getPoses(0., std::make_tuple(0., 0., 0.), stamps);


    Camera cam(600, {300, 200}, Eigen::Isometry3d::Identity());
    Camera::Ptr cam_ptr = std::make_shared<Camera>(cam);
    auto ts = makeTracklets(poses,
                            lms_origin,
                            std::map<CameraId, Camera::Ptr>{{0, cam_ptr}},
                            std::make_tuple(0., 0.),
                            std::map<LandmarkId, keyframe_bundle_adjustment::CameraIds>(),
                            std::vector<LandmarkId>(),
                            stamps);

    // add data to adjuster and add keyframe that should be deactivated
    BundleAdjusterKeyframes adjuster;
    adjuster.set_solver_time(20.);
    Eigen::Isometry3d pose_to_deactivate = Eigen::Isometry3d::Identity();
    pose_to_deactivate.translate(Eigen::Vector3d(10., 10., 10.));
    adjuster.push(Keyframe(0, ts, cam_ptr, pose_to_deactivate, Keyframe::FixationStatus::Pose));
    for (const auto& el : poses) {
        adjuster.push(Keyframe(el.first, ts, cam_ptr, el.second, Keyframe::FixationStatus::None));
    }
    adjuster.keyframes_.at(convert(TimestampSec(0.1)))->fixation_status_ = Keyframe::FixationStatus::Scale;

    // do deactivation
    adjuster.deactivateKeyframes(3, 0.0, 1000.0);

    // test result
    std::cout << "testing" << std::endl;
    ASSERT_EQ(adjuster.active_keyframe_ids_.size(), 5);
    ASSERT_EQ(adjuster.active_landmark_ids_.size(), lms_origin.size());
    ASSERT_EQ(adjuster.keyframes_.at(convert(TimestampSec(0.1)))->fixation_status_, Keyframe::FixationStatus::Pose);
    ASSERT_EQ(adjuster.keyframes_.at(convert(TimestampSec(0.2)))->fixation_status_, Keyframe::FixationStatus::Scale);
}

TEST(KeyFrameBundleAdjustment, solve) {
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();

    p.rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(1., 0., 0.)));
    p.rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(0., 0., 1.)));
    p.translate(Eigen::Vector3d(-1.5, 0.2, -1.35));

    p = p.inverse();

    std::cout << "======================== Mono =======================" << std::endl;
    std::cout << "--------------------------- zero noise ----------------------------" << std::endl;
    evaluate_bundle_adjustment(std::make_tuple(0., 0.), std::make_tuple(0.0, 0., 0.0, 0.), 0.001, {p});

    std::cout << "--------------------------- noise "
                 "=0.,0.,5pi/180,0.2,0.1,0.1----------------------------"
              << std::endl;
    evaluate_bundle_adjustment(std::make_tuple(0., 0.), std::make_tuple(5. * M_PI / 180., 0.2, 0.1, 0.1), 0.001, {p});

    std::cout << "--------------------------- noise "
                 "=0.1,0.1,5pi/180,0.2,0.1,0.1----------------------------"
              << std::endl;
    evaluate_bundle_adjustment(std::make_tuple(1.5, 1.5), std::make_tuple(5. * M_PI / 180., 0.2, 0.1, 0.1), 0.01, {p});

    std::cout << "======================== Multi cam =======================" << std::endl;

    std::cout << p.matrix() << std::endl;

    std::vector<Eigen::Isometry3d> trf_camI_vehicle;
    trf_camI_vehicle.push_back(p);

    p.translate(Eigen::Vector3d(0., -0.5, 0.));
    p.rotate(Eigen::AngleAxisd(M_PI / 18., Eigen::Vector3d(0., 1., 0.)));
    p.rotate(Eigen::AngleAxisd(M_PI / 18., Eigen::Vector3d(1., 0., 0.)));
    trf_camI_vehicle.push_back(p);

    std::cout << p.matrix() << std::endl;

    std::cout << "--------------------------- zero noise ----------------------------" << std::endl;
    evaluate_bundle_adjustment(std::make_tuple(0., 0.), std::make_tuple(0.0, 0., 0.0, 0.), 0.001, trf_camI_vehicle);

    std::cout << "--------------------------- noise "
                 "=0.,0.,5pi/180,0.2,0.1,0.1----------------------------"
              << std::endl;
    evaluate_bundle_adjustment(
        std::make_tuple(0., 0.), std::make_tuple(5. * M_PI / 180., 0.2, 0.1, 0.1), 0.001, trf_camI_vehicle);

    std::cout << "--------------------------- noise "
                 "=0.1,0.1,5pi/180,0.2,0.1,0.1----------------------------"
              << std::endl;
    evaluate_bundle_adjustment(
        std::make_tuple(1.5, 1.5), std::make_tuple(5. * M_PI / 180., 0.2, 0.1, 0.1), 0.01, trf_camI_vehicle);
}
namespace {
void evaluate_bundle_adjustment_depth(std::tuple<double, double, double> noise_lms,
                                      std::tuple<double, double, double, double> noise_poses,
                                      double acceptance_thres,
                                      std::vector<Eigen::Isometry3d> poses_cameras_keyframe,
                                      bool test_motion_only = false) {
    using namespace keyframe_bundle_adjustment;
    //    // define extrinsics
    //    std::map<keyframe_bundle_adjustment::KeyframeId, Eigen::Isometry3d> cam_ext;
    //    cam_ext[0] = Eigen::Isometry3d::Identity();
    //    cam_ext[0].translate(Eigen::Vector3d(1.3, -0.1, 1.65));
    //    cam_ext[0].rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(-1, 0., 0.)) *
    //                      Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(0., 1., 0.)));

    double focal_length = 600.;
    Eigen::Vector2d principal_point{200., 100.};

    Eigen::Matrix3d intrinsics;
    intrinsics << focal_length, 0., principal_point[0], 0., focal_length, principal_point[1], 0., 0., 1.;

    double noise_angle = std::get<0>(noise_poses);
    std::tuple<double, double, double> noise_transl =
        std::make_tuple(std::get<1>(noise_poses), std::get<2>(noise_poses), std::get<3>(noise_poses));

    std::vector<TimestampNSec> stamps{0, 1, 2, 3, 4};
    // define groudntruth poses
    std::map<KeyframeId, Eigen::Isometry3d> poses_gt = getPoses(0., std::make_tuple(0., 0., 0.), stamps);

    std::map<KeyframeId, Eigen::Isometry3d> noisy_poses = getPoses(noise_angle, noise_transl, stamps);
    for (int i = 2; i < int(noisy_poses.size()); ++i) {
        auto& pos = noisy_poses[i];
        pos.translation().normalize();
    }

    // define measurements
    std::vector<Eigen::Vector3d> lms_origin{Eigen::Vector3d(10., 3., 5.5),
                                            Eigen::Vector3d(11., 1., 6.5),
                                            Eigen::Vector3d(14., -5., 6.),
                                            Eigen::Vector3d(9., 1., 5.),
                                            Eigen::Vector3d(16., -1., 4.)};

    std::map<CameraId, Camera::Ptr> extrinsics_camera_kf;
    for (int i = 0; i < int(poses_cameras_keyframe.size()); ++i) {
        extrinsics_camera_kf[i] = std::make_shared<Camera>(focal_length, principal_point, poses_cameras_keyframe[i]);
    }

    std::vector<LandmarkId> lm_ids{0, 1, 2, 3, 4};

    std::map<LandmarkId, CameraIds> landmark_to_cameras;
    for (int i = 0; i < int(lm_ids.size()); ++i) {
        CameraId cam_id = i % poses_cameras_keyframe.size();
        landmark_to_cameras[lm_ids[i]] = CameraIds{cam_id};
    }

    auto ts =
        makeTrackletsDepth(poses_gt, lms_origin, extrinsics_camera_kf, noise_lms, landmark_to_cameras, lm_ids, stamps);

    // output
    std::cout << "gt_poses:" << std::endl;
    for (const auto& pos : poses_gt) {
        std::cout << "pose id=" << pos.first << "\n" << pos.second.matrix() << std::endl;
    }

    std::cout << "noisy_poses:" << std::endl;
    for (const auto& pos : noisy_poses) {
        std::cout << "pose id=" << pos.first << "\n" << pos.second.matrix() << std::endl;
    }

    // add landmark selection scheme: all
    BundleAdjusterKeyframes b;
    b.set_solver_time(20.);

    //    BundleAdjusterKeyframes b(
    //        LandmarkSelectionSchemeRandom::createConst(std::numeric_limits<size_t>::max()));


    // add measurements to keyframes
    int max_ind = int(stamps.size() - 1);

    // motion only uses only last noisy pose and adjusts it.
    if (test_motion_only) {
        for (int i = 0; i < max_ind; ++i) {
            noisy_poses.at(stamps.at(i)) = poses_gt.at(stamps.at(i));
        }
    }
    if (poses_cameras_keyframe.size() == 1) {
        // test mono interface
        Camera::Ptr camera = std::make_shared<Camera>(focal_length, principal_point, poses_cameras_keyframe[0]);

        ASSERT_EQ(camera->getEigenPose().matrix().isApprox(poses_cameras_keyframe[0].matrix()), true);

        b.push(Keyframe(stamps.at(0), ts, camera, noisy_poses.at(stamps.at(0)), Keyframe::FixationStatus::Pose));
        b.push(Keyframe(stamps.at(1), ts, camera, noisy_poses.at(stamps.at(1)), Keyframe::FixationStatus::Scale));

        for (int i = 2; i < max_ind; ++i) {
            b.push(Keyframe(stamps.at(i), ts, camera, noisy_poses.at(stamps.at(i))));
        }
    } else {
        // test multi_cam interface
        b.push(Keyframe(stamps.at(0),
                        ts,
                        extrinsics_camera_kf,
                        landmark_to_cameras,
                        noisy_poses.at(stamps.at(0)),
                        Keyframe::FixationStatus::Pose));
        b.push(Keyframe(stamps.at(1),
                        ts,
                        extrinsics_camera_kf,
                        landmark_to_cameras,
                        noisy_poses.at(stamps.at(1)),
                        Keyframe::FixationStatus::Scale));
        for (int i = 2; i < max_ind; ++i) {
            b.push(Keyframe(stamps.at(i), ts, extrinsics_camera_kf, landmark_to_cameras, noisy_poses.at(stamps.at(i))));
        }
    }

    for (int i = 0; i < int(lms_origin.size()); ++i) {
        ASSERT_EQ(lms_origin.size(), b.landmarks_.size());
        Eigen::Vector3d rec_lm(b.landmarks_.at(lm_ids.at(i))->pos.data());
        ASSERT_NEAR((lms_origin[i] - rec_lm).norm(), 0., 1e-1);
    }


    //    // remember non optimized poses for later
    //    std::vector<std::array<double, 7>> poses_before_bundling;
    //    for (const auto& kf : b.keyframes_) {
    //        poses_before_bundling.push_back(kf.second->pose_);
    //    }
    //    if(test_motion_only){
    //        poses
    //    }
    // ////////////////////////////////////////////////////
    // test landmark selector
    //    std::vector<double> angles;
    for (int i = 0; i < int(lms_origin.size()); ++i) {
        const auto& lm = lms_origin[i];
        CameraIds cam_ids = landmark_to_cameras[lm_ids[i]];
        assert(cam_ids.size() == 1);
        EigenPose pose_camera_keyframe = extrinsics_camera_kf.at(cam_ids[0])->getEigenPose();
        Eigen::Isometry3d pose_cam_origin_0 = pose_camera_keyframe * poses_gt[0];
        Eigen::Isometry3d pose_cam_origin_4 = pose_camera_keyframe * poses_gt[4];

        Eigen::Vector3d first = pose_cam_origin_0 * lm;
        first.normalize();

        Eigen::Vector3d last = pose_cam_origin_4 * lm;
        last.normalize();

        std::cout << "angle=" << std::acos(first.dot(last)) << std::endl;
    }
    //    keyframe_bundle_adjustment::LandmarkSelectionSchemeObservability::Parameters p;
    //    keyframe_bundle_adjustment::LandmarkSelectionSchemeObservability obs_scheme(p);
    //    auto selected_landmark_ids = obs_scheme.getSelection(b.landmarks_, b.keyframes_);


    // ////////////////////////////////////////////////////
    // test bundle adjuster
    std::string summary;
    if (test_motion_only) {
        Keyframe kf;
        if (poses_cameras_keyframe.size() == 1) {
            Camera::Ptr camera = std::make_shared<Camera>(focal_length, principal_point, poses_cameras_keyframe[0]);
            kf = Keyframe(stamps.back(), ts, camera, noisy_poses.at(stamps.back()));
        } else {
            kf = Keyframe(stamps.back(), ts, extrinsics_camera_kf, landmark_to_cameras, noisy_poses.at(stamps.back()));
        }
        summary = b.adjustPoseOnly(kf);
        std::cout << "Optimization done" << std::endl;
        std::cout << "pose= " << Eigen::Map<Eigen::Matrix<double, 1, 7>>(kf.pose_.data()) << std::endl;
        ASSERT_EQ(kf.getEigenPose().isApprox(poses_gt.at(kf.timestamp_), acceptance_thres), true);
        // Add for printing.
        b.keyframes_[kf.timestamp_] = std::make_shared<Keyframe>(kf);
    } else {
        summary = b.solve();

        // compare before and after optimization
        auto poses_gt_iter = poses_gt.cbegin();
        auto poses_iter = b.keyframes_.cbegin();
        //    if (poses_gt.size() != b.keyframes_.size()) {
        //        throw std::runtime_error("poses_gt.size()=" + std::to_string(poses_gt.size()) +
        //                                 " != poses.size()=" +
        //                                 std::to_string(b.keyframes_.size()));
        //    }
        for (; poses_gt_iter != poses_gt.cend() && poses_iter != b.keyframes_.cend(); ++poses_gt_iter, ++poses_iter) {
            ASSERT_EQ(poses_iter->second->getEigenPose().isApprox(poses_gt_iter->second, acceptance_thres), true);
        }
    }

    std::cout << summary << std::endl;

    // print stuff
    {
        std::cout << "before bundling | after bundling | gt" << std::endl;
        auto poses_before_bundling_iter = noisy_poses.cbegin();
        auto b_keyframes_iter = b.keyframes_.cbegin();
        auto poses_gt_iter = poses_gt.cbegin();
        for (; poses_before_bundling_iter != noisy_poses.cend() && b_keyframes_iter != b.keyframes_.cend() &&
               poses_gt_iter != poses_gt.cend();
             ++poses_before_bundling_iter, ++b_keyframes_iter, ++poses_gt_iter) {

            auto converted_gt = convert(poses_gt_iter->second);
            auto converted_before_bundling = convert(poses_before_bundling_iter->second);
            for (int i = 0; i < int(7); ++i) {
                std::cout << converted_before_bundling[i] << "|" << b_keyframes_iter->second->pose_[i] << "|"
                          << converted_gt[i] << "\n";
            }

            std::cout << std::endl;
        }
    }
}
}

TEST(KeyFrameBundleAdjustment, solve_depth) {
    std::cout << "======================== Mono =======================" << std::endl;
    std::cout << "--------------------------- zero noise ----------------------------" << std::endl;
    evaluate_bundle_adjustment_depth(
        std::make_tuple(0., 0., 0.), std::make_tuple(0.0, 0., 0.0, 0.), 0.001, {Eigen::Isometry3d::Identity()});

    std::cout << "--------------------------- noise "
                 "=0.,0.,5pi/180,0.2,0.1,0.1----------------------------"
              << std::endl;
    evaluate_bundle_adjustment_depth(std::make_tuple(0., 0., 0.),
                                     std::make_tuple(5. * M_PI / 180., 0.2, 0.1, 0.1),
                                     0.001,
                                     {Eigen::Isometry3d::Identity()});

    std::cout << "--------------------------- noise "
                 "=0.1,0.1,5pi/180,0.2,0.1,0.1----------------------------"
              << std::endl;
    evaluate_bundle_adjustment_depth(std::make_tuple(1.5, 1.5, 0.),
                                     std::make_tuple(5. * M_PI / 180., 0.2, 0.1, 0.1),
                                     0.01,
                                     {Eigen::Isometry3d::Identity()});

    std::cout << "======================== Multi cam =======================" << std::endl;
    std::vector<Eigen::Isometry3d> trf_camI_vehicle;
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();

    p.rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(1., 0., 0.)));
    p.rotate(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d(0., 0., 1.)));
    p.translate(Eigen::Vector3d(-1.5, 0.2, -1.35));

    p = p.inverse();

    trf_camI_vehicle.push_back(p);

    p.translate(Eigen::Vector3d(0., -0.5, 0.));
    p.rotate(Eigen::AngleAxisd(M_PI / 18., Eigen::Vector3d(0., 1., 0.)));
    p.rotate(Eigen::AngleAxisd(M_PI / 18., Eigen::Vector3d(1., 0., 0.)));
    trf_camI_vehicle.push_back(p);

    std::cout << p.matrix() << std::endl;

    evaluate_bundle_adjustment_depth(
        std::make_tuple(0., 0., 0.), std::make_tuple(0.0, 0., 0.0, 0.), 0.001, trf_camI_vehicle);

    std::cout << "--------------------------- noise "
                 "=0.,0.,5pi/180,0.2,0.1,0.1----------------------------"
              << std::endl;
    evaluate_bundle_adjustment_depth(
        std::make_tuple(0., 0., 0.), std::make_tuple(5. * M_PI / 180., 0.2, 0.1, 0.1), 0.001, trf_camI_vehicle);

    std::cout << "--------------------------- noise "
                 "=0.1,0.1,5pi/180,0.2,0.1,0.1----------------------------"
              << std::endl;
    evaluate_bundle_adjustment_depth(
        std::make_tuple(1.5, 1.5, 0.), std::make_tuple(5. * M_PI / 180., 0.2, 0.1, 0.1), 0.01, trf_camI_vehicle);
}


TEST(LandmarkCreator, CreateWithDepth) {
    // Test if 3d landmarks are created correctly by using tracklets with depth bias

    // calc data on keyframes
    std::vector<Eigen::Vector3d> lms_origin{Eigen::Vector3d(0.5, 3., 5.5),
                                            Eigen::Vector3d(0., 1., -20.),
                                            Eigen::Vector3d(1., -5., 4.),
                                            Eigen::Vector3d(2.0, 1., 1.5),
                                            Eigen::Vector3d(-2.0, -1., 10.)};

    std::map<LandmarkId, Landmark> lms_arr_origin;
    int count = 0;
    for (const auto& el : lms_origin) {
        lms_arr_origin[count] = Landmark(el);
        count++;
    }

    std::map<keyframe_bundle_adjustment::TimestampNSec, Eigen::Isometry3d> poses =
        getPoses(0., std::make_tuple(0., 0., 0.), {0, 1, 2, 3, 4});

    Camera cam(600, {300, 200}, Eigen::Isometry3d::Identity());
    Camera::Ptr cam_ptr = std::make_shared<Camera>(cam);
    auto ts = makeTrackletsDepth(poses, lms_origin, std::map<CameraId, Camera::Ptr>{{0, cam_ptr}});

    std::map<KeyframeId, Keyframe> kfs;
    int count2 = 0;
    for (const auto& el : poses) {
        kfs[count2] = Keyframe(count2, ts, cam_ptr, el.second);
        count2++;
    }

    BundleAdjusterKeyframes adjuster;
    adjuster.set_solver_time(20.);

    for (const auto& kf : kfs) {
        adjuster.push(kf.second);
    }

    const auto& landmarks_calc = adjuster.landmarks_;

    for (const auto& lm : lms_arr_origin) {
        std::cout << "get landmark with id: " << lm.first << std::endl;

        if (landmarks_calc.count(lm.first) == 0)
            std::cout << "no landmark with id: " << lm.first << std::endl;

        std::cout << "Original landmark position: "
                  << "x: " << lm.second.pos.at(0) << ", y: " << lm.second.pos.at(1) << ", z: " << lm.second.pos.at(2)
                  << std::endl;

        const auto& calc_lm = landmarks_calc.at(lm.first);

        std::cout << "Calculated landmark position: "
                  << "x: " << calc_lm->pos.at(0) << ", y: " << calc_lm->pos.at(1) << ", z: " << calc_lm->pos.at(2)
                  << std::endl;

        Eigen::Vector3d lm_origin_eigen(lm.second.pos.at(0), lm.second.pos.at(1), lm.second.pos.at(2));
        Eigen::Vector3d lm_calc_eigen(calc_lm->pos.at(0), calc_lm->pos.at(1), calc_lm->pos.at(2));

        ASSERT_NEAR((lm_origin_eigen - lm_calc_eigen).norm(), 0., 0.01);
    }
}

TEST(CostFunctors, motion_regularization) {
    using namespace keyframe_bundle_adjustment;

    regularization::MotionModelRegularization cost_func;

    // Yaw only
    {
        double yaw = 10. / 180. * M_PI;
        double l = 1.;
        double x = l / yaw * std::sin(yaw);
        double y = l / yaw * (1 - std::cos(yaw));
        //        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        //                             Eigen::AngleAxisd(yaw / 2., Eigen::Vector3d::UnitX()));
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        std::array<double, 7> p0{{1., 0., 0., 0., 0., 0., 0.}};
        std::array<double, 7> p1{{q.w(), q.x(), q.y(), q.z(), x, y, 0.}};


        std::array<double, 1> res;
        cost_func(p1.data(), p0.data(), res.data());

        ASSERT_NEAR(res[0], 0., 1e-10);
    }
}
/*TEST(IndexedHistogram, main) {
    std::vector<double> data(50);
    std::iota(data.begin(), data.end(), -10);
    std::shuffle(data.begin(), data.end(), std::mt19937{std::random_device{}()});

    int num_bins = 5;
    IndexedHistogram hist(num_bins, -5, 30);
    hist.addData([&data](const int& a) { return data[a]; }, data.size());

    auto out = hist.get();
    // One bin more for outliers at end.
    ASSERT_EQ(out.size(), num_bins + 1);
    for (const auto& el : out) {
        std::cout << "bin=" << el.first << " els:\n";
        for (const int& ind : el.second) {
            std::cout << ind << " ";
        }
        std::cout << std::endl << "val:\n";
        for (const auto& ind : el.second) {
            ASSERT_LE(data[ind], el.first);
            std::cout << data[ind] << " ";
        }
        std::cout << std::endl;
    }

    std::cout << "done" << std::endl;
}

TEST(VoxelGrid, polar) {
    // Generate data

    std::vector<std::array<double, 3>> lms_origin{
        {{10., 3., 5.5}}, {{11., 1., 6.5}}, {{14., -5., 6.}}, {{9., 1., 5.}}, {{16., -1., 4.}}};

    VoxelGridPolar grid;
    grid.setData(std::make_shared<VoxelGridBase::Lms>(lms_origin));
    {
        auto out = grid.get();
        std::cout << out << std::endl;
        {
            std::stringstream ss;
            ss << "/tmp/debug_grid_inidices.txt";
            std::ofstream file(ss.str().c_str());
            file.precision(12);
            file << out.printGrid().str();
            file.close();
        }
        {
            std::stringstream ss;
            ss << "/tmp/debug_grid_points.txt";
            std::ofstream file(ss.str().c_str());
            file.precision(12);
            for (const auto& el : lms_origin) {
                file << el[0] << " " << el[1] << " " << el[2] << std::endl;
            }
            file.close();
        }
        for (const auto& el : out.data_) {
            const double& a = el.first;
            for (const auto& el2 : el.second) {
                const auto& d = el2.first;
                if (!el2.second.empty()) {
                    std::cout << a << " " << d << std::endl;
                    std::cout << d * std::cos(a) << " " << d * std::sin(a) << std::endl;
                }
            }
        }
        ASSERT_EQ(out.size(), lms_origin.size());
        auto non_zero_els = out.getNonZeroElements();

        ASSERT_EQ(non_zero_els[0].indices.size(), 1);
        ASSERT_NEAR(non_zero_els[0].dim0, -0.174533, 1e-4);
        ASSERT_NEAR(non_zero_els[0].dim1, 20.7812, 1e-4);
        ASSERT_EQ(non_zero_els[0].indices[0], 2);
        ASSERT_EQ(non_zero_els[1].indices.size(), 1);
        ASSERT_NEAR(non_zero_els[1].dim0, 0., 1e-4);
        ASSERT_NEAR(non_zero_els[1].dim1, 20.7812, 1e-4);
        ASSERT_EQ(non_zero_els[1].indices[0], 4);
        ASSERT_EQ(non_zero_els[2].indices.size(), 2);
        ASSERT_NEAR(non_zero_els[2].dim0, 0.174533, 1e-4);
        ASSERT_NEAR(non_zero_els[2].dim1, 13.1875, 1e-4);
        ASSERT_EQ(non_zero_els[2].indices[0], 1);
        ASSERT_EQ(non_zero_els[2].indices[1], 3);
        ASSERT_EQ(non_zero_els[3].indices.size(), 1);
        ASSERT_NEAR(non_zero_els[3].dim0, 0.349066, 1e-4);
        ASSERT_NEAR(non_zero_els[3].dim1, 13.1875, 1e-4);
        ASSERT_EQ(non_zero_els[3].indices[0], 0);
    }

    grid.sparsify();
    {
        auto out = grid.get();
        {
            std::stringstream ss;
            ss << "/tmp/debug_sparsified_grid_indices.txt";
            std::ofstream file(ss.str().c_str());
            file.precision(12);
            file << out.printGrid().str();
            file.close();
        }
        std::cout << out << std::endl;
        for (const auto& el : out.data_) {
            const double& a = el.first;
            for (const auto& el2 : el.second) {
                const auto& d = el2.first;
                if (!el2.second.empty()) {
                    std::cout << a << " " << d << std::endl;
                    std::cout << d * std::cos(a) << " " << d * std::sin(a) << std::endl;
                }
            }
        }
        ASSERT_EQ(out.size(), 4);
    }
}
TEST(VoxelGrid, cartesian) {
    // Generate data

    std::vector<std::array<double, 3>> lms_origin{{{10., 3., 5.5}},
                                                  {{11., 1., 6.5}},
                                                  {{11., 1.3, 6.5}},
                                                  {{14., -5., 6.}},
                                                  {{9., 1., 5.}},
                                                  {{9.7, 1., 5.}},
                                                  {{16., -1., 4.}}};

    VoxelGridCartesian::Parameters params;
    params.max_x = 50.;
    params.min_x = -50.;
    params.num_bins_x = 100;
    params.max_y = 50.;
    params.min_y = -50.;
    params.num_bins_y = 100;

    VoxelGridCartesian grid;
    grid.setParameters(params);
    grid.setData(std::make_shared<VoxelGridBase::Lms>(lms_origin));

    std::cout << grid.get() << std::endl;
    ASSERT_EQ(grid.get().getNonZeroElements().size(), 5);
    ASSERT_EQ(grid.getIndices().size(), 7);

    grid.sparsify();

    std::cout << grid.get() << std::endl;
    ASSERT_EQ(grid.get().getNonZeroElements().size(), 5);
    ASSERT_EQ(grid.getIndices().size(), 5);
}

TEST(LandmarkSelector, voxel) {
    using namespace keyframe_bundle_adjustment;

    // calc data on keyframes
    std::vector<Eigen::Vector3d> lms_origin{Eigen::Vector3d(0.5, 3., 5.5),
                                            Eigen::Vector3d(0., 100., 30.),
                                            Eigen::Vector3d(1., -5., 4.),
                                            Eigen::Vector3d(2.0, 1., 1.5),
                                            Eigen::Vector3d(-2.0, -1., 10.),
                                            Eigen::Vector3d(-1.95, -0.99, 10.1),
                                            Eigen::Vector3d(0.5, 3.01, 5.52)};
    std::map<LandmarkId, Landmark::ConstPtr> lms_arr_origin;
    int count = 0;
    for (const auto& el : lms_origin) {
        lms_arr_origin[count] = std::make_shared<const Landmark>(el);
        count++;
    }

    std::map<keyframe_bundle_adjustment::TimestampNSec, Eigen::Isometry3d> poses =
        getPoses(0., std::make_tuple(0., 0., 0.), {0, 1, 2, 3, 4});


    Camera cam(600, {300, 200}, Eigen::Isometry3d::Identity());
    Camera::Ptr cam_ptr = std::make_shared<Camera>(cam);
    auto ts = makeTrackletsDepth(poses, lms_origin, std::map<CameraId, Camera::Ptr>{{0, cam_ptr}});

    std::map<KeyframeId, Keyframe::ConstPtr> kf_const_ptrs;
    int count2 = 0;
    for (const auto& el : poses) {
        kf_const_ptrs[count2] = std::make_shared<const Keyframe>(Keyframe(count2, ts, cam_ptr, el.second));
        count2++;
    }
    LandmarkSelector selector;

    LandmarkSelectionSchemeVoxel::Parameters p;
    p.max_num_landmarks_far = 50;
    p.max_num_landmarks_middle = 50;
    p.max_num_landmarks_near = 50;
    p.roi_far_xyz = std::array<double, 3>{{30., 30., 30.}};
    p.roi_middle_xyz = std::array<double, 3>{{10., 10., 10.}};

    //        p.depth_params.min = 1.0;
    //        p.depth_params.max = 30.0;
    //        p.depth_params.bin_size = 0.7;
    //        p.depth_params.bin_size_scale_factor = 1.1;
    selector.addScheme(LandmarkSelectionSchemeVoxel::create(p));

    auto selected_lms = selector.select(lms_arr_origin, kf_const_ptrs);

    for (const auto& el : selected_lms) {
        std::cout << el << std::endl;
    }

    // the behaviour of the radius rejection is strange.
    // If we set number neighbours to 1 both lms are rejected.
    // If we set 2 both remain.
    ASSERT_EQ(selected_lms.size(), 5);

    // test if categorizer interface works
    ASSERT_EQ(selector.getLandmarkCategories().size(), 5);
}
*/
TEST(BundleAdjusterKeyframes, adjustMotionOnly) {

    evaluate_bundle_adjustment_depth(
        std::make_tuple(0., 0., 0.), std::make_tuple(0.0, 0., 0.0, 0.), 0.001, {Eigen::Isometry3d::Identity()}, true);
}
