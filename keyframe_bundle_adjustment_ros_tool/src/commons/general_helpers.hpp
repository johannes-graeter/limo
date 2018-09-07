// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <cv.hpp>
#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>
#include <matches_msg_types/tracklets.hpp>
#include <commons/color_by_index_hsv.hpp>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/eigen.hpp> //attention, eigen must be icluded before that!

namespace keyframe_bundle_adjustment_ros_tool {

using CvPoints = std::vector<cv::Point2d>;

namespace helpers {

std::string poseToString(Eigen::Matrix<double, 4, 4> m) {
    std::stringstream ss;
    ss << m(0, 0) << " " << m(0, 1) << " " << m(0, 2) << " " << m(0, 3) << " " << m(1, 0) << " " << m(1, 1) << " "
       << m(1, 2) << " " << m(1, 3) << " " << m(2, 0) << " " << m(2, 1) << " " << m(2, 2) << " " << m(2, 3);
    return ss.str();
}

std::set<int> loadSetFromYaml(std::string yaml_path, std::string field_name) {
    std::set<int> out;
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (root[field_name] && root[field_name].IsSequence()) {
        for (const auto& el : root[field_name]) {
            out.insert(el.as<int>());
        }
    } else {
        throw std::runtime_error("LabelReader: vector outlier_labels not defined.");
    }
    return out;
}

std::tuple<CvPoints, CvPoints> getMatches(const matches_msg_types::Tracklets& tracklets,
                                          keyframe_bundle_adjustment::TimestampNSec ts0,
                                          keyframe_bundle_adjustment::TimestampNSec ts1,
                                          const std::set<int>& outlier_labels) {

    // get indices of stamps
    int index0;
    {
        auto iter = std::find(tracklets.stamps.cbegin(), tracklets.stamps.cend(), ts0);
        index0 = std::distance(tracklets.stamps.cbegin(), iter);
    }

    int index1;
    {
        auto iter = std::find(tracklets.stamps.cbegin(), tracklets.stamps.cend(), ts1);
        index1 = std::distance(tracklets.stamps.cbegin(), iter);
    }


    // get matches corresponding to timest
    CvPoints points0;
    CvPoints points1;

    for (const auto& track : tracklets.tracks) {
        if (int(track.feature_points.size()) > index0 && int(track.feature_points.size()) > index1 &&
            outlier_labels.find(track.label) == outlier_labels.cend()) {
            points0.push_back(cv::Point2d(track.feature_points[index0].u, track.feature_points[index0].v));
            points1.push_back(cv::Point2d(track.feature_points[index1].u, track.feature_points[index1].v));
        }
    }

    return std::make_tuple(points0, points1);
}

double getMeanFlow(const CvPoints& points0, const CvPoints& points1) {
    double out = 0.;
    if (points0.size() != points1.size()) {
        throw std::runtime_error("In getMeanFlow: points size not consistent.");
    }
    if (points0.size() == 0) {
        return 0.;
    }

    for (int i = 0; i < int(points0.size()); ++i) {
        out += cv::norm(points0[i] - points1[i]);
    }
    out /= static_cast<double>(points0.size());
    return out;
}

/**
 * @brief calcMotion5Point, calc motion with 5 point algo from opencv
 * @param motion, motino beetween frames will be changed if successful
 * @param points0, points from first frame
 * @param points1, points from second frame
 * @param focal, focal length
 * @param pp, principal point
 * @return flag if sucessfull==true
 */
bool calcMotion5Point(Eigen::Isometry3d& motion,
                      const CvPoints& points0,
                      const CvPoints& points1,
                      double focal,
                      cv::Point2d pp,
                      double probability,
                      double flow_thres = 3.0) {

    if (points0.size() == 0 || points1.size() == 0) {
        ROS_DEBUG_STREAM("no points for 5 point available!");
        return false;
    }

    if (getMeanFlow(points0, points1) < flow_thres) {
        motion.translation() = Eigen::Vector3d(0., 0., 0.);
        ROS_DEBUG_STREAM("in 5-point: not enough flow! Return motion=\n" << motion.matrix());
        return false;
    }

    cv::Mat essential_mat = cv::findEssentialMat(points1, points0, focal, pp, cv::RANSAC, probability, 2.0);
    if (essential_mat.rows != 3 || essential_mat.cols != 3) {
        return false;
    }


    cv::Mat R, t;
    int number_inliers = cv::recoverPose(essential_mat, points1, points0, R, t, focal, pp);

    Eigen::Matrix3d r_eigen;
    Eigen::Vector3d t_eigen;
    cv::cv2eigen(R, r_eigen);
    cv::cv2eigen(t, t_eigen);

    motion.linear() = r_eigen;
    motion.translation() = t_eigen;

    return true;
}


/**
 * @brief getFlowImg, plot selected measurements from keyframe
 * @param bundle_adjuster
 * @return img with a lot of dots
 */
cv::Mat getFlowImg(keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster) {
    cv::Mat debug_img(600, 1300, CV_8UC3);
    debug_img.setTo(0);

    int num_colors = 10;

    for (const auto& kf : bundle_adjuster->keyframes_) {
        if (kf.second->is_active_) {
            for (const auto& m : kf.second->measurements_) {
                const auto& m_id = m.first;
                const auto& meas = m.second.cbegin()->second;

                auto color = util_image::get_color(m_id, num_colors);

                cv::circle(debug_img, cv::Point(meas.u, meas.v), 1, color, -1);
            }
        }
    }

    return debug_img;
}

void dumpMap(std::string filename, const keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr& bundle_adjuster_) {
    std::ofstream file(filename.c_str());
    file.precision(12);

    file << "landmarks with depth: [";
    for (const auto& id_lm_ptr : bundle_adjuster_->landmarks_) {
        if (id_lm_ptr.second->has_measured_depth) {
            file << "[" << id_lm_ptr.second->pos[0] << ", " << id_lm_ptr.second->pos[1] << ", "
                 << id_lm_ptr.second->pos[2] << ", " << id_lm_ptr.second->weight << "],\n";
        }
    }
    file << "]\n";

    file << "landmarks without depth: [";
    for (const auto& id_lm_ptr : bundle_adjuster_->landmarks_) {
        if (!id_lm_ptr.second->has_measured_depth) {
            file << "[" << id_lm_ptr.second->pos[0] << ", " << id_lm_ptr.second->pos[1] << ", "
                 << id_lm_ptr.second->pos[2] << ", " << id_lm_ptr.second->weight << "],\n";
        }
    }
    file << "]\n";


    file << "poses: {";
    for (const auto& id_kf_ptr : bundle_adjuster_->keyframes_) {
        file << id_kf_ptr.second->timestamp_ << ": [" << id_kf_ptr.second->pose_[0] << ", "
             << id_kf_ptr.second->pose_[1] << ", " << id_kf_ptr.second->pose_[2] << ", " << id_kf_ptr.second->pose_[3]
             << ", " << id_kf_ptr.second->pose_[4] << ", " << id_kf_ptr.second->pose_[5] << ", "
             << id_kf_ptr.second->pose_[6] << "],\n";
    }
    file << "}";

    file.close();

    std::cout << "--------------------------------------\nDumped map to " << filename
              << "--------------------------------------\n"
              << std::endl;
}

Eigen::Isometry3d getMotionUnscaled(double focal_length,
                                    cv::Point2d princ_point,
                                    const keyframe_bundle_adjustment::TimestampNSec& cur_ts,
                                    const keyframe_bundle_adjustment::TimestampNSec& ts_last_kf,
                                    const keyframe_bundle_adjustment::Tracklets& tracklets,
                                    const Eigen::Isometry3d& trf_camera_vehicle,
                                    double speed_m_per_second = 13.) {
    CvPoints last_points, cur_points;
    std::tie(last_points, cur_points) = helpers::getMatches(tracklets, ts_last_kf, cur_ts, {23, 24, 25, 26});

    Eigen::Isometry3d motion_camera_t0_t1 = Eigen::Isometry3d::Identity();
    motion_camera_t0_t1.translation() = Eigen::Vector3d(0., 0., 1.);
    helpers::calcMotion5Point(motion_camera_t0_t1, last_points, cur_points, focal_length, princ_point, 0.999, 5.);

    // Scale translation so that we drive at approx 45 km/h.
    double dt = keyframe_bundle_adjustment::convert(cur_ts) - keyframe_bundle_adjustment::convert(ts_last_kf);
    motion_camera_t0_t1.translation() = motion_camera_t0_t1.translation() /
                                        std::max(0.0001, motion_camera_t0_t1.translation().norm()) *
                                        speed_m_per_second * dt;
    Eigen::Isometry3d motion_vehicle_t1_t0 =
        trf_camera_vehicle.inverse() * motion_camera_t0_t1.inverse() * trf_camera_vehicle;
    return motion_vehicle_t1_t0;
}
}
}
