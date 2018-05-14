// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <Eigen/Eigen>
#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>

// for publishing landmarks
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <tf2_eigen/tf2_eigen.h>

#include <nav_msgs/Path.h>

#include <ros/ros.h>

namespace keyframe_bundle_adjustment_ros_tool {

namespace helpers {

/**
 * @brief toGeometryMsg, convert to geometry_msg style  pose
 * @param p, pose eigen isometry
 * @return geometry_msg style pose
 */
template<int TransformType>
void toGeometryMsg(geometry_msgs::Pose& out, const Eigen::Transform<double, 3, TransformType>& pose) {
    // convert accumulated_pose_ to transformStamped
    Eigen::Quaterniond rot_quat(pose.rotation());

    out.position.x = pose.translation().x();
    out.position.y = pose.translation().y();
    out.position.z = pose.translation().z();

    out.orientation.x = rot_quat.x();
    out.orientation.y = rot_quat.y();
    out.orientation.z = rot_quat.z();
    out.orientation.w = rot_quat.w();
}

///@brief overload for transform
template<int TransformType>
void toGeometryMsg(geometry_msgs::Transform& out, const Eigen::Transform<double, 3, TransformType>& pose) {
    // convert accumulated_pose_ to transformStamped
    Eigen::Quaterniond rot_quat(pose.rotation());

    out.translation.x = pose.translation().x();
    out.translation.y = pose.translation().y();
    out.translation.z = pose.translation().z();

    out.rotation.x = rot_quat.x();
    out.rotation.y = rot_quat.y();
    out.rotation.z = rot_quat.z();
    out.rotation.w = rot_quat.w();
}

namespace {
using Category = keyframe_bundle_adjustment::LandmarkCategorizatonInterface::Category;
float getColorVal(const keyframe_bundle_adjustment::LandmarkId& lm_id,
                  const std::map<keyframe_bundle_adjustment::LandmarkId, Category>& lm_categories) {
    // Choose value of color according to category.
    // Outliers are not present in category -> val=20
    float val = 20.;
    auto it = lm_categories.find(lm_id);
    if (it != lm_categories.cend()) {
        switch (it->second) {
        case Category::FarField:
            val = 70.;
            break;
        case Category::MiddleField:
            val = 150.;
            break;
        case Category::NearField:
            val = 250.;
            break;
        default:
            break;
        }
    }
    return val;
}
}

/**
 * @brief publishLandmarks, publish the landmarks as sensor_msgs::PointCloud2 cloud; frame_id is
 * global coordinate system; timestamp is the one of last keyframe
 * @param landmarks_publisher
 * @param bundle_adjuster
 * @param tf_parent_frame_id
 */
void publishLandmarks(ros::Publisher& landmarks_publisher,
                      keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                      std::string tf_parent_frame_id) {
    using PointType = pcl::PointXYZRGB;
    using PointCloud = pcl::PointCloud<PointType>;

    ros::Time stamp;
    stamp.fromNSec(bundle_adjuster->getKeyframe().timestamp_);

    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = "/" + tf_parent_frame_id;
    msg->header.stamp = stamp.toNSec();
    msg->height = 1;


    // plot selected landmarks in green
    const auto& selected_lms = bundle_adjuster->getSelectedLandmarkConstPtrs();

    // get difference to active landmarks -> outlier
    std::vector<std::pair<keyframe_bundle_adjustment::LandmarkId, keyframe_bundle_adjustment::Landmark::ConstPtr>> diff;
    const auto& active_lms = bundle_adjuster->getActiveLandmarkConstPtrs();
    // see https://www.fluentcpp.com/2017/01/09/know-your-algorithms-algos-on-sets/
    std::set_symmetric_difference(selected_lms.cbegin(),
                                  selected_lms.cend(),
                                  active_lms.cbegin(),
                                  active_lms.cend(),
                                  std::inserter(diff, diff.end()),
                                  [](const auto& a, const auto& b) { return a.first < b.first; });
    // size of pointcloud should be equal to atctive_lms
    msg->width = active_lms.size();

    // Get categories to color landmarks wether if the are in near, middle or far field
    std::map<keyframe_bundle_adjustment::LandmarkId,
             keyframe_bundle_adjustment::LandmarkCategorizatonInterface::Category>
        lm_categories = bundle_adjuster->landmark_selector_->getLandmarkCategories();

    // Plot inliers(selected lms) green.
    for (const auto& lm_id_pos : selected_lms) {
        const auto& lm = *(lm_id_pos.second);

        float val = getColorVal(lm_id_pos.first, lm_categories);

        std::array<float, 3> color_rgb{0., val, 0.};

        if (lm.has_measured_depth) {
            color_rgb = std::array<float, 3>{val, val, 0.};
        }

        PointType p;
        p.x = lm.pos[0];
        p.y = lm.pos[1];
        p.z = lm.pos[2];
        p.r = color_rgb[0];
        p.g = color_rgb[1];
        p.b = color_rgb[2];

        msg->points.push_back(p);
    }

    // plot outliers red
    for (const auto& lm_id : diff) {
        const auto& lm = *(active_lms.at(lm_id.first));

        float val = getColorVal(lm_id.first, lm_categories);

        std::array<float, 3> color_rgb{val, 0., 0.};

        PointType p;
        p.x = lm.pos[0];
        p.y = lm.pos[1];
        p.z = lm.pos[2];
        p.r = color_rgb[0];
        p.g = color_rgb[1];
        p.b = color_rgb[2];

        msg->points.push_back(p);
    }

    landmarks_publisher.publish(msg);
}

/**
 * @brief publishPath, publish
 * two nav_msgs::path that can be
 * shown in rviz, one for frames that are optimized at the moemnt and one for all frames;frame id is
 * same as tf_parent, timestamp of msg is timestamp of the most
 * current keyframe;
 * @param path_publisher
 * @param active_path_publisher
 * @param bundle_adjuster
 */
void publishPaths(ros::Publisher& path_publisher,
                  ros::Publisher& active_path_publisher,
                  keyframe_bundle_adjustment::BundleAdjusterKeyframes::Ptr bundle_adjuster,
                  std::string tf_parent_frame_id) {
    nav_msgs::Path path_msg;

    // timestamp of msg is same as last keyframe
    ros::Time cur_ts;
    cur_ts.fromNSec(bundle_adjuster->getKeyframe().timestamp_);
    path_msg.header.stamp = cur_ts;
    // frame id of msg is same as tf_parent without tf2 convention
    path_msg.header.frame_id = "/" + tf_parent_frame_id;

    nav_msgs::Path active_path_msg = path_msg;

    for (const auto& kf : bundle_adjuster->keyframes_) {
        geometry_msgs::PoseStamped cur_pose;
        ros::Time p_ts;
        p_ts.fromNSec(kf.second->timestamp_);
        cur_pose.header.stamp = p_ts;
        cur_pose.header.frame_id = path_msg.header.frame_id;
        //            ROS_DEBUG_STREAM("origin in cur_pose=\n"
        //                             << kf.second.getEigenPose().translation().transpose());

        toGeometryMsg(cur_pose.pose, kf.second->getEigenPose().inverse());

        path_msg.poses.push_back(cur_pose);

        if (kf.second->is_active_) {
            active_path_msg.poses.push_back(cur_pose);
        }
    }

    path_publisher.publish(path_msg);
    active_path_publisher.publish(active_path_msg);
}
}
}
