/*
 * parameters.h
 *
 *  Created on: Sep 14, 2017
 *      Author: wilczynski
 */

#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>

namespace tracklets_depth {

class TrackletDepthParameters {
public:
    void print();
    void fromFile(const std::string& filePath);

    // subscriber
    std::string subscriber_msg_name_cloud;
    std::string subscriber_msg_name_camera_info;
    std::string subscriber_msg_name_tracklets;
    std::string subscriber_msg_name_semantics;

    // publisher
    std::string publisher_msg_name_cloud_interpolated;
    std::string publisher_msg_name_cloud_camera_cs;
    std::string publisher_msg_name_image_projection_cloud;
    std::string publisher_msg_name_depthcalc_stats;
    std::string publisher_msg_name_tracklets_depth;

    // frame names
    std::string tf_frame_name_cameraLeft;
    std::string tf_frame_name_velodyne;

    // Misc
    int msg_queue_size;
};
}
