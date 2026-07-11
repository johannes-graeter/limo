/*
 * parameters.cpp
 *
 *  Created on: Sep 14, 2017
 *      Author: wilczynski
 */

#include "tracklets_depth/parameters.h"

namespace tracklets_depth {

void TrackletDepthParameters::fromFile(const std::string& filePath) {
    using namespace std;

    cout << "Load settings from: " << filePath << endl << endl;

    cv::FileStorage fSettings(filePath, cv::FileStorage::READ);

    if (!fSettings.isOpened())
        throw "Cant find settings file";

    // subscriber
    subscriber_msg_name_cloud = (string)fSettings["subscriber_msg_name_cloud"];
    subscriber_msg_name_camera_info = (string)fSettings["subscriber_msg_name_camera_info"];
    subscriber_msg_name_tracklets = (string)fSettings["subscriber_msg_name_tracklets"];
    subscriber_msg_name_semantics = (string)fSettings["subscriber_msg_name_semantics"];

    // publisher
    publisher_msg_name_cloud_interpolated = (string)fSettings["publisher_msg_name_cloud_interpolated"];
    publisher_msg_name_cloud_camera_cs = (string)fSettings["publisher_msg_name_cloud_camera_cs"];
    publisher_msg_name_image_projection_cloud = (string)fSettings["publisher_msg_name_image_projection_cloud"];
    publisher_msg_name_depthcalc_stats = (string)fSettings["publisher_msg_name_depthcalc_stats"];
    publisher_msg_name_tracklets_depth = (string)fSettings["publisher_msg_name_tracklets_depth"];

    // frame name
    tf_frame_name_cameraLeft = (string)fSettings["tf_frame_name_cameraLeft"];
    tf_frame_name_velodyne = (string)fSettings["tf_frame_name_velodyne"];

    // misc
    msg_queue_size = (int)fSettings["msg_queue_size"];
}

void TrackletDepthParameters::print() {
}
}
