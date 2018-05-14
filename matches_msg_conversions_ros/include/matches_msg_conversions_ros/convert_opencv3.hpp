/*
 * Copyright 2017. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */
#pragma once

#include <matches_msg_ros/MatchesMsg.h>
#include <opencv2/opencv.hpp>

namespace matches_msg_conversions_ros {
using PointsVec=std::vector<cv::Point2f>;
using LastCurPoints = std::pair<PointsVec,PointsVec>;

///@brief convert to opencv3
LastCurPoints ConvertF2F(const matches_msg_ros::MatchesMsg::ConstPtr);
} // end of ns
