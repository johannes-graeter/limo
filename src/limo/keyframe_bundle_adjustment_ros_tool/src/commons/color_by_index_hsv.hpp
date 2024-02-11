/*
 * Copyright 2014. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */
#ifndef COLORBYINDEX_HPP
#define COLORBYINDEX_HPP

#include <opencv2/opencv.hpp>

namespace util_image {
/// /////////////////////////////////////////////
/// @brief function to convert color in hsv to bgr
/// @return color in bgr
/// @par color in hsv
///
/// uses the intern opencv color representation, so max s,v = 255 max(h)=180 (increase of one means
/// increase of 2°)
inline cv::Scalar hsv2bgr(cv::Scalar ColorHSV) {
    cv::Mat old_color(1, 1, CV_8UC3, ColorHSV);
    cv::Mat new_color(1, 1, CV_8UC3);
    cv::cvtColor(old_color, new_color, cv::COLOR_HSV2BGR);
    return cv::Scalar(new_color.at<cv::Vec3b>(0, 0));
}
inline cv::Scalar get_color(uint32_t ID, int NumColors) {
    if (ID == 0) {
        return cv::Scalar(123, 22, 234);
    } else {
        uint32_t ModID = ID - 1;
        ModID = ModID % NumColors;

        int dH = 180 / NumColors;
        int S = 200;
        int V = 200;

        return hsv2bgr(cv::Scalar(ModID * dH, S, V));
    }
}

inline cv::Scalar get_color(uint32_t ID) {
    // seed generator with ID -> always same color
    cv::RNG rng(ID);
    // Generate number from uniform distribution
    return cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
}
}
#endif // COLORBYINDEX_HPP
