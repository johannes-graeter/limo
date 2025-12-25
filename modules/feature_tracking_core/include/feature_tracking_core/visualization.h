#pragma once

#include "tracklet.h"

namespace cv {
class Mat;
}

namespace feature_tracking {

namespace visualization {


void drawMatches(const feature_tracking::StereoTrackletList& tracklets,
                 const std::vector<std::pair<cv::Mat, cv::Mat>>& images,
                 cv::Mat& output,
                 int size = 3,
                 int thickness = 1,
                 int linetype = 8);

void drawMatches(const feature_tracking::TrackletList& tracklets,
                 const std::vector<cv::Mat>& images,
                 cv::Mat& output,
                 int size = 3,
                 int thickness = 1,
                 int linetype = 8);


} // namespace visualization

} // namespace feature_tracking
