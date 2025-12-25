#pragma once

// standard includes
#include <feature_tracking_core/tracker_libviso.h>
#include <matches_msg_ros/MatchesMsg.h>
#include <ros/ros.h>
#include "viso_feature_tracking/viso_feature_tracking_parameters.h"
#include <opencv2/opencv.hpp>

#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <opencv2/core/core.hpp>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace viso_feature_tracking {
class VisoFeatureTrackerModule {
 public:
  VisoFeatureTrackerModule(const Configuration& config, const Parameters& params)
      : config_(config), viso_params_(params){
        tracker_ = std::make_shared<feature_tracking::TrackerLibViso>(viso_params_);
      };
  
  // Process the image and perform feature matching.
  void process(cv::Mat& src, const ros::Time& time);

  // Erase old timestamps when the sample size exceeds the limit. 
  void reduce();
  matches_msg_ros::MatchesMsg get_matches_msg();

 private:
  ///@brief pointer to tracker
  std::shared_ptr<feature_tracking::TrackerLibViso> tracker_;

  ///@brief store info that shall be published, from current to old
  feature_tracking::TrackletVector tracklets_;

  ///@brief timestamps storage for matches, descending order (from current to
  /// old)
  std::deque<ros::Time> timestamps_;

  const Configuration config_;
  const Parameters viso_params_;

  ///@brief maximum size of timestamps_
  int max_size_timestamps_ = 1000;
};
}  // namespace viso_feature_tracking