#include "viso_feature_tracking/viso_feature_tracker_module.h"

namespace viso_feature_tracking {

///@brief release build assert
void Assert(bool condition, std::string error_message) {
    if (!condition) {
        throw std::runtime_error("in viso_feature_tracking: " + error_message);
    }
}

void VisoFeatureTrackerModule::process(cv::Mat& src, const ros::Time& time) {
  if (config_.scale_factor != 1.0) {
    cv::Size imageSize = src.size();
    cv::resize(src, src,
               cv::Size(imageSize.width * config_.scale_factor,
                        imageSize.height * config_.scale_factor));
  }

  cv::GaussianBlur(src, src,
                   cv::Size(viso_params_.blur_size, viso_params_.blur_size),
                   viso_params_.blur_sigma);

  tracker_->pushBack(src);
  tracker_->getTracklets(tracklets_, 0);
  this->timestamps_.push_front(time);

    // static std::vector<cv::Mat> images;
    // images.insert(images.begin(), cv_bridge_ptr->image);
    // cv::Mat output;
    // feature_tracking::visualization::drawMatches(tracklet_list, images, output);
    // static int cnt = 0;
    // std::string img_fname = "/tmp/out_" + std::to_string(cnt++) + ".png";
    // cv::imwrite(img_fname, output);


}

void VisoFeatureTrackerModule::reduce() {

    // cut timestamps
    if (static_cast<int>(timestamps_.size()) > max_size_timestamps_) {
        timestamps_.pop_back();
    }
}

matches_msg_ros::MatchesMsg VisoFeatureTrackerModule::get_matches_msg() {
  using MsgMatch = matches_msg_ros::FeaturePoint;
  using MsgTracklet = matches_msg_ros::Tracklet;

  matches_msg_ros::MatchesMsg out;

  // assign matches with scaling
  out.tracks.reserve(tracklets_.size());

  // get maximum length of tracklet
  size_t max_length = 0;
  for (const auto& cur_tracklet : tracklets_) {
    MsgTracklet t;
    t.feature_points.reserve(cur_tracklet.size());

    // assign matches to tracklet msg, unscale if necessary
    for (const auto& cur_point : cur_tracklet) {
      MsgMatch msg_match;
      if (config_.scale_factor != 1.0) {
        msg_match.u = float(cur_point.p1_.u_) / config_.scale_factor;
        msg_match.v = float(cur_point.p1_.v_) / config_.scale_factor;
      } else {
        msg_match.u = float(cur_point.p1_.u_);
        msg_match.v = float(cur_point.p1_.v_);
      }
      t.feature_points.push_back(msg_match);
    }
    Assert(t.feature_points.size() == cur_tracklet.size(),
           "inconsistent tracklet size");

    // assign unique id from tracker_ to be able to recognize the tracklets in
    // different time steps
    t.id = cur_tracklet.id_;

    // assign tracklet to output matches_msg
    out.tracks.push_back(t);

    // get maximum lenth
    if (t.feature_points.size() > max_length)
      max_length = t.feature_points.size();
  }

  // assign as many timestamps as we have measurements
  out.stamps.reserve(max_length);
  // timestamps.back() is the most recent so insert from last max_length
  // elements
  out.stamps.insert(out.stamps.end(), timestamps_.begin(),
                    std::next(timestamps_.begin(), max_length));
  ROS_DEBUG_STREAM("timestamps length="
                   << timestamps_.size() << " max tracklet length=" << max_length
                   << " out.stamps length=" << out.stamps.size());

  //    Assert(std::next(out.stamps.rbegin()) == out.stamps.rend() ||
  //               (out.stamps.back() - *std::next(out.stamps.rbegin())).toSec()
  //               >= 0.,
  //           "timestamps in wrong order");

  if (out.stamps.size() > 1) {
    Assert((out.stamps.front() - *std::next(out.stamps.begin())).toSec() >= 0.,
           "timestamps in wrong order");

    ROS_DEBUG_STREAM("viso_tracking: front()="
                     << out.stamps.front().toSec()
                     << " next=" << std::next(out.stamps.begin())->toSec());
  }

  return out;
}

}  // namespace viso_feature_tracking