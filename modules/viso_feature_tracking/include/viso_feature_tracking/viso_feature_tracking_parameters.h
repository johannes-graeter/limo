/*
 * Copyright 2016. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#pragma once

#include <feature_tracking_core/tracker_libviso.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <string>

namespace bfs = boost::filesystem;

namespace viso_feature_tracking {
struct Parameters : public feature_tracking::TrackerLibViso::Parameters {
  Parameters() : feature_tracking::TrackerLibViso::Parameters() {
    blur_size = 3;
    blur_sigma = 0.8f;
  }

  ///@brief size of kernel for gaussian blur, must be impair
  int blur_size;
  ///@brief standard deviation of gaussian bluring
  float blur_sigma;
};

struct Configuration {
  Configuration()
      : matches_msg_name("/viso_topic"),
        image_msg_name("/image_topic"),
        matches_msg_queue_size(10),
        image_msg_queue_size(10),
        scale_factor(1.0) {}

  // ROS settings
  std::string matches_msg_name, image_msg_name;
  int matches_msg_queue_size, image_msg_queue_size;
  double scale_factor;
};

/**@class VisoFeatureTrackingParameters
 *   @brief class for reading parameters from yaml for package
 * viso_feature_tracking
 */
class VisoFeatureTrackingParameters {
 public:
  VisoFeatureTrackingParameters(const bfs::path& fileName);

  const Configuration& get_config() const { return config; }

  const Parameters& get_matcher_parameters() const { return params; }

 private:
  void read_configuration(const bfs::path& file_name);

  void read_matcher_parameters(const bfs::path& file_name);

  Configuration config;
  Parameters params;
};
}  // namespace viso_feature_tracking
