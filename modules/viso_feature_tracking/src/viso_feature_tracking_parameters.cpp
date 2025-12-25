/*
 * Copyright 2016. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#include "viso_feature_tracking/viso_feature_tracking_parameters.h"

namespace viso_feature_tracking {

// wrappers for reading nodes from yaml
template <typename T>
void get_config_value_optional(const YAML::Node& node, const std::string& child, T& value) {
    if (node[child].IsDefined()) {
        value = node[child].as<T>();
    }
}

template <typename T>
T get_config_value(const YAML::Node& node, const std::string& child) {
    if (node[child]) {
        return node[child].as<T>();
    } else {
        throw std::runtime_error(std::string("Parameter '") + child + std::string("' is not defined"));
    }
}

///@brief implementation of parameters manager for viso_feature_tracking
VisoFeatureTrackingParameters::VisoFeatureTrackingParameters(const bfs::path& file_name) {
    read_configuration(file_name);
    read_matcher_parameters(file_name);
}

void VisoFeatureTrackingParameters::read_configuration(const bfs::path& file_name) {
    try {
        YAML::Node root = YAML::LoadFile(file_name.string());
        // general configuration
        auto general_node = root["general"];
        if (general_node.IsDefined()) {
            // ROS configurations
            config.matches_msg_name = get_config_value<std::string>(general_node, "matches_msg_name");
            config.image_msg_name = get_config_value<std::string>(general_node, "image_msg_name");
            get_config_value_optional(general_node, "matches_msg_queue_size", config.matches_msg_queue_size);
            get_config_value_optional(general_node, "image_msg_queue_size", config.image_msg_queue_size);
            get_config_value_optional(general_node, "scale_factor", config.scale_factor);
        }
    } catch (YAML::ParserException& e) {
        throw std::runtime_error(std::string("Failed to parse YAML file: ") + e.what());
    }
}

void VisoFeatureTrackingParameters::read_matcher_parameters(const bfs::path& file_name) {
    try {
        YAML::Node root = YAML::LoadFile(file_name.string());
        // general configuration
        auto general_node = root["matcher"];
        if (general_node.IsDefined()) {
            // ROS configurations
            get_config_value_optional(general_node, "nms_n", params.nms_n);
            get_config_value_optional(general_node, "nms_tau", params.nms_tau);
            get_config_value_optional(general_node, "match_binsize", params.match_binsize);
            get_config_value_optional(general_node, "match_radius", params.match_radius);
            get_config_value_optional(general_node, "match_disp_tolerance", params.match_disp_tolerance);
            get_config_value_optional(general_node, "outlier_disp_tolerance", params.outlier_disp_tolerance);
            get_config_value_optional(general_node, "outlier_flow_tolerance", params.outlier_flow_tolerance);
            get_config_value_optional(general_node, "max_track_length", params.maxTracklength);
            get_config_value_optional(general_node, "blur_size", params.blur_size);
            get_config_value_optional(general_node, "blur_sigma", params.blur_sigma);
            get_config_value_optional(general_node, "multi_stage", params.multi_stage);
            if (params.multi_stage > 1) {
                throw std::runtime_error("in viso_feature_tracking_parameters: multi stage is 1 or 0 (boolean)");
            }
            get_config_value_optional(general_node, "half_resolution", params.half_resolution);
            if (params.half_resolution > 1) {
                throw std::runtime_error("in viso_feature_tracking_parameters: half resolution is 1 or 0 (boolean)");
            }
            get_config_value_optional(general_node, "method", params.method);
            if (params.method > 1) {
                throw std::runtime_error(
                    "in viso_feature_tracking_parameters: matching mehtod must be 0(flow) or 1(stereo)");
            }
            get_config_value_optional(general_node, "refinement", params.refinement);
            if (params.refinement > 2) {
                throw std::runtime_error("in viso_feature_tracking_parameters: refinement must be 0, 1 or 2");
            }
        }
    } catch (YAML::ParserException& e) {
        throw std::runtime_error(std::string("Failed to parse YAML file: ") + e.what());
    }
}
}
