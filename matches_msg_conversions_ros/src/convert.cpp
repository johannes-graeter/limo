/*
 * Copyright 2017. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#include "convert.hpp"

namespace matches_msg_conversions_ros {

void Assert(bool condition, std::string s) {
    if (!condition) {
        throw std::runtime_error("in matches_msg_conversions_ros: " + s);
    }
}
} // end of ns
