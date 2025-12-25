#pragma once

#include <stdint.h>
#include "tracklet.hpp"

namespace matches_msg_types {

using TimestampNSec = uint64_t; ///< Timestamp in unix system time

struct Tracklets {
    std::vector<TimestampNSec> stamps;
    std::vector<Tracklet> tracks;
};
}
