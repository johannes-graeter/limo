#pragma once

#include "tracklet.hpp"

namespace matches_msg_types{

using TimestampNSec=unsigned long; ///< Timestamp in unix system time

struct Tracklets {
   std::vector<TimestampNSec> stamps;
   std::vector<Tracklet> tracks;
};
}
