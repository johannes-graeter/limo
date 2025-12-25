#include "stereo_tracker.h"

namespace feature_tracking {
;

void StereoTracker::getTracklets(StereoTrackletList& tracklets, int minTrackLength) {
    if (minTrackLength > 0) {
        tracklets.clear();
        for (const StereoTracklet& thisTracklet : tracklets_) {
            if (thisTracklet.size() >= minTrackLength) {
                tracklets.push_back(thisTracklet);
            }
        }
    } else {
        tracklets.resize(tracklets_.size());
        std::copy(tracklets_.begin(), tracklets_.end(), tracklets.begin());
    }
}

void StereoTracker::getTracklets(StereoTrackletVector& tracklets, int minTrackLength) {
    if (minTrackLength > 0) {
        StereoTrackletList temp;
        this->getTracklets(temp, minTrackLength);
        tracklets.resize(temp.size());
        std::copy(temp.begin(), temp.end(), tracklets.begin());
    } else {
        // copy from internal list to vector
        tracklets.resize(tracklets_.size());
        std::copy(tracklets_.begin(), tracklets_.end(), tracklets.begin());
    }
}

StereoTrackletList& StereoTracker::getInternalTracklets() {
    return tracklets_;
}
}
