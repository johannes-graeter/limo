#include "stereo_tracker_libviso.h"

#include <opencv2/core/core.hpp>
#include <viso2/matcher.h>

namespace feature_tracking {

using namespace viso2;

StereoTrackerLibViso::StereoTrackerLibViso(Parameters params) {
    // copy params to internal memory
    params_ = params;
    // copy the params to configure matcher
    Matcher::parameters matcherParams;
    matcherParams.nms_n = params.nms_n;
    matcherParams.nms_tau = params.nms_tau;
    matcherParams.match_binsize = params.match_binsize;
    matcherParams.match_radius = params.match_radius;
    matcherParams.match_disp_tolerance = params.match_disp_tolerance;
    matcherParams.outlier_disp_tolerance = params.outlier_disp_tolerance;
    matcherParams.outlier_flow_tolerance = params.outlier_flow_tolerance;
    matcherParams.multi_stage = params.multi_stage;
    matcherParams.half_resolution = params.half_resolution;
    matcherParams.f = params.f;
    matcherParams.cu = params.cu;
    matcherParams.cv = params.cv;
    matcherParams.base = params.base;
    matcherParams.refinement = 0;
    // set matcher
    matcher_.reset(new Matcher(matcherParams));
}

StereoTrackerLibViso::~StereoTrackerLibViso() {
}


void StereoTrackerLibViso::pushBack(cv::Mat& im1, cv::Mat& im2) {
    // prepare the image for processing and hand to matchers
    this->handImageToMatcher(im1, im2);
    // get matches, add them to track
    this->updateTracklets();
}

void StereoTrackerLibViso::handImageToMatcher(cv::Mat& im1, cv::Mat& im2) {
    // check if 8 bit image
    if (im1.depth() != CV_8U || im2.depth() != CV_8U) {
        throw std::runtime_error("Unsupported image type");
    }
    // check channels, convert if necessary
    cv::Mat toProcessLeft;
    switch (im1.channels()) {
    case 1:
        toProcessLeft = im1;
        break;
    case 3:
        cv::cvtColor(im1, toProcessLeft, cv::COLOR_BGR2GRAY);
        break;
    default:
        throw std::runtime_error("Unsupported number of channels");
        break;
    }
    cv::Mat toProcessRight;
    switch (im2.channels()) {
    case 1:
        toProcessRight = im2;
        break;
    case 3:
        cv::cvtColor(im2, toProcessRight, cv::COLOR_BGR2GRAY);
        break;
    default:
        throw std::runtime_error("Unsupported number of channels");
        break;
    }
    // get image size information
    int32_t dims[3];
    dims[0] = toProcessLeft.cols;
    dims[1] = toProcessLeft.rows;
    dims[2] = toProcessLeft.step;
    // push back to matcher
    matcher_->pushBack(toProcessLeft.data, toProcessRight.data, dims, false);
}

void StereoTrackerLibViso::updateTracklets() {
    // compute feature matches
    matcher_->matchFeatures(params_.method);
    // get the matches from the matcher
    std::vector<Matcher::p_match> visoMatches = matcher_->getMatches();

    // sorting a list is faster than sorting a vector
    std::list<Matcher::p_match> listMatch;
    listMatch.insert(listMatch.end(), visoMatches.begin(), visoMatches.end());
    listMatch.sort([](const Matcher::p_match& l, const Matcher::p_match& r) { return l.i1p < r.i1p; });
    // make sure the current tracklets are sorted
    tracklets_.sort([](const StereoTracklet& l, const StereoTracklet& r) {
        return l.empty() || r.empty() || l.front().p1_.index_ < r.front().p1_.index_;
    });
    // now, go through vector of both current matches and tracklets and fuse them
    StereoTrackletList::iterator currentTracklet = tracklets_.begin();
    for (Matcher::p_match& match : listMatch) {
        // advance in tracklet vector until match found.
        while (currentTracklet != tracklets_.end() && currentTracklet->front().p1_.index_ < match.i1p) {
            // no valid match found
            currentTracklet = tracklets_.erase(currentTracklet);
        }
        // at this point, either the current tracklet has the same or a higher index
        if (currentTracklet != tracklets_.end() && currentTracklet->front().p1_.index_ == match.i1p) {
            // association successfull
            currentTracklet->push_front(StereoMatch(match.u1c, match.v1c, match.i1c, match.u2c, match.v2c, match.i2c));
            ++currentTracklet->age_;
            if (params_.maxTracklength > 0 && currentTracklet->size() > params_.maxTracklength) {
                // limit track length
                currentTracklet->pop_back();
            }
            // next tracklet
        } else {
            // new observation
            currentTracklet = tracklets_.insert(currentTracklet, StereoTracklet());
            currentTracklet->push_front(StereoMatch(match.u1p, match.v1p, match.i1p, match.u2p, match.v2p, match.i2p));
            currentTracklet->push_front(StereoMatch(match.u1c, match.v1c, match.i1c, match.u2c, match.v2c, match.i2c));
        }
        ++currentTracklet;
    }
    // there might still be unassociated tracklets after the last associated
    if (currentTracklet != tracklets_.end()) {
        tracklets_.erase(currentTracklet, tracklets_.end());
    }
}

} // namespace
