#include "feature_tracking_core/tracker_libviso.h"

#include <opencv2/core/core.hpp>
#include <viso2/matcher.h>

namespace feature_tracking {

using namespace viso2;

void TrackerLibViso::setParameters(const Parameters& params) {
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
    /* matcherParams.f  = params.f;
     matcherParams.cu = params.cu;
     matcherParams.cv  = params.cv;
     matcherParams.base  = params.base;*/
    matcherParams.refinement = params.refinement;
    // set matcher parameters without resettin matcher
    // Could be that this makes problems if you alter multi_stage and have resolution after having matched stuff.
    if (!matcher_) {
        matcher_.reset(new Matcher(matcherParams));
    } else {
        matcher_->param = matcherParams;
    }
}

TrackerLibViso::Parameters TrackerLibViso::getParameters() const {
    return params_;
}

TrackerLibViso::TrackerLibViso(Parameters params) : params_(params) {
    setParameters(params);
}

TrackerLibViso::~TrackerLibViso() {
}

void TrackerLibViso::pushBack(cv::Mat& im) {
    this->pushBack(im, cv::Mat(0, 0, CV_8UC1));
}

void TrackerLibViso::pushBack(cv::Mat& im, const cv::Mat& mask) {
    // prepare the image for processing and hand to matchers
    this->handImageToMatcher(im, mask);
    // get matches, add them to track
    this->updateTracklets();
}

void TrackerLibViso::handImageToMatcher(cv::Mat& im, const cv::Mat& mask) {
    // check if 8 bit image
    if (im.depth() != CV_8U) {
        throw std::runtime_error("Unsupported image type");
    }

    // check if 8 bit image
    if (mask.depth() != CV_8UC1) {
        throw std::runtime_error("Unsupported mask type");
    }

    // check channels, convert if necessary
    cv::Mat toProcess;
    switch (im.channels()) {
    case 1:
        toProcess = im;
        break;
    case 3:
        cv::cvtColor(im, toProcess, cv::COLOR_BGR2GRAY);
        break;
    default:
        throw std::runtime_error("Unsupported number of channels");
        break;
    }
    // get image size information
    int32_t dims[3];
    dims[0] = toProcess.cols;
    dims[1] = toProcess.rows;
    dims[2] = toProcess.step;
    // push back to matcher
    matcher_->pushBack(toProcess.data, dims, false, mask);
}

void TrackerLibViso::updateTracklets() {
    // compute feature matches
    matcher_->matchFeatures(params_.method);
    // get the matches from the matcher
    std::vector<Matcher::p_match> visoMatches = matcher_->getMatches();
    // sorting a list is faster than sorting a vector
    std::list<Matcher::p_match> listMatch;
    listMatch.insert(listMatch.end(), visoMatches.begin(), visoMatches.end());
    listMatch.sort([](const Matcher::p_match& l, const Matcher::p_match& r) { return l.i1p < r.i1p; });
    // make sure the current tracklets are sorted
    tracklets_.sort([](const Tracklet& l, const Tracklet& r) {
        return l.empty() || r.empty() || l.front().p1_.index_ < r.front().p1_.index_;
    });
    printf("Update tracklet\n");
    // now, go through vector of both current matches and tracklets and fuse them
    TrackletList::iterator currentTracklet = tracklets_.begin();
    for (Matcher::p_match& match : listMatch) {
        // advance in tracklet vector until match found.
        while (currentTracklet != tracklets_.end() && currentTracklet->front().p1_.index_ < match.i1p) {
            // no valid match found
            currentTracklet = tracklets_.erase(currentTracklet);
        }
        // at this point, either the current tracklet has the same or a higher index
        if (currentTracklet != tracklets_.end() && currentTracklet->front().p1_.index_ == match.i1p) {
            // association successfull
            currentTracklet->push_front(Match(match.u1c, match.v1c, match.i1c));
            ++currentTracklet->age_;
            if (params_.maxTracklength > 0 && currentTracklet->size() > static_cast<size_t>(params_.maxTracklength)) {
                // limit track length
                currentTracklet->pop_back();
            }
            // next tracklet
        } else {
            // new observation
            currentTracklet = tracklets_.insert(currentTracklet, Tracklet());
            currentTracklet->push_front(Match(match.u1p, match.v1p, match.i1p));
            currentTracklet->push_front(Match(match.u1c, match.v1c, match.i1c));
        }
        ++currentTracklet;
    }
    // there might still be unassociated tracklets after the last associated
    if (currentTracklet != tracklets_.end()) {
        tracklets_.erase(currentTracklet, tracklets_.end());
    }
}

void TrackerLibViso::getTracklets(TrackletList& tracklets, int minTrackLength) {
    if (minTrackLength > 0) {
        tracklets.clear();
        for (const Tracklet& thisTracklet : tracklets_) {
            if (thisTracklet.size() >= static_cast<size_t>(minTrackLength)) {
                tracklets.push_back(thisTracklet);
            }
        }
    } else {
        tracklets.resize(tracklets_.size());
        std::copy(tracklets_.begin(), tracklets_.end(), tracklets.begin());
    }
}

void TrackerLibViso::getTracklets(TrackletVector& tracklets, int minTrackLength) {
    if (minTrackLength > 0) {
        TrackletList temp;
        this->getTracklets(temp, minTrackLength);
        tracklets.resize(temp.size());
        std::copy(temp.begin(), temp.end(), tracklets.begin());
    } else {
        // copy from internal list to vector
        tracklets.resize(tracklets_.size());
        std::copy(tracklets_.begin(), tracklets_.end(), tracklets.begin());
    }
}

TrackletList& TrackerLibViso::getInternalTracklets() {
    return tracklets_;
}

} // namespace
