#include "utilities.h"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

namespace feature_tracking {

namespace utils {

void estimateFundamentalMatrix(const StereoTrackletList& tracklets,
                               cv::Mat& F,
                               std::shared_ptr<std::vector<bool>> inliers,
                               double inlierThresh,
                               double prob) {
    // prepare point vectors
    std::vector<cv::Point2f> p1(tracklets.size()), p2(tracklets.size());
    // prepare inlier vector
    if (!inliers) {
        inliers.reset(new std::vector<bool>());
    }
    inliers->resize(tracklets.size(), false);
    // copy to opencv data type
    int index = 0;
    for (const StereoTracklet& tracklet : tracklets) {
        // check validity. should always be ok, but just to make sure
        if (tracklet.empty()) {
            ++index;
            continue;
        }
        // for sake of legibility
        cv::Point2f& p = p1[index];
        cv::Point2f& q = p2[index];
        const StereoMatch& match = tracklet[0];
        // parse point
        p.x = match.p1_.u_;
        p.y = match.p1_.v_;
        q.x = match.p2_.u_;
        q.y = match.p2_.v_;
        // this is to show that points are valid
        inliers->at(index) = true;
        // next
        ++index;
    }
    // compute fundamental matrix
    cv::Mat inlierMask;
    F = cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, inlierThresh, prob, inlierMask);
    // parse inliers into vector
    cv::MatIterator_<uchar> data = inlierMask.begin<uchar>();
    for (std::vector<bool>::reference inlier : *inliers) {
        inlier = bool(inlier) & bool(*data > 0);
        ++data;
    }
}

void estimateFundamentalMatrix(const StereoTrackletList& tracklets,
                               const std::pair<int, int> matchPairing,
                               cv::Mat& F,
                               std::shared_ptr<std::vector<bool>> inliers,
                               bool firstCam,
                               double inlierThresh,
                               double prob) {
    // prepare point vectors
    std::vector<cv::Point2f> p1(tracklets.size()), p2(tracklets.size());
    // prepare inlier vector
    if (!inliers) {
        inliers.reset(new std::vector<bool>());
    }
    inliers->resize(tracklets.size(), false);
    // copy to opencv data type
    int index = 0;
    for (const StereoTracklet& tracklet : tracklets) {
        // check validity. should always be ok, but just to make sure
        if (tracklet.empty() || tracklet.size() <= std::max(matchPairing.first, matchPairing.second)) {
            ++index;
            continue;
        }
        // for sake of legibility
        cv::Point2f& p = p1[index];
        cv::Point2f& q = p2[index];
        const StereoMatch& firstMatch = tracklet[matchPairing.first];
        const StereoMatch& secondMatch = tracklet[matchPairing.second];
        // parse point
        if (firstCam) {
            p.x = firstMatch.p1_.u_;
            p.y = firstMatch.p1_.v_;
            q.x = secondMatch.p1_.u_;
            q.y = secondMatch.p1_.v_;
        } else {
            p.x = firstMatch.p2_.u_;
            p.y = firstMatch.p2_.v_;
            q.x = secondMatch.p2_.u_;
            q.y = secondMatch.p2_.v_;
        }
        // this is to show that points are valid
        inliers->at(index) = true;
        // next
        ++index;
    }
    // compute fundamental matrix
    cv::Mat inlierMask;
    F = cv::findFundamentalMat(p1, p2, cv::FM_RANSAC, inlierThresh, prob, inlierMask);
    // parse inliers into vector
    cv::MatIterator_<uchar> data = inlierMask.begin<uchar>();
    for (std::vector<bool>::reference inlier : *inliers) {
        inlier = bool(inlier) & bool(*data > 0);
        ++data;
    }
}

void estimateEssentialMatrix(const StereoTrackletList& tracklets,
                             const cv::Mat& K1,
                             const cv::Mat& K2,
                             cv::Mat& E,
                             std::shared_ptr<std::vector<bool>> inliers,
                             double inlierThresh,
                             double prob) {
    // estimate fundamental matrix
    feature_tracking::utils::estimateFundamentalMatrix(tracklets, E, inliers, inlierThresh, prob);
    // compute essential from that
    E = K2.t() * E * K1;
}


void estimateEssentialMatrix(const StereoTrackletList& tracklets,
                             const std::pair<int, int> matchPairing,
                             const cv::Mat& K1,
                             const cv::Mat& K2,
                             cv::Mat& E,
                             std::shared_ptr<std::vector<bool>> inliers,
                             bool firstCam,
                             double inlierThresh,
                             double prob) {
    throw std::runtime_error("Not implemented yet");
}


void estimateRotTrans(const StereoTrackletList& tracklets,
                      const cv::Mat& K1,
                      const cv::Mat& K2,
                      cv::Mat& R,
                      cv::Mat& t,
                      std::shared_ptr<std::vector<bool>> inliers,
                      double inlierThresh,
                      double prob) {
    // reset inliers if not present, they are needed for the cheirality constraint later
    if (!inliers) {
        inliers.reset(new std::vector<bool>());
    }
    // get essential matrix
    cv::Mat E;
    feature_tracking::utils::estimateEssentialMatrix(tracklets, K1, K2, E, inliers, inlierThresh, prob);
    // get possible rotation/translation pairs
    cv::SVD svd(E);
    cv::Mat u = svd.u;
    cv::Mat vt = svd.vt;
    cv::Mat w = svd.w;

    double singular_values_ratio = std::abs(w.at<double>(0) / w.at<double>(1));
    // flip ratio to keep it [0,1]
    if (singular_values_ratio > 1.0) {
        singular_values_ratio = 1.0 / singular_values_ratio;
    }
    if (singular_values_ratio < 0.7) {
        // should be close to 1
        R = cv::Mat();
        t = cv::Mat();
        return;
    }

    // check Hatley & Zisserman for this
    cv::Mat W = cv::Mat::zeros(3, 3, CV_64F);
    W.at<double>(0, 1) = -1;
    W.at<double>(1, 0) = 1;
    W.at<double>(2, 2) = 1;
    cv::Mat Wt = W.t();
    cv::Mat R1 = u * W * vt;
    cv::Mat R2 = u * Wt * vt;
    cv::Mat t1 = u.col(2);
    cv::Mat t2 = -u.col(2);

    // check the four possible configurations
    std::vector<std::pair<cv::Mat, cv::Mat>> configs(4);
    configs[0].first = R1;
    configs[0].second = t1;
    configs[1].first = R1;
    configs[1].second = t2;
    configs[2].first = R2;
    configs[2].second = t1;
    configs[3].first = R2;
    configs[3].second = t2;

    // first projection matrix is the camera matrix
    cv::Mat P1 = cv::Mat::zeros(3, 4, CV_64FC1);
    K1.copyTo(cv::Mat(P1, cv::Rect(0, 0, 3, 3)));

    // second has to be determined from the four combinations
    std::pair<cv::Mat, cv::Mat>& bestConfig = configs[0];
    int bestSupportCount = 0;
    for (const std::pair<cv::Mat, cv::Mat>& thisConfig : configs) {
        std::vector<bool>::iterator valid = inliers->begin();
        int supportCount = 0;
        // make projection matrix from this config
        cv::Mat P2 = cv::Mat::zeros(3, 4, CV_64FC1);
        thisConfig.first.copyTo(cv::Mat(P2, cv::Rect(0, 0, 3, 3)));
        thisConfig.second.copyTo(cv::Mat(P2, cv::Rect(3, 0, 1, 3)));
        P2 = K2 * P2;

        // triangulate all tracklets
        for (const StereoTracklet& tracklet : tracklets) {
            if (!*valid) {
                ++valid;
                continue;
            }

            cv::Mat J(4, 4, CV_64FC1);
            for (int32_t j = 0; j < 4; j++) {
                J.at<double>(0, j) = P1.at<double>(2, j) * tracklet[0].p1_.u_ - P1.at<double>(0, j);
                J.at<double>(1, j) = P1.at<double>(2, j) * tracklet[0].p1_.v_ - P1.at<double>(1, j);
                J.at<double>(2, j) = P2.at<double>(2, j) * tracklet[0].p2_.u_ - P2.at<double>(0, j);
                J.at<double>(3, j) = P2.at<double>(2, j) * tracklet[0].p2_.v_ - P2.at<double>(0, j);
            }
            cv::SVD ch(J);
            cv::Mat v = ch.vt.t();
            cv::Mat x(v, cv::Rect(3, 0, 1, 4));

            // only if this point is in front of both cameras, this config is valid.
            // x = x/x.at<double>(3);
            cv::Mat p1 = P1 * x;
            cv::Mat p2 = P2 * x;
            if (p1.at<double>(2, 0) * x.at<double>(3, 0) > 0 && p2.at<double>(2, 0) * x.at<double>(3, 0) > 0) {
                ++supportCount;
            }
            ++valid;
        }
        if (supportCount > bestSupportCount) {
            bestConfig = thisConfig;
        }
    }

    R = bestConfig.first;
    t = bestConfig.second;
}


void estimateRotTrans(const StereoTrackletList& tracklets,
                      const std::pair<int, int> matchPairing,
                      const cv::Mat& K1,
                      const cv::Mat& K2,
                      cv::Mat& R,
                      cv::Mat& t,
                      std::shared_ptr<std::vector<bool>> inliers,
                      double inlierThresh,
                      double prob) {
    throw std::runtime_error("Not implemented yet");
}

void removeOutliers(const std::vector<bool>& inliers, StereoTrackletList& tracklets) {
    std::vector<bool>::const_iterator validity = inliers.begin();
    feature_tracking::StereoTrackletList::iterator trackletIt = tracklets.begin();
    while (trackletIt != tracklets.end()) {
        if (!*validity) {
            trackletIt = tracklets.erase(trackletIt);
        } else {
            ++trackletIt;
        }
        ++validity;
    }
}

void bucketing(const StereoTrackletList& inputTracklets,
               const int bucketW,
               const int bucketH,
               StereoTrackletList& outputTracklets) {

    outputTracklets.clear();

    // compute max x, y
    int maxU = 0;
    int maxV = 0;
    for (const StereoTracklet& tracklet : inputTracklets) {
        if (maxU < tracklet[0].p1_.u_) {
            maxU = tracklet[0].p1_.u_;
        }
        if (maxV < tracklet[0].p1_.v_) {
            maxV = tracklet[0].p1_.v_;
        }
    }

    // compute no of buckets in x and y direction
    int bucketCountW = maxU / bucketW + 1;
    int bucketCountH = maxV / bucketH + 1;

    // fill buckets
    std::vector<std::list<const StereoTracklet*>> buckets(bucketCountW * bucketCountH);
    for (const StereoTracklet& tracklet : inputTracklets) {
        int bucketX = std::min(std::max(0, int(tracklet[0].p1_.u_) / bucketW), bucketCountW);
        int bucketY = std::min(std::max(0, int(tracklet[0].p1_.v_) / bucketH), bucketCountH);

        buckets[bucketY * bucketCountW + bucketX].push_back(&tracklet);
    }

    // select oldest tracklet from bucket
    for (std::list<const StereoTracklet*>& bucket : buckets) {
        int maxAge = -1;
        const StereoTracklet* oldest = NULL;
        for (const StereoTracklet* tracklet : bucket) {
            if (maxAge < tracklet->age_) {
                maxAge = tracklet->age_;
                oldest = tracklet;
            }
        }
        if (oldest) {
            outputTracklets.push_back(*oldest);
        }
    }
}


} // namespace utils

} // namespace feature_tracking
