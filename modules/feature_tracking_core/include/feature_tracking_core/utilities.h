#pragma once

#include "tracklet.h"

#include <memory>

#include <opencv2/core/core.hpp>

namespace feature_tracking {

namespace utils {

void estimateFundamentalMatrix(const StereoTrackletList& tracklets,
                               cv::Mat& F,
                               std::shared_ptr<std::vector<bool>> inliers = nullptr,
                               double inlierThresh = 3.,
                               double prob = 0.9);

void estimateFundamentalMatrix(const StereoTrackletList& tracklets,
                               const std::pair<int, int> matchPairing,
                               cv::Mat& F,
                               std::shared_ptr<std::vector<bool>> inliers = nullptr,
                               bool firstCam = true,
                               double inlierThresh = 3.,
                               double prob = 0.9);

void estimateEssentialMatrix(const StereoTrackletList& tracklets,
                             const cv::Mat& K1,
                             const cv::Mat& K2,
                             cv::Mat& E,
                             std::shared_ptr<std::vector<bool>> inliers = nullptr,
                             double inlierThresh = 3.,
                             double prob = 0.9);

void estimateEssentialMatrix(const StereoTrackletList& tracklets,
                             const std::pair<int, int> matchPairing,
                             const cv::Mat& K1,
                             const cv::Mat& K2,
                             cv::Mat& E,
                             std::shared_ptr<std::vector<bool>> inliers = nullptr,
                             bool firstCam = true,
                             double inlierThresh = 3.,
                             double prob = 0.9);

void estimateRotTrans(const StereoTrackletList& tracklets,
                      const cv::Mat& K1,
                      const cv::Mat& K2,
                      cv::Mat& R,
                      cv::Mat& t,
                      std::shared_ptr<std::vector<bool>> inliers = nullptr,
                      double inlierThresh = 3.,
                      double prob = 0.9);

void estimateRotTrans(const StereoTrackletList& tracklets,
                      const std::pair<int, int> matchPairing,
                      const cv::Mat& K1,
                      const cv::Mat& K2,
                      cv::Mat& R,
                      cv::Mat& t,
                      std::shared_ptr<std::vector<bool>> inliers = nullptr,
                      double inlierThresh = 3.,
                      double prob = 0.9);


void removeOutliers(const std::vector<bool>& inliers, StereoTrackletList& tracklets);

void bucketing(const StereoTrackletList& inputTracklets,
               const int bucketW,
               const int bucketH,
               StereoTrackletList& outputTracklets);

} // namespace utils

} // namespace feature_tracking
