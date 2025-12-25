#pragma once

#include <memory>
#include "tracklet.h"

namespace cv {
class Mat;
}

namespace feature_tracking {

///////////////////////////////////
/// \brief The StereoTrackerLibViso class
/// alows feature tracking of LibViso
/// stereo feature dectection.
///
/// The structure is essentially that of
/// the LibViso matcher. Images are pushed
/// into internal memory. For those, features are
/// computed. Macthing between features of subsequent
/// frame pairs happens upon push back of a second image pair.
/// Tracklets can be obtained by using the getTracklet
/// method.
class StereoTracker {
public:
    ///////////////////////////////////
    /// \brief pushBack adds images to
    /// internal memory, performs matching
    /// and track association.
    ///
    /// \param im1  input image of camera 1
    /// \param im2  input image of camera 2
    ///
    virtual void pushBack(cv::Mat& im1, cv::Mat& im2) = 0;
    ///////////////////////////////////
    /// \brief getTracklets copies internal
    /// tracklet memory to output list
    ///
    /// \param tracklets reference to output list of tracklets
    ///
    virtual void getTracklets(StereoTrackletList& tracklets, int minTrackLength = 0);
    ///////////////////////////////////
    /// \brief getTracklets copies internal
    /// tracklet memory to output vector
    ///
    /// \param tracklets reference to output vector of tracklets
    ///
    virtual void getTracklets(StereoTrackletVector& tracklets, int minTrackLength = 0);
    ///////////////////////////////////
    /// \brief getInternalTracklets returns
    /// reference to internal tracklet memory.
    /// Can be used, e.g. if you'd like to delete
    /// outliers for tracking.
    ///
    /// \return Reference to internal tracklet memory
    ///
    virtual StereoTrackletList& getInternalTracklets();

protected:
    StereoTrackletList tracklets_; ///< internal memory of tracklets
};


using StereoTrackerPtr = std::shared_ptr<StereoTracker>;

} // namespace
