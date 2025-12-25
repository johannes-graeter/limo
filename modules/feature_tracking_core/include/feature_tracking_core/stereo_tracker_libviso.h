#pragma once

#include <memory>
#include "stereo_tracker.h"
#include "tracklet.h"

namespace viso2 {
class Matcher;
}

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
class StereoTrackerLibViso : public StereoTracker {
public:
    ///////////////////////////////////
    /// \brief The Parameters struct
    /// to set up parameters of the matcher
    /// and tracker
    struct Parameters {
        // from libviso >>
        int32_t nms_n;                  ///<  non-max-suppression: min. distance between maxima (in pixels)
        int32_t nms_tau;                ///<  non-max-suppression: interest point peakiness threshold
        int32_t match_binsize;          ///<  matching bin width/height (affects efficiency only)
        int32_t match_radius;           ///<  matching radius (du/dv in pixels)
        int32_t match_disp_tolerance;   ///<  dv tolerance for stereo matches (in pixels)
        int32_t outlier_disp_tolerance; ///<  outlier removal: disparity tolerance (in pixels)
        int32_t outlier_flow_tolerance; ///<  outlier removal: flow tolerance (in pixels)
        int32_t multi_stage;            ///<  0=disabled,1=multistage matching (denser and faster)
        int32_t half_resolution;        ///<  0=disabled,1=match at half resolution, refine at full resolution
        double f, cu, cv, base;
        // << end from lib viso

        int maxTracklength; ///< older detections of one feature than this age are discarded
        int method;         ///< matching method. Should be stereo or flow.

        ///////////////////////////////////
        /// \brief Parameters default constructor
        /// fills in default values.
        Parameters() {
            nms_n = 3;
            nms_tau = 50;
            match_binsize = 50;
            match_radius = 200;
            match_disp_tolerance = 2;
            outlier_disp_tolerance = 5;
            outlier_flow_tolerance = 5;
            multi_stage = 1;
            half_resolution = 1;
            maxTracklength = 3;
            method = 2;
        }
    };

    ///////////////////////////////////
    /// \brief StereoTrackerLibViso constructor
    /// takes parameters struct to set up
    /// internal matcher and tracking.
    ///
    /// \param params  Parameter struct
    ///
    StereoTrackerLibViso(Parameters params = Parameters());
    ///////////////////////////////////
    /// \brief ~StereoTrackerLibViso default
    /// destructor.
    ~StereoTrackerLibViso();

    ///////////////////////////////////
    /// \brief pushBack adds images to
    /// internal memory, performs matching
    /// and track association.
    ///
    /// \param im1  input image of camera 1
    /// \param im2  input image of camera 2
    ///
    virtual void pushBack(cv::Mat& im1, cv::Mat& im2);

protected:
    ///////////////////////////////////
    /// \brief handImageToMatcher performs
    /// matcher input update. Checks input format
    /// and reformats images if possible.
    ///
    /// \param im1  input image of camera 1
    /// \param im2  input image of camera 2
    ///
    void handImageToMatcher(cv::Mat& im1, cv::Mat& im2);
    ///////////////////////////////////
    /// \brief updateTracklets triggers matching
    /// and associates outcome to existing tracks.
    /// Also initializes tracks if no association
    /// is possible. Deletes tracks, if no match
    /// was associated in current frame.
    void updateTracklets();

    std::unique_ptr<viso2::Matcher> matcher_; ///< LibViso matcher used internally
    Parameters params_;                       ///< internal memory of parameters
};

} // namespace
