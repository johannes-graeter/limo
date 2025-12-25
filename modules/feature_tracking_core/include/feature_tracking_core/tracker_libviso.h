#pragma once

#include <memory>
#include "tracklet.h"


///////////////////////////////////
/// Forward declarations of cv::Mat
/// and the LibViso matcher
///
namespace viso2 {
class Matcher;
}
namespace cv {
class Mat;
}

namespace feature_tracking {

///////////////////////////////////
/// \brief The TrackerLibViso class
/// alows feature tracking of LibViso
/// mono feature dectection.
///
/// The structure is essentially that of
/// the LibViso matcher. An image is pushed
/// into internal memory. For that, features are
/// computed. Macthing between features of subsequent
/// frames happens upon push back of a second image.
/// Tracklets can be obtained by using the getTracklet
/// method.
class TrackerLibViso {
public:
    ///////////////////////////////////
    /// \brief The Parameters struct
    /// to set up parameters of the matcher
    /// and tracker
    struct Parameters {
        // from libviso >>
        int32_t nms_n;                  ///< non-max-suppression: min. distance between maxima (in pixels)
        int32_t nms_tau;                ///< non-max-suppression: interest point peakiness threshold
        int32_t match_binsize;          ///< matching bin width/height (affects efficiency only)
        int32_t match_radius;           ///< matching radius (du/dv in pixels)
        int32_t match_disp_tolerance;   ///< dv tolerance for stereo matches (in pixels)
        int32_t outlier_disp_tolerance; ///< outlier removal: disparity tolerance (in pixels)
        int32_t outlier_flow_tolerance; ///< outlier removal: flow tolerance (in pixels)
        int32_t multi_stage;            ///< 0=disabled,1=multistage matching (denser and faster)
        int32_t half_resolution;        ///< 0=disabled,1=match at half resolution, refine at full resolution
        int32_t refinement;             ///< 0=disabled,1=do refinement with parabolic fitting
        // double  f,cu,cv,base;        /// only needed for stereo tracking
        // << end from lib viso

        int maxTracklength; ///< older detections of one feature than this age are discarded
        int method;         ///< matching method. Should be stereo(1) or flow(0).

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
            method = 0;
            refinement = 1;
        }
    };

    ///////////////////////////////////
    /// \brief TrackerLibViso constructor
    /// takes parameters struct to set up
    /// internal matcher and tracking.
    ///
    /// \param params  Parameter struct
    ///
    TrackerLibViso(Parameters params = Parameters());
    ///////////////////////////////////
    /// \brief ~TrackerViso default
    /// destructor.
    ~TrackerLibViso();

    ///////////////////////////////////
    /// \brief pushBack adds image to
    /// internal memory, performs matching
    /// and track association.
    ///
    /// \param im   input image
    /// \param mask mask for matching, if pixel value is 0, keypoints will not be matched
    ///
    void pushBack(cv::Mat& im, const cv::Mat& mask);

    ///////////////////////////////////
    /// \brief pushBack, overload for simple interface
    /// \param im   input image
    ///
    void pushBack(cv::Mat& im);

    ///////////////////////////////////
    /// \brief getTracklets copies internal
    /// tracklet memory to output list
    ///
    /// \param tracklets reference to output list of tracklets
    ///
    void getTracklets(TrackletList& tracklets, int minTrackLength = 0);
    ///////////////////////////////////
    /// \brief getTracklets copies internal
    /// tracklet memory to output vector
    ///
    /// \param tracklets reference to output vector of tracklets
    ///
    void getTracklets(TrackletVector& tracklets, int minTrackLength = 0);
    ///////////////////////////////////
    /// \brief getInternalTracklets returns
    /// reference to internal tracklet memory.
    /// Can be used, e.g. if you'd like to delete
    /// outliers for tracking.
    ///
    /// \return Reference to internal tracklet memory
    ///
    TrackletList& getInternalTracklets();

    ///////////////////////////////////
    /// \brief setParameters set
    /// tracker parameters
    void setParameters(const Parameters& params);
    Parameters getParameters() const;

protected:
    ///////////////////////////////////
    /// \brief handImageToMatcher performs
    /// matcher input update. Checks input format
    /// and reformats image if possible.
    ///
    /// \param im   input image.
    /// \param mask mask for matching, if pixel value is 0, keypoints will not be matched
    ///
    void handImageToMatcher(cv::Mat& im, const cv::Mat& mask);
    ///////////////////////////////////
    /// \brief updateTracklets triggers matching
    /// and associates outcome to existing tracks.
    /// Also initializes tracks if no association
    /// is possible. Deletes tracks, if no match
    /// was associated in current frame.
    void updateTracklets();

    std::unique_ptr<viso2::Matcher> matcher_; ///< LibViso matcher used internally
    TrackletList tracklets_;                  ///< internal memory of tracklets
    Parameters params_;                       ///< internal memory of parameters
    std::vector<cv::Mat> images_;
    int img_idx_ = 0;
};

} // namespace
