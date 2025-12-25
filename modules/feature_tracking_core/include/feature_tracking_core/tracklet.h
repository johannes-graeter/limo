#pragma once

#include <deque>
#include <list>
#include <memory>
#include <vector>

namespace feature_tracking {


///////////////////////////////////
/// \brief The WorldPoint struct
/// stores 3D world point information
/// for matches
struct WorldPoint {
    double data[3];

    double& operator[](const int i) {
        return data[i];
    }

    WorldPoint() {
        data[0] = 0.0;
        data[1] = 0.0;
        data[2] = 0.0;
    }
};


///////////////////////////////////
/// \brief The ImagePoint struct
/// stores 2D image coordinates for
/// matches. Also supplies and index
/// for feature identification.
struct ImagePoint {
    ///////////////////////////////////
    /// \brief ImagePoint constructor
    /// with value initilization from parameters.
    ///
    /// \param u      u value. default: -1
    /// \param v      v value. default: -1
    /// \param index  index. default: -1
    ///
    ImagePoint(float u = -1, float v = -1, int index = -1);

    ///////////////////////////////////
    /// \brief Equality operator
    /// Needed for example in python binding
    ///
    /// \param ImagePoint    Point to compare to
    bool operator ==(const ImagePoint& other) const;

    int index_; ///< index for feature identification
    float u_;   ///< u coordinate
    float v_;   ///< v coordinate

    /// \brief Threshold for equality in image
    /// coordinates.
    static constexpr float equality_threshold_{0.1};
};


///////////////////////////////////
/// \brief The Match class
/// defines an image point in one image
/// with optional corresponding world point.
class Match {
public:
    ///////////////////////////////////
    /// \brief Match initializes the match
    /// from an observed image point
    ///
    /// \param p 2d image point
    ///
    Match(ImagePoint p);
    ///////////////////////////////////
    /// \brief Match intilizes the match from a
    /// set of u,v-image coordinates and index
    ///
    /// \param u    u-value. default: -1
    /// \param v    v-value. default: -1
    /// \param index  index. default: -1
    ///
    Match(float u = -1, float v = -1, int index = -1);

    ///////////////////////////////////
    /// \brief Equality operator
    /// Needed for example in python binding
    ///
    /// \param Match    Match to compare to
    bool operator ==(const Match& other) const;

    ImagePoint p1_;                 ///< 2d point in image
    std::shared_ptr<WorldPoint> x_; ///< optional 3 world point
};

///////////////////////////////////
/// \brief The StereoMatch class
/// defines a stereo match as two image points
/// and an optional world point. It is derived
/// from the Match class and just adds the second point
class StereoMatch : public Match {
public:
    ///////////////////////////////////
    /// \brief StereoMatch constructor
    /// from two observed image points
    ///
    /// \param p1   2d image point as observed from first camera
    /// \param p2   2d image point as observed from second camera
    ///
    StereoMatch(ImagePoint p1, ImagePoint p2);

    ///////////////////////////////////
    /// \brief StereoMatch constructor
    /// from u,v-coordinates and indices of
    /// points observed from first and second camers
    ///
    /// \param u1       u-value observed in first camera. default: -1
    /// \param v1       v-value observed in first camera. default: -1
    /// \param index1   index of observation in first camera. default: -1
    /// \param u2       u-value observed in second camera. default: -1
    /// \param v2       v-value observed in second camera. default: -1
    /// \param index2   index of observation in second camera. default: -1
    ///
    StereoMatch(float u1 = -1, float v1 = -1, int index1 = -1, float u2 = -1, float v2 = -1, int index2 = -1);

    ImagePoint p2_; ///< the image point observed from the second camera
};

///////////////////////////////////
/// \brief The Tracklet class
/// derived from deque, its meant to store a track of mutliple
/// matches together with an ID for the track.
class Tracklet : public std::deque<Match> {
public:
    ///////////////////////////////////
    /// \brief Tracklet
    /// default constructor will also set the ID.
    ///
    /// The ID is taken from the static nextId field. It then is
    /// incremented by one.
    Tracklet();

    u_int64_t id_; ///< track ID for feature identification
    int age_;      ///< age of tracklet in frames
private:
    static u_int64_t nextId; ///< stores the next ID to be given to the next tacklet.
};

///////////////////////////////////
/// \brief The StereoTracklet class
/// derived from deque, its meant to store a track of mutliple
/// stereo matches together with an ID for the track.
class StereoTracklet : public std::deque<StereoMatch> {
public:
    ///////////////////////////////////
    /// \brief StereoTracklet
    /// default constructor will also set the ID.
    ///
    /// The ID is taken from the static nextId field. It then is
    /// incremented by one.
    StereoTracklet();

    u_int64_t id_; ///< track ID for feature identification
    int age_;      ///< age of tracklet in frames
private:
    static u_int64_t nextId; ///< stores the next ID to be given to the next tacklet.
};

///////////////////////////////////
/// for easier track maintenance,
/// standard STL-containers for tracks
/// are abbreviated here.
///
using TrackletList = std::list<Tracklet>;
using StereoTrackletList = std::list<StereoTracklet>;
using TrackletVector = std::vector<Tracklet>;
using StereoTrackletVector = std::vector<StereoTracklet>;

} // namespace
