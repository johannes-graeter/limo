#include <boost/cstdint.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <vector>

#include <matches_msg_types/tracklets.hpp>
#include <matches_msg_types/tracklet.hpp>
#include <matches_msg_types/feature_point.hpp>

namespace p = boost::python;

BOOST_PYTHON_MODULE(matches_msg) {
    using namespace matches_msg_types;

    p::class_<FeaturePoint>("FeaturePoint", p::init<>())
	.def(p::init<float, float>())
	.def(p::init<float, float, float>())
	.def_readwrite("u", &FeaturePoint::u)
	.def_readwrite("v", &FeaturePoint::v)
	.def_readwrite("d", &FeaturePoint::d);

    p::class_<std::vector<FeaturePoint>>("_FeaturePointVec")
        .def(p::vector_indexing_suite<std::vector<FeaturePoint>>());

    p::class_<Tracklet>("Tracklet", p::init<>())
	.def_readwrite("feature_points", &Tracklet::feature_points)
	.def_readwrite("id", &Tracklet::id)
	.def_readwrite("age", &Tracklet::age)
	.def_readwrite("is_outlier", &Tracklet::is_outlier)
	.def_readwrite("label", &Tracklet::label);
    
    p::class_<std::vector<Tracklet>>("_TrackletVec")
        .def(p::vector_indexing_suite<std::vector<Tracklet>>());

    p::class_<std::vector<uint64_t>>("_TimestampVec")
        .def(p::vector_indexing_suite<std::vector<uint64_t>>());
    
    p::class_<Tracklets>("Tracklets", p::init<>())
	.def_readwrite("stamps", &Tracklets::stamps)
	.def_readwrite("tracks", &Tracklets::tracks);
}
