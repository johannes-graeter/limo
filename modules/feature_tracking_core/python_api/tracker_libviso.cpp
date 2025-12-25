#include <boost/cstdint.hpp>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>

// #define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <fstream>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>
#include <deque>

#include <opencv2/opencv.hpp>
#include <feature_tracking_core/tracklet.h>
#include <feature_tracking_core/tracker_libviso.h>

using namespace feature_tracking;

namespace p = boost::python;
namespace np = p::numpy;

///@brief Wrap the push back call of tracker.
///       It requires that the passed NumPy array be exactly what we're 
///       looking for - no conversion from nested sequences or arrays with 
///       other data types, because we want to modify it in-place. 
///       Modified example from boost_1_63_0/libs/python/example/numpy/wrap.cpp
void wrapPushBack(TrackerLibViso& obj, np::ndarray const& array)
{
    if (array.get_dtype() != np::dtype::get_builtin<uint8_t>())
    {
        PyErr_SetString(PyExc_TypeError, "Incorrect array data type");
        p::throw_error_already_set();
    }
    if (array.get_nd() > 3 || array.get_nd() < 2)
    {
        PyErr_SetString(PyExc_TypeError, "Incorrect number of dimensions");
        p::throw_error_already_set();
    }
    int32_t height = array.shape(0);
    int32_t width = array.shape(1);
    int32_t num_channels = array.shape(2);
    cv::Mat img(height, width, CV_8UC(num_channels), reinterpret_cast<uint8_t *>(array.get_data()));
    obj.pushBack(img);
}

void wrapPushBackMask(TrackerLibViso &obj, np::ndarray const& img_array, np::ndarray const& mask_array)
{
    if (img_array.get_dtype() != np::dtype::get_builtin<uint8_t>() || mask_array.get_dtype() != np::dtype::get_builtin<uint8_t>())
    {
        PyErr_SetString(PyExc_TypeError, "Incorrect array data type");
        p::throw_error_already_set();
    }
    if (img_array.get_nd() > 3 || img_array.get_nd() < 2)
    {
        PyErr_SetString(PyExc_TypeError, "Incorrect number of image dimensions");
        p::throw_error_already_set();
    }
    if (mask_array.get_nd() == 1)
    {
        PyErr_SetString(PyExc_TypeError, "Incorrect number of mask dimensions");
        p::throw_error_already_set();
    }

    int32_t height = img_array.shape(0);
    int32_t width = img_array.shape(1);
    int32_t num_channels = img_array.shape(2);

    if (height != mask_array.shape(0) || width != mask_array.shape(1))
    {
        PyErr_SetString(PyExc_TypeError, "image and mask array have incosistent height or width.");
        p::throw_error_already_set();
    }

    cv::Mat img(height, width, CV_8UC(num_channels), reinterpret_cast<uint8_t *>(img_array.get_data()));
    cv::Mat mask(height, width, CV_8UC1, reinterpret_cast<uint8_t *>(mask_array.get_data()));
    obj.pushBack(img, mask);
}

std::vector<Tracklet> wrapGetTracklets(TrackerLibViso& obj)
{
     std::vector<Tracklet> tracklets;
     obj.getTracklets(tracklets, 0);
     return tracklets;
}

BOOST_PYTHON_MODULE(tracker_libviso)
{
    np::initialize(); // have to put this in any module that uses Boost.NumPy

    p::object parameters_class = p::class_<TrackerLibViso::Parameters>("Parameters")
        .def_readwrite("nms_n", &TrackerLibViso::Parameters::nms_n)
        .def_readwrite("nms_tau", &TrackerLibViso::Parameters::nms_tau)
        .def_readwrite("match_binsize", &TrackerLibViso::Parameters::match_binsize)
        .def_readwrite("match_radius", &TrackerLibViso::Parameters::match_radius)
        .def_readwrite("match_disp_tolerance", &TrackerLibViso::Parameters::match_disp_tolerance)
        .def_readwrite("outlier_disp_tolerance", &TrackerLibViso::Parameters::outlier_disp_tolerance)
        .def_readwrite("outlier_flow_tolerance", &TrackerLibViso::Parameters::outlier_flow_tolerance)
        .def_readwrite("multi_stage", &TrackerLibViso::Parameters::multi_stage)
        .def_readwrite("half_resolution", &TrackerLibViso::Parameters::half_resolution)
        .def_readwrite("refinement", &TrackerLibViso::Parameters::refinement)
        .def_readwrite("max_track_length", &TrackerLibViso::Parameters::maxTracklength)
        .def_readwrite("method", &TrackerLibViso::Parameters::method);
    p::object default_parameters = parameters_class();

    p::class_<ImagePoint>("ImagePoint", p::init<>())
	.def(p::init<float, float>())
	.def(p::init<float, float, int>())
	.def_readwrite("index_", &ImagePoint::index_)
	.def_readwrite("u_", &ImagePoint::u_)
	.def_readwrite("v_", &ImagePoint::v_);

    p::class_<Match>("Match", p::init<>())
        .def(p::init<ImagePoint>())
	.def(p::init<float, float>())
	.def(p::init<float, float, int>())
	.def_readwrite("p1_", &Match::p1_);
    
    using Matches = std::deque<Match>;
    p::class_<Matches>("_Matches").def(p::vector_indexing_suite<Matches>());

    p::class_<Tracklet, p::bases<Matches>>("Tracklet", p::init<>())
        .def_readwrite("id_", &Tracklet::id_)
        .def_readwrite("age_", &Tracklet::age_);

    using Tracklets = std::vector<Tracklet>;
    p::class_<Tracklets>("Tracklets").def(p::vector_indexing_suite<Tracklets>());

    // Choose one of the overloaded functions and cast to function pointer.
    void (TrackerLibViso::*getTracklets1)(Tracklets&, int) = &TrackerLibViso::getTracklets;
    p::class_<TrackerLibViso, boost::noncopyable>("TrackerLibViso", p::init<>())
	.def(p::init<TrackerLibViso::Parameters>())
        .def("get_tracklets", getTracklets1);

    p::def("push_back", &wrapPushBack);
    p::def("push_back_mask", &wrapPushBackMask);
    p::def("get_tracklets", &wrapGetTracklets);
}
