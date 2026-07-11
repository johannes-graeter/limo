/*
 * DepthEstimator.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: wilczynski
 */

#include <exception>
#include <limits>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "HistogramPointDepth.h"
#include "Logger.h"
#include "NeighborFinderPixel.h"
#include "PCA.h"
//#include "NeighborFinderKdd.h"
//#include "Converter.h"
#include "DepthEstimator.h"

#include "LinePlaneIntersectionNormal.h"
#include "LinePlaneIntersectionOrthogonalTreshold.h"
#include "RoadDepthEstimatorLeastSquares.h"
#include "RoadDepthEstimatorMEstimator.h"
#include "RoadDepthEstimatorMaxSpanningTriangle.h"

#include <chrono>

namespace Mono_Lidar {

bool DepthEstimator::Initialize(const std::shared_ptr<CameraPinhole>& camera,
                                const Eigen::Affine3d& transform_lidar_to_cam) {
    if (!_isInitializedConfig) {
        throw "Call 'InitConfig' before calling 'Initialize'.";
    }

    _camera = camera;
    _camera->getImageSize(_imgWitdh, _imgHeight);
    _transform_lidar_to_cam = transform_lidar_to_cam;
    _transform_cam_to_lidar = transform_lidar_to_cam.inverse();

    // Initialize nearest neighbor search module
    if (_parameters->neighbor_search_mode == 0) {
        // neighbor pixel search
        this->_neighborFinder = std::make_shared<NeighborFinderPixel>(
            _imgWitdh, _imgHeight, _parameters->pixelarea_search_witdh, _parameters->pixelarea_search_height);
        this->_neighborFinder->Initialize(_parameters);
    } else if (_parameters->neighbor_search_mode == 1) {
        //		// neighbor kdd search
        //		this->_neighborFinder = std::make_shared<NeighborFinderKdd>();
        //		this->_neighborFinder->Initialize(_parameters);
    } else
        throw "neighbor_search_mode has the invalid value: " + std::to_string(_parameters->neighbor_search_mode);

    // Initialize treshold depth module
    if (_parameters->treshold_depth_enabled) {
        auto mode = (eTresholdDepthMode)_parameters->treshold_depth_mode;
        this->_tresholdDepthGlobal = std::make_shared<TresholdDepthGlobal>(
            mode, _parameters->treshold_depth_min, _parameters->treshold_depth_max);
    } else
        this->_tresholdDepthGlobal = NULL;

    // Initialize local treshold depth module
    if (_parameters->treshold_depth_local_enabled) {
        auto mode = (eTresholdDepthMode)_parameters->treshold_depth_local_mode;
        auto toleranceType = (eTresholdToleranceType)_parameters->treshold_depth_local_valuetype;
        this->_tresholdDepthLocal =
            std::make_shared<TresholdDepthLocal>(mode, toleranceType, _parameters->treshold_depth_local_value);
    } else
        this->_tresholdDepthLocal = NULL;

    // Initialize line plane intersection module
    if (_parameters->viewray_plane_orthoganality_treshold > 0)
        this->_linePlaneIntersection = std::make_shared<LinePlaneIntersectionOrthogonalTreshold>(
            _parameters->viewray_plane_orthoganality_treshold);
    else
        this->_linePlaneIntersection = std::make_shared<LinePlaneIntersectionNormal>();

    // Initialize Ransac plane for road estimation
    if (_parameters->do_use_ransac_plane) {
        // Initialize Depth Estimator for points which lie on the ground plane/road
        if (_parameters->plane_estimator_use_triangle_maximation)
            this->_roadDepthEstimator =
                std::make_shared<RoadDepthEstimatorMaxSpanningTriangle>(_parameters->plane_estimator_z_x_min_relation);
        else if (_parameters->plane_estimator_use_leastsquares)
            this->_roadDepthEstimator = std::make_shared<RoadDepthEstimatorLeastSquares>();
        else if (_parameters->plane_estimator_use_mestimator)
            this->_roadDepthEstimator = std::make_shared<RoadDepthEstimatorMEstimator>();
        else
            throw "No road depth estimator selected.";

        if (this->_tresholdDepthGlobal != NULL)
            this->_roadDepthEstimator->EnableTresholdDepthGlobal(this->_tresholdDepthGlobal);

        if (this->_tresholdDepthLocal != NULL)
            this->_roadDepthEstimator->EnableTresholdDepthLocal(this->_tresholdDepthLocal);
    } else {
        this->_roadDepthEstimator = NULL;
    }

    // Module for further depth segmentation
    if (_parameters->do_use_depth_segmentation) {
        _lidarRowSegmenter = std::make_shared<HelperLidarRowSegmentation>();
    }

    // Module for constructing a maximum size plane with 3 points with a given pointcloud
    if (_parameters->do_use_triangle_size_maximation)
        _planeCalcMaxSpanning =
            std::make_shared<PlaneEstimationCalcMaxSpanningTriangle>(_parameters->do_publish_points);
    else
        _planeCalcMaxSpanning = NULL;

    // Plane planarity checker if the plane is constructed by using exactly 3 points
    if (_parameters->do_check_triangleplanar_condition) {
        _checkPlanarTriangle =
            std::make_shared<PlaneEstimationCheckPlanar>(_parameters->triangleplanar_crossnorm_treshold);
    } else
        this->_checkPlanarTriangle = NULL;

    _isInitialized = true;

    return true;
}

bool DepthEstimator::InitConfig(const std::string& filePath, const bool printparams) {
    _parameters = std::make_shared<DepthEstimatorParameters>();
    _parameters->fromFile(filePath);

    if (printparams)
        _parameters->print();

    _isInitializedConfig = true;

    return true;
}


bool DepthEstimator::InitConfig(std::shared_ptr<DepthEstimatorParameters> parameters, const bool printparams) {
    if (!parameters)
        _parameters = std::make_shared<DepthEstimatorParameters>();
    else 
        _parameters = parameters;
        
    if (printparams)
        _parameters->print();

    _isInitializedConfig = true;

    return true;
}

PointcloudData Transform_Cloud_LidarToCamera(const DepthEstimator::Cloud::ConstPtr& cloud_lidar_cs,
                                             const Eigen::Affine3d& lidar_to_cam,
                                             std::shared_ptr<CameraPinhole> camera,
                                             std::shared_ptr<HelperLidarRowSegmentation> lidarRowSegmenter) {

    Logger::Instance().Log(Logger::MethodStart, "DepthEstimator::Transform_Cloud_LidarToCamera");

    using namespace std;

    PointcloudData points; // stored all points of the pointcloud in

    // Convert cloud to Eigen-Format
    // Eigen::MatrixXf pointcloud_eigen{cloud_lidar_cs->getMatrixXfMap()};
    points._points_cs_lidar = cloud_lidar_cs->getMatrixXfMap().cast<double>().topRows<3>();
    int pointCount = points._points_cs_lidar.cols();

    // Transform cloud into camera frame
    points._points_cs_camera = lidar_to_cam * points._points_cs_lidar;

    // Project cloud from camera cs into image cs
    points._points_cs_image.resize(2, pointCount);

    points._pointsInImgRange = camera->getImagePoints(points._points_cs_camera, points._points_cs_image);

    int imgWitdh, imgHeight;
    camera->getImageSize(imgWitdh, imgHeight);
    int pointCountImgVisible = 0;

    for (int i = 0; i < points._points_cs_image.cols(); i++) {
        if (points._pointsInImgRange[i]) {
            if ((points._points_cs_image(0, i) > 0) && (points._points_cs_image(0, i) < imgWitdh) &&
                (points._points_cs_image(1, i) > 0) && (points._points_cs_image(1, i) < imgHeight))
                pointCountImgVisible++;
        }
    }
    // create data which just stores visible points (in image cs)
    points._points_cs_image_visible.resize(2, pointCountImgVisible);

    int visibleIndex = 0;
    points._pointIndex.clear();

    for (int i = 0; i < pointCount; i++) {
        if (points._pointsInImgRange[i]) {
            if ((points._points_cs_image(0, i) > 0) && (points._points_cs_image(0, i) < imgWitdh) &&
                (points._points_cs_image(1, i) > 0) && (points._points_cs_image(1, i) < imgHeight)) {
                points._points_cs_image_visible(0, visibleIndex) = points._points_cs_image(0, i);
                points._points_cs_image_visible(1, visibleIndex) = points._points_cs_image(1, i);
                points._pointIndex.push_back(i);
                visibleIndex++;
            }
        }
    }

    if (lidarRowSegmenter != NULL) {
        lidarRowSegmenter->SegmentPoints(points._points_cs_image_visible);
    }

    // Debug info
    Logger::Instance().Log(Logger::MethodEnd, "DepthEstimator::Transform_Cloud_LidarToCamera");

    return points;
}


void DepthEstimator::setInputCloud(const Cloud::ConstPtr& cloud, GroundPlane::Ptr& groundPlane) {
    using namespace std;
    auto start_time_stuff = std::chrono::steady_clock::now();
    Logger::Instance().Log(Logger::MethodStart, "DepthEstimator::setInputCloud");

    // Precheck
    if (!_isInitialized)
        throw "call of 'setInputCloud' without 'initialize'";
    _isInitializedPointCloud = true;

    _points_interpolated.clear();
    _points_interpolated_plane.clear();
    _points_triangle_corners.clear();
    _points_neighbors.clear();
    _points_groundplane.clear();

    //    auto start_time_cut = std::chrono::steady_clock::now();
    //    // Extract required part of the original pointcloud
    //    Cloud::Ptr cloud_cut = {std::make_shared<Cloud>()};
    //    CutPointCloud(cloud, cloud_cut);

    //    points_cut = Transform_Cloud_LidarToCamera(cloud_cut, _transform_lidar_to_cam, _camera, _lidarRowSegmenter);
    //    std::cout << "Duration monolidar:set_input_cloud:cut="
    //              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
    //                                                                       start_time_cut)
    //                     .count()
    //              << " ms" << std::endl;


    auto start_time_transf = std::chrono::steady_clock::now();
    // Change coordinate frame
    _points = Transform_Cloud_LidarToCamera(cloud, _transform_lidar_to_cam, _camera, _lidarRowSegmenter);
    //    std::cout << "Duration monolidar:set_input_cloud:trasnform="
    //              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
    //                                                                       start_time_transf)
    //                     .count()
    //              << " ms" << std::endl;

    if (_parameters->neighbor_search_mode == 0) {
        // use next pixel neighbor search
        // Transform the visible lidar points if neighbor search mode is pixel based
        auto neighborFinder = std::dynamic_pointer_cast<NeighborFinderPixel>(_neighborFinder);

        neighborFinder->InitializeLidarProjection(
            this->_points._points_cs_image_visible, this->_points._points_cs_camera, this->_points._pointIndex);
    } else if (_parameters->neighbor_search_mode == 1) {
        // use kdd tree
        //		auto neighborFinder = std::dynamic_pointer_cast<NeighborFinderKdd>(_neighborFinder);
        //		neighborFinder->InitKdTree(this->_points._points_cs_image_visible);
    } else {
        throw "neighbor_search_mode has the invalid value: " + std::to_string(_parameters->neighbor_search_mode);
    }

    // Do Ransac plane estimation
    if (_parameters->do_use_ransac_plane) {
        if (groundPlane == nullptr) {
            // Initialize ground plane estimator with RANSAC scheme
            groundPlane = std::make_shared<RansacPlane>(_parameters);
        }

        // only segment if this wasn't done yet
        if (!groundPlane->isSegmented()) {
            groundPlane->CalculateInliersPlane(cloud, _parameters->ransac_plane_min_z, _parameters->ransac_plane_max_z);
        }

        // set initial plane for m_estimator (weighted least squares)
        if (_parameters->plane_estimator_use_mestimator) {
            auto depthEstimatorWeighted = std::dynamic_pointer_cast<RoadDepthEstimatorMEstimator>(_roadDepthEstimator);
            const auto planeCoeffs = groundPlane->getModelCoeffs();
            Eigen::Vector3d planeNormal(planeCoeffs[0], planeCoeffs[1], planeCoeffs[2]);
            auto planeEigen = Eigen::Hyperplane<double, 3>(planeNormal.normalized(), planeCoeffs[3]);
            depthEstimatorWeighted->setPlanePrior(planeEigen);
        }

        auto inliers = groundPlane->getInlinersIndex();
        for (const auto& inlier : inliers) {
            // This is bug, inlier indices belong to point cloud cut and _points to cloud.
            Eigen::Vector3d point(_points._points_cs_camera(0, inlier),
                                  _points._points_cs_camera(1, inlier),
                                  _points._points_cs_camera(2, inlier));

            if (_parameters->ransac_plane_use_camx_treshold) {
                double treshold = _parameters->ransac_plane_treshold_camx;

                if (fabs(point.x()) <= treshold)
                    _points_groundplane.push_back(point);
            } else
                _points_groundplane.push_back(point);
        }
    }

    Logger::Instance().Log(Logger::MethodEnd, "DepthEstimator::setInputCloud");
}

void DepthEstimator::getCloudCameraCs(Cloud::Ptr& pointCloud_cam_cs) {
    int count = _points._points_cs_camera.cols();
    pointCloud_cam_cs->clear();

    for (int i = 0; i < count; i++) {
        double x = _points._points_cs_camera(0, i);
        double y = _points._points_cs_camera(1, i);
        double z = _points._points_cs_camera(2, i);
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = 1;

        pointCloud_cam_cs->points.push_back(point);
    }

    pointCloud_cam_cs->width = (uint32_t)pointCloud_cam_cs->points.size();
    pointCloud_cam_cs->height = 1;
    pointCloud_cam_cs->is_dense = false;
}

void DepthEstimator::getCloudNeighbors(Cloud::Ptr& pointCloud_neighbors) {
    FillCloud(_points_neighbors, pointCloud_neighbors);
}

void DepthEstimator::getCloudInterpolated(Cloud::Ptr& pointCloud_interpolated) {
    FillCloud(_points_interpolated, pointCloud_interpolated);
}

void DepthEstimator::getCloudInterpolatedPlane(Cloud::Ptr& pointCloud_interpolated_plane) {
    FillCloud(_points_interpolated, pointCloud_interpolated_plane);
}

void DepthEstimator::getCloudTriangleCorners(Cloud::Ptr& pointCloud_triangle_corner) {
    FillCloud(_points_triangle_corners, pointCloud_triangle_corner);
}

void DepthEstimator::FillCloud(const VecOfVec3d content, const Cloud::Ptr& cloud) {
    int count = content.size();
    cloud->clear();

    for (int i = 0; i < count; i++) {
        double x = content[i].x();
        double y = content[i].y();
        double z = content[i].z();
        pcl::PointXYZI point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = 1;

        cloud->points.push_back(point);
    }

    cloud->width = (uint32_t)cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
}

void DepthEstimator::CutPointCloud(const Cloud::ConstPtr& cloud_in, const Cloud::Ptr& cloud_out) {
    cloud_out->header.frame_id = cloud_in->header.frame_id;
    cloud_out->header.stamp = cloud_in->header.stamp;

    // Do filtering that was done in Ransac plane here, since passes seem to be very slow!
    for (const auto& pt : cloud_in->points) {
        if (_parameters->ransac_plane_min_z < pt.z && pt.z < _parameters->ransac_plane_max_z) {
            pcl::PointXYZI newPoint;
            newPoint.x = pt.x;
            newPoint.y = pt.y;
            newPoint.z = pt.z;
            newPoint.intensity = pt.intensity;

            cloud_out->points.push_back(newPoint);
        }
    }
}

void DepthEstimator::getPointsCloudImageCs(Eigen::Matrix2Xd& visiblePointsImageCs) {
    visiblePointsImageCs = this->_points._points_cs_image_visible;
}

void DepthEstimator::getCloudRansacPlane(Cloud::Ptr& pointCloud_plane_ransac) {
    FillCloud(_points_groundplane, pointCloud_plane_ransac);
}

const DepthCalculationStatistics& DepthEstimator::getDepthCalcStats() {
    return this->_depthCalcStats;
}

void DepthEstimator::CalculateDepth(const Cloud::ConstPtr& pointCloud,
                                    const Eigen::Matrix2Xd& points_image_cs,
                                    Eigen::VectorXd& points_depths,
                                    GroundPlane::Ptr& ransacPlane) {
    setInputCloud(pointCloud, ransacPlane);
    CalculateDepth(points_image_cs, points_depths, ransacPlane);
}

void DepthEstimator::CalculateDepth(const Cloud::ConstPtr& pointCloud,
                                    const Eigen::Matrix2Xd& points_image_cs,
                                    Eigen::VectorXd& points_depths,
                                    Eigen::VectorXi& resultType,
                                    GroundPlane::Ptr& ransacPlane) {

    setInputCloud(pointCloud, ransacPlane);
    CalculateDepth(points_image_cs, points_depths, resultType, ransacPlane);
}

void DepthEstimator::CalculateDepth(const Eigen::Matrix2Xd& featurePoints_image_cs,
                                    Eigen::VectorXd& points_depths,
                                    const GroundPlane::Ptr& ransacPlane) {
    Eigen::VectorXi depthTypes(featurePoints_image_cs.cols());
    CalculateDepth(featurePoints_image_cs, points_depths, depthTypes, ransacPlane);
}

void DepthEstimator::CalculateDepth(const Eigen::Matrix2Xd& featurePoints_image_cs,
                                    Eigen::VectorXd& points_depths,
                                    Eigen::VectorXi& resultType,
                                    const GroundPlane::Ptr& ransacPlane) {
    Mono_Lidar::Logger::Instance().Log(Mono_Lidar::Logger::MethodStart, "DepthEstimator::CalculateDepth Start: ");

    using namespace std;

    // Precheck
    if (!_isInitializedPointCloud) {
        throw "call of 'CalculateDepth' without 'SetInputCloud'";
    }
    int imgPointCount = featurePoints_image_cs.cols();
    points_depths.resize(imgPointCount);
    resultType.resize(imgPointCount);

    if (_parameters->do_depth_calc_statistics)
        _depthCalcStats.Clear();

    if (_parameters->set_all_depths_to_zero) {
        resultType.setConstant(1);
        points_depths.setConstant(-1);

        return;
    }

#pragma omp parallel for
    for (int i = 0; i < imgPointCount; i++) {
        double imgPointX = featurePoints_image_cs(0, i);
        double imgPointY = featurePoints_image_cs(1, i);
        Eigen::Vector2d imgPoint(imgPointX, imgPointY);

        std::shared_ptr<DepthCalcStatsSinglePoint> stats = NULL;

        if (_parameters->do_debug_singleFeatures)
            stats = std::make_shared<DepthCalcStatsSinglePoint>();

        auto depthPair = this->CalculateDepth(imgPoint, ransacPlane, stats);
        points_depths(i) = depthPair.second;
        resultType(i) = depthPair.first;

        //        #pragma omp critical
        //        {
        //            LogDepthCalcStats(depthPair.first);

        //            if (_parameters->do_debug_singleFeatures)
        //            {
        //                stats->_calcResult = depthPair.first;
        //                this->_depthCalcStats.getPointStats().push_back(stats);
        //            }
        //        }
    }

    if (_parameters->do_depth_calc_statistics) {
        int featuresCount = featurePoints_image_cs.cols();
        _depthCalcStats.SetPointCount(featuresCount);
    }

    Mono_Lidar::Logger::Instance().Log(Mono_Lidar::Logger::MethodEnd, "DepthEstimator::CalculateDepth.");
}


std::pair<DepthResultType, double> DepthEstimator::CalculateDepth(
    const Eigen::Vector2d& featurePoint_image_cs,
    const GroundPlane::Ptr& ransacPlane,
    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats) {
    using namespace std;

    // Debug log feature
    if (calcStats != nullptr) {
        calcStats->_featureX = featurePoint_image_cs.x();
        calcStats->_featureY = featurePoint_image_cs.y();
    }

    // Get the neighbor pixels around the given feature point
    std::vector<int> neighborIndicesCut;
    VecOfVec3d neighbors;
    auto result = std::make_pair<DepthResultType, double>(DepthResultType::Unspecified, -1);

    // Get the neighbors around the feature point (on the image plane) as 3D lidar points
    if (!this->CalculateNeighbors(featurePoint_image_cs, neighborIndicesCut, neighbors, calcStats))
        return std::pair<DepthResultType, double>(DepthResultType::RadiusSearchInsufficientPoints, -1);

    // Calculate points using region growing if enabled
    if (_lidarRowSegmenter != nullptr) {
        std::cout << "---------------------------> run region growing" << std::endl;
        // Segment the neighbors due to depth using a histogram
        VecOfVec3d neighborsSegmented;
        std::vector<int> neighborsSegmentedIndex;

        Eigen::Vector3d nearestPoint;
        int nearestPointIndex;

        if (!this->CalculateNearestPoint(neighbors, neighborIndicesCut, nearestPoint, nearestPointIndex))
            return std::pair<DepthResultType, double>(DepthResultType::HistogramNoLocalMax, -1);

        // check if in range
        if (nearestPoint.z() > _parameters->treshold_depth_max)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthGlobalGreaterMax, -1);

        neighborsSegmented.push_back(nearestPoint);
        neighborsSegmentedIndex.push_back(nearestPointIndex);

        // Use region growing for further depth segmentation if activated
        int resultRegionGrowing =
            CalcDepthSegmentionRegionGrowing(featurePoint_image_cs, neighborsSegmented, neighborsSegmentedIndex);

        // if result is < 0 then region growing failed
        if (resultRegionGrowing <= 0) {
            switch (resultRegionGrowing) {
            case -1:
                result =
                    std::make_pair<DepthResultType, double>(DepthResultType::RegionGrowingNearestSeedNotAvailable, -1);
                break;
            case -2:
                result = std::make_pair<DepthResultType, double>(DepthResultType::RegionGrowingSeedsOutOfRange, -1);
                break;
            case -3:
                result = std::make_pair<DepthResultType, double>(DepthResultType::RegionGrowingInsufficientPoints, -1);
                break;
            }
        } else {
            auto result = this->CalculateDepthSegmented(featurePoint_image_cs, neighborsSegmented, calcStats, false);

            if (result.first == DepthResultType::Success) {
                result.first = DepthResultType::SuccessRegionGrowing;
                return result;
            }
        }
    }

    // Segment the neighbors due to depth using a histogram
    VecOfVec3d neighborsSegmented;
    std::vector<int> neighborsSegmentedIndex;

    if (!this->CalculateDepthSegmentation(
            neighbors, neighborIndicesCut, neighborsSegmented, neighborsSegmentedIndex, calcStats)) {
        result = std::pair<DepthResultType, double>(DepthResultType::HistogramNoLocalMax, -1);
    }

    // Calculate depth
    bool checkPlanar = _checkPlanarTriangle != NULL;
    if (result.first != DepthResultType::HistogramNoLocalMax) {
        result = this->CalculateDepthSegmented(featurePoint_image_cs, neighborsSegmented, calcStats, checkPlanar);

        if (result.first == DepthResultType::Success)
            return result;
    }

    // Special treatment for road features
    DepthResultType resultOld = result.first;
    if ((ransacPlane != nullptr) && (this->_roadDepthEstimator != NULL)) {
        // calculate neighbors in a wider area for the road
        neighborIndicesCut.clear();
        neighbors.clear();

        if (!this->CalculateNeighbors(featurePoint_image_cs, neighborIndicesCut, neighbors, calcStats, 2.0, 1.5))
            return std::pair<DepthResultType, double>(DepthResultType::RadiusSearchInsufficientPoints, -1);

        // get the neighbor points which are inliers of the plane estimation
        if (!this->CalculateDepthSegmentationPlane(
                neighbors, neighborIndicesCut, neighborsSegmented, calcStats, ransacPlane))
            return std::pair<DepthResultType, double>(resultOld, -1);

        // calculate the depth with points which lay on the ground plane
        Eigen::Vector3d intersectionPoint;
        result = this->_roadDepthEstimator->CalculateDepth(
            featurePoint_image_cs, _camera, neighborsSegmented, intersectionPoint);
    }

    return result;
}

int DepthEstimator::CalcDepthSegmentionRegionGrowing(const Eigen::Vector2d& featurePoint_image_cs,
                                                     VecOfVec3d& neighborsSegmented,
                                                     std::vector<int>& imgNeighborsSegmentedIndex) {
    if (_lidarRowSegmenter == NULL)
        return 0;

    throw std::runtime_error("DepthEstimator: Region growing not supported!");

    // Find neighbors from the depth segmented points
    int result = _lidarRowSegmenter->calculateNeighborPoints(
        _points,
        featurePoint_image_cs,
        _parameters->depth_segmentation_max_treshold_gradient,
        _parameters->depth_segmentation_max_seedpoint_to_seedpoint_distance,
        _parameters->depth_segmentation_max_seedpoint_to_seedpoint_distance_gradient,
        _parameters->depth_segmentation_max_neighbor_to_seedpoint_distance,
        _parameters->depth_segmentation_max_neighbor_to_seedpoint_distance_gradient,
        _parameters->depth_segmentation_max_neighbor_distance,
        _parameters->depth_segmentation_max_neighbor_distance_gradient,
        _parameters->depth_segmentation_max_pointcount,
        imgNeighborsSegmentedIndex);

    neighborsSegmented.clear();

    if (result != 1)
        return result;

    for (auto index : imgNeighborsSegmentedIndex) {
        neighborsSegmented.push_back(_points.get3DPointFromCut(index));
    }

    return 1;
}

bool DepthEstimator::CalculateNeighbors(const Eigen::Vector2d& featurePoint_image_cs,
                                        std::vector<int>& neighborIndicesCut,
                                        VecOfVec3d& neighbors,
                                        std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
                                        const float scaleX,
                                        const float scaleY) {
    using namespace std;

    // get the indices of the neighbors in the image
    this->_neighborFinder->getNeighbors(featurePoint_image_cs,
                                        this->_points._points_cs_camera,
                                        this->_points._pointIndex,
                                        neighborIndicesCut,
                                        calcStats,
                                        scaleX,
                                        scaleY);

    // get the 3D neighbor points from the pointcloud using the given indices
    this->_neighborFinder->getNeighbors(
        this->_points._points_cs_camera, this->_points._pointIndex, neighborIndicesCut, neighbors);

    // Debug
    //    #pragma omp critical
    //    {
    //        if (_parameters->do_publish_points)
    //        {
    //            for (uint i = 0; i < neighbors.size(); i++)
    //                this->_points_neighbors.push_back(neighbors[i]);
    //        }

    //        // Debug log neighbors
    //        if (calcStats != NULL)
    //        {
    //            for (const auto& point3d : neighbors)
    //            {
    //                calcStats->_neighbors3d.push_back(std::tuple<float, float, float>(point3d.x(), point3d.y(),
    //                point3d.z()));
    //                Eigen::Vector2d point2D;
    //                _camera->getImagePoint(point3d, point2D);
    //                calcStats->_neighbors2d.push_back(std::pair<float, float>(point2D.x(), point2D.y()));
    //            }
    //        }
    //    }

    if (neighbors.size() < (uint)_parameters->radiusSearch_count_min)
        return false;

    return true;
}


bool DepthEstimator::CalculateNearestPoint(const VecOfVec3d& neighbors,
                                           const std::vector<int>& neighborsIndexCut,
                                           Eigen::Vector3d& nearestPoint,
                                           int& nearestPointIndex) {
    // Create depth vector of the 3d lidar points
    Eigen::VectorXd points3dDepth;
    points3dDepth.resize(neighbors.size());

    for (uint i = 0; i < neighbors.size(); i++) {
        double distance = neighbors[i].z();
        points3dDepth[i] = distance;
    }

    // bool pointFound =PointHistogram::GetNearestPoint(neighbors, neighborsIndexCut, points3dDepth,
    //		pointsSegmented, pointsSegmentedIndex);

    float minDepth = std::numeric_limits<double>::max();
    int minIndex = -1;
    int depthCount = points3dDepth.rows();

    for (int i = 0; i < depthCount; i++) {
        if (points3dDepth(i) < minDepth) {
            minDepth = points3dDepth(i);
            minIndex = i;
        }
    }

    // check if no point available
    if (minIndex == -1) {
        return false;
    }

    // nearest point has been found
    nearestPoint = neighbors.at(minIndex);
    nearestPointIndex = neighborsIndexCut.at(minIndex);

    return true;
}

bool DepthEstimator::CalculateDepthSegmentation(const VecOfVec3d& neighbors,
                                                const std::vector<int>& neighborsIndexCut,
                                                VecOfVec3d& pointsSegmented,
                                                std::vector<int>& pointsSegmentedIndex,
                                                std::shared_ptr<DepthCalcStatsSinglePoint> calcStats) {
    using namespace std;

    pointsSegmented.clear();
    pointsSegmentedIndex.clear();

    if (_parameters->do_use_histogram_segmentation) {
        // Create depth vector of the 3d lidar points
        Eigen::VectorXd points3dDepth;
        points3dDepth.resize(neighbors.size());

        for (uint i = 0; i < neighbors.size(); i++) {
            double distance = neighbors[i].z();
            points3dDepth[i] = std::min(distance, 999.);
        }

        double lowBorder;
        double highBorder;

        bool localMaxFound = PointHistogram::FilterPointsMinDistBlob(neighbors,
                                                                     neighborsIndexCut,
                                                                     points3dDepth,
                                                                     _parameters->histogram_segmentation_bin_witdh,
                                                                     _parameters->histogram_segmentation_min_pointcount,
                                                                     pointsSegmented,
                                                                     pointsSegmentedIndex,
                                                                     lowBorder,
                                                                     highBorder,
                                                                     calcStats);

        if (!localMaxFound)
            return false;
    } else {
        pointsSegmented = neighbors;
    }

    // Debug log segmented points
    //	if (calcStats != NULL)
    //	{
    //	    for (const auto& point3d : pointsSegmented)
    //	    {
    //	    	calcStats->_pointsSegmented3d.push_back(std::tuple<float, float, float>(point3d.x(), point3d.y(),
    // point3d.z()));
    //	    	Eigen::Vector2d point2D;
    //	    	_camera->getImagePoint(point3d, point2D);
    //	    	calcStats->_pointsSegmented2d.push_back(std::pair<float, float>(point2D.x(), point2D.y()));
    //	    }
    //	}

    return true;
}

bool DepthEstimator::CalculateDepthSegmentationPlane(const VecOfVec3d& neighbors,
                                                     const std::vector<int> neighborIndices,
                                                     VecOfVec3d& pointsSegmented,
                                                     std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
                                                     GroundPlane::Ptr ransacPlane) {
    // check precondition
    if (neighbors.size() != neighborIndices.size()) {
        throw("DepthEstimator::CalculateDepthSegmentationPlane: neighbors.size() (" + std::to_string(neighbors.size()) +
              ") != neighborIndices.size() (" + std::to_string(neighborIndices.size()) + ")");
    }

    pointsSegmented.clear();

    if (ransacPlane == nullptr) {
        std::cout << "Ransac plane is nullptr" << std::endl;
        throw std::runtime_error("Ransac plane is nullptr");
    }
    const auto& planeCoeffs = ransacPlane->getModelCoeffs();

    double treshold = _parameters->ransac_plane_point_distance_treshold;

    for (size_t i = 0; i < neighbors.size(); i++) {
        int index = neighborIndices[i];

        // Map the index from the visible pointcloud to the original lidar cloud
        // The indizes of the ransac plane are based on the original cloud
        int inidexRaw = _points._pointIndex[index];

        Eigen::Vector3d point_lidar_cs = _transform_cam_to_lidar * neighbors[i];
        pcl::PointXYZ point(point_lidar_cs.x(), point_lidar_cs.y(), point_lidar_cs.z());
        double distance = pcl::pointToPlaneDistance(point, planeCoeffs);

        if (distance > treshold)
            return false;

        if (ransacPlane->CheckPointInPlane(inidexRaw)) {
            // use treshold
            pointsSegmented.push_back(neighbors[i]);

#pragma omp critical
            { _points_triangle_corners.push_back(neighbors[i]); }
        }
    }

    // Check if enough points are in the set to span a plane
    if (pointsSegmented.size() < 3) {
        return false;
    }

    // Check if the z size is bigger than the x size
    double minX = std::numeric_limits<int>::max();
    double maxX = std::numeric_limits<int>::min();
    double minZ = std::numeric_limits<int>::max();
    double maxZ = std::numeric_limits<int>::min();

    for (const auto& pt : pointsSegmented) {
        if (pt.x() < minX)
            minX = pt.x();
        else if (pt.x() > maxX)
            maxX = pt.x();

        if (pt.z() < minZ)
            minZ = pt.z();
        else if (pt.z() > maxZ)
            maxZ = pt.z();
    }

    double deltaX = maxX - minX;
    double deltaZ = maxZ - minZ;

    if (deltaX >= deltaZ) {
        // return false;
    }

    // Points segmented
    Eigen::Vector3d pos1 = Eigen::Vector3d::Zero();

    for (const auto& pt : pointsSegmented)
        pos1 += pt;
    pos1 /= pointsSegmented.size();

    // neighbors points
    Eigen::Vector3d pos2 = Eigen::Vector3d::Zero();

    for (const auto& pt : neighbors) {
        pos2 += pt;
    }
    pos2 /= neighbors.size();

    //	Logger::Instance().PrintEigenVector(pos1, "Mean segmented");
    //	Logger::Instance().PrintEigenVector(pos2, "Mean all");
    //	std::cout << "Sizes: " << pointsSegmented.size() << "/" << neighbors.size() << std::endl;

    // calculate if the distance of the mean point is under a treshold


    //	Eigen::Vector3d point_lidar_cs = _transform_cam_to_lidar * pos1;
    //	pcl::PointXYZ point(point_lidar_cs.x(), point_lidar_cs.y(), point_lidar_cs.z());
    //	double distance = pcl::pointToPlaneDistance(point, planeCoeffs);
    //
    //	if (distance > treshold)
    //	{
    //		return false;
    //	}

    //	for (const auto& pt : neighbors)
    //	{
    //		Eigen::Vector3d point_lidar_cs = _transform_cam_to_lidar * pos2;
    //		pcl::PointXYZ point(point_lidar_cs.x(), point_lidar_cs.y(), point_lidar_cs.z());
    //		double distance = pcl::pointToPlaneDistance(point, planeCoeffs);
    //
    //		if (distance > treshold)
    //		{
    //			return false;
    //		}
    //	}

    return true;
}


std::pair<DepthResultType, double> DepthEstimator::CalculateDepthSegmented(
    const Eigen::Vector2d& point_image_cs,
    const VecOfVec3d& pointsSegmented,
    std::shared_ptr<DepthCalcStatsSinglePoint> calcStats,
    const bool checkPlanarTriangle) {
    using namespace std;

    // Get the spanning triangle from lidar points
    Eigen::Vector3d corner1;
    Eigen::Vector3d corner2;
    Eigen::Vector3d corner3;

    if (!_parameters->do_use_PCA && (this->_planeCalcMaxSpanning != nullptr)) {
        if (!this->_planeCalcMaxSpanning->CalculatePlaneCorners(
                pointsSegmented, corner1, corner2, corner3, _points_triangle_corners))
            return std::pair<DepthResultType, double>(DepthResultType::TriangleNotPlanarInsufficientPoints, -1);
    } else {
        if (pointsSegmented.size() < 3)
            return std::pair<DepthResultType, double>(DepthResultType::HistogramNoLocalMax, -1);

        corner1 = pointsSegmented[0];
        corner2 = pointsSegmented[1];
        corner3 = pointsSegmented[2];
    }

    if (!_parameters->do_use_PCA && (_checkPlanarTriangle != nullptr)) {
        if (!_checkPlanarTriangle->CheckPlanar(corner1, corner2, corner3))
            return std::pair<DepthResultType, double>(DepthResultType::TriangleNotPlanar, -1);
    }

    // get the depth of the feature point
    Eigen::Vector3d viewingRaySupportPoint;
    Eigen::Vector3d viewingRayDirection;
    _camera->getViewingRays(point_image_cs, viewingRaySupportPoint, viewingRayDirection);

    if (viewingRayDirection.z() < 0)
        viewingRayDirection *= -1;

    Eigen::Vector3d intersectionPoint;
    double depth;

    if (_parameters->do_use_PCA) {
        Eigen::MatrixXd pointCloud;
        pointCloud.resize(3, pointsSegmented.size());

        for (size_t i = 0; i < pointsSegmented.size(); i++) {
            pointCloud(0, i) = pointsSegmented[i].x();
            pointCloud(1, i) = pointsSegmented[i].y();
            pointCloud(2, i) = pointsSegmented[i].z();
        }

        // analyse the ointcloud using pca
        auto pca = Mono_LidarPipeline::PCA(_parameters, pointCloud);
        auto pcaResult = pca.getResult();

        switch (pcaResult) {
        case Mono_LidarPipeline::PCA_Result::Point:
            return std::pair<DepthResultType, double>(DepthResultType::PcaIsPoint, -1);
        case Mono_LidarPipeline::PCA_Result::Linear:
            return std::pair<DepthResultType, double>(DepthResultType::PcaIsLine, -1);
        case Mono_LidarPipeline::PCA_Result::Cubic:
            return std::pair<DepthResultType, double>(DepthResultType::PcaIsCubic, -1);
        default:
            break;
        }

        // debug log PCA
        if (calcStats != NULL) {
            switch (pcaResult) {
            case Mono_LidarPipeline::PCA_Result::Point:
                calcStats->_pcaResult = "Point";
                break;
            case Mono_LidarPipeline::PCA_Result::Linear:
                calcStats->_pcaResult = "Line";
                break;
            case Mono_LidarPipeline::PCA_Result::Cubic:
                calcStats->_pcaResult = "Cubic";
                break;
            default:
                break;
            }

            calcStats->_pcaEigenVector1 = pca.getEigenVector1();
            calcStats->_pcaEigenVector2 = pca.getEigenVector2();
            calcStats->_pcaEigenVector3 = pca.getEigenVector3();
            calcStats->_pcaPlaneAnchor = pca.getPlaneAnchorPoint();
            calcStats->_pcaPlaneNormal = pca.getPlaneNormal();
        }

        // continues if the cloud is (approximately) a plane
        if (!_linePlaneIntersection->GetIntersectionPoint(pca.getPlaneNormal(),
                                                          pca.getPlaneAnchorPoint(),
                                                          viewingRaySupportPoint,
                                                          viewingRayDirection,
                                                          intersectionPoint,
                                                          depth))
            return std::pair<DepthResultType, double>(DepthResultType::PlaneViewrayNotOrthogonal, -1);
    } else {
        if (!_linePlaneIntersection->GetIntersectionPoint(
                corner1, corner2, corner3, viewingRaySupportPoint, viewingRayDirection, intersectionPoint, depth))
            return std::pair<DepthResultType, double>(DepthResultType::PlaneViewrayNotOrthogonal, -1);
    }

    if (this->_tresholdDepthGlobal != NULL) {
        auto result = this->_tresholdDepthGlobal->CheckInDepth(depth);
        if (result == eTresholdResult::SmallerMin)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthGlobalSmallerMin, -1);
        else if (result == eTresholdResult::GreaterMax)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthGlobalGreaterMax, -1);
    }

    if (this->_tresholdDepthLocal != NULL) {
        auto result = this->_tresholdDepthLocal->CheckInBounds(pointsSegmented, depth);
        if (result == eTresholdResult::SmallerMin)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthLocalSmallerMin, -1);
        else if (result == eTresholdResult::GreaterMax)
            return std::pair<DepthResultType, double>(DepthResultType::TresholdDepthLocalGreaterMax, -1);
    }

    if (depth < 0) {
        if (_parameters->do_use_cut_behind_camera) {
            return std::pair<DepthResultType, double>(DepthResultType::CornerBehindCamera, -1);
        }
    }

    //    #pragma omp critical
    //    {
    //    if (_parameters->do_publish_points)
    //    {
    //        _points_interpolated.push_back(intersectionPoint);
    //    }
    //    }

    return std::pair<DepthResultType, double>(DepthResultType::Success, depth);
}

void DepthEstimator::LogDepthCalcStats(const DepthResultType depthResult) {
    switch (depthResult) {
    case DepthResultType::CornerBehindCamera:
        _depthCalcStats.AddCornerBehindCamera();
        break;
    case DepthResultType::HistogramNoLocalMax:
        _depthCalcStats.AddHistogramNoLocalMax();
        break;
    case DepthResultType::PcaIsCubic:
        _depthCalcStats.AddPCAIsCubic();
        break;
    case DepthResultType::PcaIsLine:
        _depthCalcStats.AddPCAIsLine();
        break;
    case DepthResultType::PcaIsPoint:
        _depthCalcStats.AddPCAIsPoint();
        break;
    case DepthResultType::PlaneViewrayNotOrthogonal:
        _depthCalcStats.AddPlaneViewrayNotOrthogonal();
        break;
    case DepthResultType::RadiusSearchInsufficientPoints:
        _depthCalcStats.AddRadiusSearchInsufficientPoints();
        break;
    case DepthResultType::Success:
        _depthCalcStats.AddSuccess();
        break;
    case DepthResultType::TresholdDepthGlobalGreaterMax:
        _depthCalcStats.AddTresholdDepthGlobalGreaterMax();
        break;
    case DepthResultType::TresholdDepthGlobalSmallerMin:
        _depthCalcStats.AddTresholdDepthGlobalSmallerMin();
        break;
    case DepthResultType::TresholdDepthLocalGreaterMax:
        _depthCalcStats.AddTresholdDepthLocalGreaterMax();
        break;
    case DepthResultType::TresholdDepthLocalSmallerMin:
        _depthCalcStats.AddTresholdDepthLocalSmallerMin();
        break;
    case DepthResultType::TriangleNotPlanar:
        _depthCalcStats.AddTriangleNotPlanar();
        break;
    case DepthResultType::TriangleNotPlanarInsufficientPoints:
        _depthCalcStats.AddTriangleNotPlanarInsufficientPoints();
        break;
    case DepthResultType::InsufficientRoadPoints:
        _depthCalcStats.AddInsufficientRoadPoints();
        break;
    default:

        break;
    }
}


} /* namespace lidorb_ros_tool */
