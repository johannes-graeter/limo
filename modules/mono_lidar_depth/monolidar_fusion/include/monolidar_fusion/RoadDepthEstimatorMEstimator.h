/*
 * RoadDepthEstimatorMEstimator.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <Eigen/Eigen>
#include "PlaneEstimationMEstimator.h"
#include "RoadDepthEstimatorBase.h"
#include "eigen_stl_defs.h"

namespace Mono_Lidar {
class RoadDepthEstimatorMEstimator : public RoadDepthEstimatorBase {
public:
    // Specify Eigen Alignment, should be obsolete with c++17
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    RoadDepthEstimatorMEstimator();

    /*
     * Calculates the depth of a feature point in an image (with a triangulated point)
     */
    std::pair<DepthResultType, double> CalculateDepth(const Eigen::Vector2d& point_image,
                                                      const std::shared_ptr<CameraPinhole> _camera,
                                                      const VecOfVec3d& planePoints,
                                                      Eigen::Vector3d& pointIntersection) override;

    void setPlanePrior(const Eigen::Hyperplane<double, 3>& plane);

private:
    // Object for estimation of a plane with a given list of points
    std::shared_ptr<PlaneEstimationMEstimator> _planeEstimation;

    // Plane prior from ransac groundplane estimation
    bool _isPriorSet;
    Eigen::Hyperplane<double, 3> _priorPlane;
};
}
