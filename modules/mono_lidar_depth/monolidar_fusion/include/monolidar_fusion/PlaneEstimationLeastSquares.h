/*
 * PlaneEstimationLeastSquares.h
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#pragma once

#include <vector>
#include <Eigen/Eigen>
#include "eigen_stl_defs.h"

namespace Mono_Lidar {
class PlaneEstimationLeastSquares {
public:
    PlaneEstimationLeastSquares();

    bool EstimatePlane(const VecOfVec3d& points, Eigen::Vector3d& planeNormal, double& resultDist);
};
}
