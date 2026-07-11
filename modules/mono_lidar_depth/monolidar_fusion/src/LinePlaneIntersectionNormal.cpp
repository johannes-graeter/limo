/*
 * LinePlaneIntersectionNormal.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "LinePlaneIntersectionNormal.h"

namespace Mono_Lidar {
bool LinePlaneIntersectionNormal::GetIntersection(const Eigen::Hyperplane<double, 3>& plane,
                                                  const Eigen::Vector3d& n0,
                                                  const Eigen::Vector3d& n1,
                                                  Eigen::Vector3d& intersectionPoint,
                                                  double& intersectionDistance) {
    // create raycast
    Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(n0, n1);

    // calculate intersection between the raycast and the plane

    auto intersectionMatrix = line.intersectionPoint(plane);

    // write output parameters
    for (int i = 0; i < 3; i++)
        intersectionPoint(i) = intersectionMatrix(i, 0);

    // intersectionDistance = line.intersection(plane);
    intersectionDistance = intersectionMatrix(2, 0);

    return true;
}
}
