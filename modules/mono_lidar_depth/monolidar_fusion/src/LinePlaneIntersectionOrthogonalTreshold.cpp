/*
 * LinePlaneIntersectionOrthogonalTreshold.cpp
 *
 *  Created on: Mar 15, 2017
 *      Author: wilczynski
 */

#include "LinePlaneIntersectionOrthogonalTreshold.h"

namespace Mono_Lidar {

LinePlaneIntersectionOrthogonalTreshold::LinePlaneIntersectionOrthogonalTreshold(const double treshold)
        : _treshold(treshold) {
}

bool LinePlaneIntersectionOrthogonalTreshold::GetIntersection(const Eigen::Hyperplane<double, 3>& plane,
                                                              const Eigen::Vector3d& n0,
                                                              const Eigen::Vector3d& n1,
                                                              Eigen::Vector3d& intersectionPoint,
                                                              double& intersectionDistance) {
    // create raycast
    Eigen::ParametrizedLine<double, 3> line = Eigen::ParametrizedLine<double, 3>::Through(n0, n1);

    // Check if the treshold condition is fulfilled
    Eigen::Vector3d lineNormal = n1.normalized();
    Eigen::Vector3d planeNormal = plane.normal();

    if (!CheckPlaneViewRayOrthogonal(planeNormal.normalized(), lineNormal, _treshold))
        return false;

    // calculate intersection between the raycast and the plane
    auto intersectionMatrix = line.intersectionPoint(plane);

    // write output parameters
    for (int i = 0; i < 3; i++)
        intersectionPoint(i) = intersectionMatrix(i, 0);

    // intersectionDistance = line.intersection(plane);
    intersectionDistance = intersectionMatrix(2, 0);

    return true;
}

bool LinePlaneIntersectionOrthogonalTreshold::CheckPlaneViewRayOrthogonal(const Eigen::Vector3d& planeNormalVec,
                                                                          const Eigen::Vector3d& viewingRay,
                                                                          double treshold) {
    return (fabs(planeNormalVec.dot(viewingRay)) >= treshold);
}
}
