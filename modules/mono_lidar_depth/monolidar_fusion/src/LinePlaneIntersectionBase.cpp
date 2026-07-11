/*
 * LinePlaneIntersectionBase.cpp
 *
 *  Created on: Mar 23, 2017
 *      Author: wilczynski
 */

#include "LinePlaneIntersectionBase.h"

namespace Mono_Lidar {

bool LinePlaneIntersectionBase::GetIntersectionPoint(const Eigen::Vector3d& planeNormal,
                                                     const Eigen::Vector3d& planePoint,
                                                     const Eigen::Vector3d& n0,
                                                     const Eigen::Vector3d& n1,
                                                     Eigen::Vector3d& intersectionPoint,
                                                     double& intersectionDistance) {
    auto plane = Eigen::Hyperplane<double, 3>(planeNormal, planePoint);

    return GetIntersection(plane, n0, n1, intersectionPoint, intersectionDistance);
}

bool LinePlaneIntersectionBase::GetIntersectionPoint(const Eigen::Vector3d& planeNormal,
                                                     const double planeDistance,
                                                     const Eigen::Vector3d& n0,
                                                     const Eigen::Vector3d& n1,
                                                     Eigen::Vector3d& intersectionPoint,
                                                     double& intersectionDistance) {
    auto plane = Eigen::Hyperplane<double, 3>(planeNormal, planeDistance);

    return GetIntersection(plane, n0, n1, intersectionPoint, intersectionDistance);
}

bool LinePlaneIntersectionBase::GetIntersectionPoint(const Eigen::Vector3d& p1,
                                                     const Eigen::Vector3d& p2,
                                                     const Eigen::Vector3d& p3,
                                                     const Eigen::Vector3d& n0,
                                                     const Eigen::Vector3d& n1,
                                                     Eigen::Vector3d& intersectionPoint,
                                                     double& intersectionDistance) {
    auto plane = Eigen::Hyperplane<double, 3>::Through(p1, p2, p3);

    return GetIntersection(plane, n0, n1, intersectionPoint, intersectionDistance);
}
}
