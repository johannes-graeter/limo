#include "mono_lidar_depth/line_plane_intersection.hpp"

#include <cmath>
#include <limits>

namespace mono_lidar {

std::optional<RayPlaneIntersection> LinePlaneIntersection::intersect(const Eigen::Hyperplane<double, 3>& plane,
                                                                     const Eigen::Vector3d& ray_support,
                                                                     const Eigen::Vector3d& ray_direction) const {
    if (orthogonality_threshold_.has_value()) {
        const double alignment = std::abs(plane.normal().normalized().dot(ray_direction.normalized()));
        if (alignment < *orthogonality_threshold_) {
            return std::nullopt;
        }
    }

    const Eigen::ParametrizedLine<double, 3> line(ray_support, ray_direction.normalized());
    const Eigen::Vector3d intersection = line.intersectionPoint(plane);

    // The depth convention of the whole pipeline: z coordinate in camera coordinates.
    return RayPlaneIntersection{intersection, intersection.z()};
}

bool checkZxRatio(const VecOfVec3d& points, double threshold) {
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for (const auto& point : points) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_z = std::min(min_z, point.z());
        max_z = std::max(max_z, point.z());
    }

    const double size_x = max_x - min_x;
    const double size_z = max_z - min_z;

    return size_z / size_x >= threshold;
}

} // namespace mono_lidar
