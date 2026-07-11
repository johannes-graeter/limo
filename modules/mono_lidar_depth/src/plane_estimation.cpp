#include "mono_lidar_depth/plane_estimation.hpp"

namespace mono_lidar {

std::optional<std::array<Eigen::Vector3d, 3>> maxSpanningTriangle(const VecOfVec3d& points, double min_distance) {
    const auto point_count = static_cast<int>(points.size());
    if (point_count < 3) {
        return std::nullopt;
    }
    const double min_distance_squared = min_distance * min_distance;

    // The two points with the greatest mutual distance...
    int max_i = -1;
    int max_j = -1;
    double max_dist = -1.;
    for (int i = 0; i < point_count - 1; ++i) {
        for (int j = i + 1; j < point_count; ++j) {
            const double dist = (points[i] - points[j]).squaredNorm();
            if (dist > max_dist) {
                max_dist = dist;
                max_i = i;
                max_j = j;
            }
        }
    }

    if (max_dist <= min_distance_squared) {
        return std::nullopt; // all points coincide (within tolerance)
    }

    // ...plus the point maximizing the summed distance to both.
    int max_k = -1;
    double max_dist_sum = -1.;
    for (int k = 0; k < point_count; ++k) {
        if (k == max_i || k == max_j) {
            continue;
        }
        const double dist1 = (points[k] - points[max_i]).squaredNorm();
        if (dist1 <= min_distance_squared) {
            continue;
        }
        const double dist2 = (points[k] - points[max_j]).squaredNorm();
        if (dist2 <= min_distance_squared) {
            continue;
        }
        if (dist1 + dist2 > max_dist_sum) {
            max_dist_sum = dist1 + dist2;
            max_k = k;
        }
    }

    if (max_k == -1) {
        return std::nullopt;
    }

    return std::array<Eigen::Vector3d, 3>{points[max_i], points[max_j], points[max_k]};
}

bool isTrianglePlanar(const Eigen::Vector3d& corner1,
                      const Eigen::Vector3d& corner2,
                      const Eigen::Vector3d& corner3,
                      double threshold) {
    const Eigen::Vector3d edge1 = (corner2 - corner1).normalized();
    const Eigen::Vector3d edge2 = (corner3 - corner1).normalized();
    const Eigen::Vector3d edge3 = (corner3 - corner2).normalized();

    return edge1.cross(edge2).norm() >= threshold && edge1.cross(edge3).norm() >= threshold &&
           edge2.cross(edge3).norm() >= threshold;
}

namespace {

/// Total/weighted least-squares plane through weighted points: normal is the left-singular vector
/// of the smallest singular value of the weighted, centered point matrix.
Eigen::Hyperplane<double, 3> fitPlaneSvd(const VecOfVec3d& points, const Eigen::VectorXd& weights) {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < points.size(); ++i) {
        center += weights(static_cast<Eigen::Index>(i)) * points[i];
    }
    center /= weights.sum();

    Eigen::Matrix3Xd centered(3, static_cast<Eigen::Index>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        centered.col(static_cast<Eigen::Index>(i)) =
            std::sqrt(weights(static_cast<Eigen::Index>(i))) * (points[i] - center);
    }

    const auto svd = centered.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::Vector3d normal = svd.matrixU().rightCols<1>().normalized();

    return Eigen::Hyperplane<double, 3>(normal, center);
}

} // namespace

std::optional<Eigen::Hyperplane<double, 3>> fitPlaneLeastSquares(const VecOfVec3d& points) {
    if (points.size() < 3) {
        return std::nullopt;
    }
    return fitPlaneSvd(points, Eigen::VectorXd::Ones(static_cast<Eigen::Index>(points.size())));
}

std::optional<Eigen::Hyperplane<double, 3>> fitPlaneMEstimator(const VecOfVec3d& points,
                                                               const Eigen::Hyperplane<double, 3>& prior) {
    if (points.size() < 3) {
        return std::nullopt;
    }

    // Inverse-distance weights relative to the prior; epsilon guards points exactly on the prior.
    constexpr double kMinDistance = 1e-9;
    Eigen::VectorXd weights(static_cast<Eigen::Index>(points.size()));
    for (size_t i = 0; i < points.size(); ++i) {
        weights(static_cast<Eigen::Index>(i)) = 1. / std::max(prior.absDistance(points[i]), kMinDistance);
    }

    return fitPlaneSvd(points, weights);
}

} // namespace mono_lidar
