#include "mono_lidar_depth/pca.hpp"

namespace mono_lidar {

PlanarityPca::PlanarityPca(const Eigen::MatrixXd& points,
                           double threshold_point_max,
                           double threshold_line_min,
                           double threshold_cubic_max) {
    // The centroid is both the PCA anchor and the plane anchor point.
    mean_ = points.rowwise().mean();
    const Eigen::MatrixXd centered = points.colwise() - mean_;
    const Eigen::Matrix3d cov = centered * centered.adjoint();

    // Eigenvalues are sorted ascending; the plane normal is the direction of least variance.
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
    const Eigen::Vector3d eigenvalues = eig.eigenvalues();
    plane_normal_ = eig.eigenvectors().col(0).normalized();

    const double smallest = eigenvalues(0);
    const double middle = eigenvalues(1);
    const double largest = eigenvalues(2);

    const double planarity = (middle - smallest) / largest;
    // Ratio of the two dominant extents; a large value means the patch is essentially a line.
    // (The former formula (largest - middle) / largest is bounded by 1 and could never reach the
    // default threshold of 15, so the line check was dead.)
    const double line_ratio = largest / middle;

    // Only the 'plane' outcome lets the depth estimation proceed, so the order of the failure
    // checks does not change which patches are accepted — it is chosen so each degenerate case
    // gets its true label.
    if (largest < threshold_point_max) {
        result_ = PcaResult::point;
    } else if (line_ratio > threshold_line_min) {
        result_ = PcaResult::linear;
    } else if (planarity < threshold_cubic_max) {
        result_ = PcaResult::cubic;
    } else {
        result_ = PcaResult::plane;
    }
}

} // namespace mono_lidar
