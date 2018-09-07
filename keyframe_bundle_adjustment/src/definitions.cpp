#include <internal/definitions.hpp>
namespace keyframe_bundle_adjustment {

Landmark::Landmark() {
    ;
}

Landmark::Landmark(const Eigen::Vector3d& p, bool has_depth) : has_measured_depth(has_depth) {
    pos[0] = p[0];
    pos[1] = p[1];
    pos[2] = p[2];
}

Pose convert(EigenPose p) {
    Eigen::Quaterniond q(p.rotation());
    // why does q.normalize() not work?
    Pose pose;
    pose[0] = q.w();
    pose[1] = q.x();
    pose[2] = q.y();
    pose[3] = q.z();

    pose[4] = p.translation()[0];
    pose[5] = p.translation()[1];
    pose[6] = p.translation()[2];

    return pose;
}

Camera::Camera(double f, const Eigen::Vector2d& pp, const EigenPose& pose_cam_veh)
        : focal_length(f), principal_point(pp) {
    pose_camera_vehicle = convert(pose_cam_veh);
    intrin_inv = getIntrinsicMatrix().inverse();
}

Eigen::Matrix3d Camera::getIntrinsicMatrix() const {
    Eigen::Matrix3d intrin;
    intrin << focal_length, 0., principal_point[0], 0., focal_length, principal_point[1], 0., 0., 1.;
    return intrin;
}

EigenPose Camera::getEigenPose() const {
    return convert(pose_camera_vehicle);
}

Eigen::Vector3d Camera::getViewingRay(const Measurement& m) {
    Eigen::Vector3d meas_hom{m.u, m.v, 1.};

    meas_hom = intrin_inv * meas_hom;
    meas_hom.normalize();

    return meas_hom;
}

double calcRotRoccMetric(const Eigen::Isometry3d& transform_1_0,
                         const Eigen::Matrix3d& intrinsics,
                         const Eigen::Vector3d& lm_0,
                         const Eigen::Vector2d& measurement_1) {
    // undefined for translation=0.
    if (transform_1_0.translation().norm() < 0.0001) {
        return 0.;
    }
    Eigen::Vector2d reproj_error_vec = measurement_1 - reproject(transform_1_0, intrinsics, lm_0);
    Eigen::Vector2d rot_error_vec = measurement_1 - reproject(transform_1_0.rotation(), intrinsics, lm_0);
    return reproj_error_vec.norm() / std::max(rot_error_vec.norm(), 1e-10);
}

TimestampSec convert(const TimestampNSec& ts) {
    return static_cast<TimestampSec>(ts * 1e-09);
}

TimestampNSec convert(const TimestampSec& ts) {
    return static_cast<TimestampNSec>(ts * 1e09);
}

double getCost(const std::map<ResidualId, std::pair<LandmarkId, int>>& ids, ceres::Problem& problem) {
    std::vector<ResidualId> v;
    v.reserve(ids.size());
    for (const auto& el : ids) {
        v.push_back(el.first);
    }
    return getCost(v, problem);
}

double getCost(const std::vector<ResidualId>& ids, ceres::Problem& problem) {
    // If residual ids in EvaluateOptoin is empty, total cost is returned, this is not what we intend to do.
    if (ids.size() == 0) {
        std::cout << "no ids" << std::endl;
        return 0.;
    }
    double cost = 0.;
    ceres::Problem::EvaluateOptions opt;
    opt.residual_blocks = ids;
    problem.Evaluate(opt, &cost, NULL, NULL, NULL);
    return cost;
}

Eigen::Matrix<double, 3, 1> convertMeasurementToRay(const Eigen::Matrix3d& intrin_inv, const Measurement& m) {
    Eigen::Matrix<double, 3, 1> ray =
        (intrin_inv * Eigen::Vector3d(static_cast<double>(m.u), static_cast<double>(m.v), 1.)).normalized();
    return ray;
}

double calcQuaternionDiff(const Pose& p0, const Pose& p1) {
    Eigen::Quaterniond q0(p0[0], p0[1], p0[2], p0[3]);
    Eigen::Quaterniond q1(p1[0], p1[1], p1[2], p1[3]);

    Eigen::Quaterniond q10 = q1.inverse() * q0;
    Eigen::AngleAxisd a(q10);
    return a.angle();
}
}
