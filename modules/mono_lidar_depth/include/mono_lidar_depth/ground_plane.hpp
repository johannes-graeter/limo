// Copyright 2017-2026. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include <memory>
#include <set>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core/mat.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "parameters.hpp"

namespace mono_lidar {

/// Ground plane (the road) segmented from a lidar cloud; used for the road-feature depth
/// fallback. Implementations: RANSAC on the raw cloud, or — once segmented images are available —
/// a fit through semantically labeled ground points.
class GroundPlane {
public:
    using Ptr = std::shared_ptr<GroundPlane>;
    using Point = pcl::PointXYZI;
    using Cloud = pcl::PointCloud<Point>;

    struct ExceptionPclInvalid : public std::runtime_error {
        ExceptionPclInvalid() : std::runtime_error("In GroundPlane: Input pointcloud is invalid") {
        }
    };

    virtual ~GroundPlane() = default;

    virtual void calculateInliersPlane(const Cloud::ConstPtr& cloud, double min_z, double max_z) = 0;

    void calculateInliersPlane(const Cloud::ConstPtr& cloud) {
        calculateInliersPlane(cloud, -1000., 1000.);
    }

    /// True once a segmentation has run.
    bool isSegmented() const {
        return is_segmented_;
    }

    /// Plane coefficients (a, b, c, d) in lidar coordinates.
    const Eigen::Vector4f& modelCoeffs() const {
        return model_coeffs_;
    }

    /// Whether the cloud point with the given index is a plane inlier.
    bool isPointInPlane(int cloud_index) const {
        return inlier_lookup_.contains(cloud_index);
    }

    /// Indices (into the original cloud) of all plane inliers.
    const std::vector<int>& inlierIndices() const {
        return inlier_indices_;
    }

protected:
    void setInliers(std::vector<int> inlier_indices) {
        inlier_indices_ = std::move(inlier_indices);
        inlier_lookup_ = std::unordered_set<int>(inlier_indices_.begin(), inlier_indices_.end());
        is_segmented_ = true;
    }

    bool is_segmented_{false};
    Eigen::Vector4f model_coeffs_{Eigen::Vector4f::Zero()};

private:
    std::unordered_set<int> inlier_lookup_;
    std::vector<int> inlier_indices_;
};

/// Ground plane estimation with RANSAC: optional z passthrough filter, random subsampling, then
/// a perpendicular-plane model (lidar z axis, 10 degrees tolerance) with optional least-squares
/// refinement on the inliers.
class RansacPlane : public GroundPlane {
public:
    RansacPlane() = default;

    explicit RansacPlane(const DepthEstimatorParameters& parameters);

    void calculateInliersPlane(const Cloud::ConstPtr& cloud, double min_z, double max_z) override;
    using GroundPlane::calculateInliersPlane;

private:
    double distance_threshold_{0.};
    int max_iterations_{0};
    bool use_refinement_{false};
    double refinement_distance_{10000.};
    double probability_{0.999};
    int sample_points_{6000};
};

/// Ground plane from a semantically segmented image: lidar points are projected into the image,
/// points on ground-plane labels are kept, and a plane is fitted through them (with one
/// refinement round). This is the hook for the upcoming segmentation input.
class SemanticPlane : public GroundPlane {
public:
    struct Camera {
        double f;
        double cu;
        double cv;

        Eigen::Affine3d transform_cam_lidar;

        Eigen::Matrix3d getIntrinsics() const {
            Eigen::Matrix3d out;
            out << f, 0., cu, 0., f, cv, 0., 0., 1.;
            return out;
        }
    };

    SemanticPlane(const cv::Mat& semantic_image,
                  Camera camera,
                  std::set<int> groundplane_labels,
                  double inlier_threshold);

    void calculateInliersPlane(const Cloud::ConstPtr& cloud, double /*min_z*/, double /*max_z*/) override {
        calculateInliersPlaneImpl(cloud);
    }
    using GroundPlane::calculateInliersPlane;

private:
    void calculateInliersPlaneImpl(const Cloud::ConstPtr& cloud);

    cv::Mat semantic_image_;
    Camera camera_;
    std::set<int> groundplane_labels_;
    double inlier_threshold_{0.1};
};

} // namespace mono_lidar
