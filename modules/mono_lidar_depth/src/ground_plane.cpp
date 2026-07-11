#include "mono_lidar_depth/ground_plane.hpp"

#include <algorithm>
#include <numeric>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace mono_lidar {

RansacPlane::RansacPlane(const DepthEstimatorParameters& parameters)
        : distance_threshold_(parameters.ransac_plane_distance_threshold),
          max_iterations_(parameters.ransac_plane_max_iterations),
          use_refinement_(parameters.ransac_plane_use_refinement),
          refinement_distance_(parameters.ransac_plane_refinement_threshold),
          probability_(parameters.ransac_plane_probability), sample_points_(parameters.ransac_plane_sample_points) {
}

void RansacPlane::calculateInliersPlane(const Cloud::ConstPtr& cloud, double min_z, double max_z) {
    if (cloud->points.size() < 3) {
        throw ExceptionPclInvalid();
    }

    std::vector<int> candidate_indices(cloud->points.size());
    std::iota(candidate_indices.begin(), candidate_indices.end(), 0);

    if (min_z > -1001.) {
        pcl::PassThrough<Point> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(static_cast<float>(min_z), static_cast<float>(max_z));
        pass.filter(candidate_indices);
    }

    if (sample_points_ > 100) {
        pcl::RandomSample<Point> sampler;
        sampler.setInputCloud(cloud);
        sampler.setIndices(std::make_shared<std::vector<int>>(candidate_indices));
        sampler.setSample(static_cast<unsigned int>(sample_points_));
        sampler.filter(candidate_indices);
    }

    auto model = std::make_shared<pcl::SampleConsensusModelPerpendicularPlane<Point>>(cloud);
    model->setAxis(Eigen::Vector3f(0.f, 0.f, 1.f));
    model->setEpsAngle(M_PI / 18.); // 10 degrees
    model->setIndices(candidate_indices);

    pcl::RandomSampleConsensus<Point> ransac(model);
    ransac.setDistanceThreshold(distance_threshold_);
    ransac.setMaxIterations(max_iterations_);
    ransac.setProbability(probability_);
    ransac.computeModel();

    std::vector<int> inliers;
    ransac.getInliers(inliers);

    Eigen::VectorXf model_coeffs;
    ransac.getModelCoefficients(model_coeffs);

    if (use_refinement_) {
        Eigen::VectorXf model_coeffs_refined;
        model->optimizeModelCoefficients(inliers, model_coeffs, model_coeffs_refined);
        model->selectWithinDistance(model_coeffs, refinement_distance_, inliers);
        model_coeffs = model_coeffs_refined;
    }

    model_coeffs_ = model_coeffs.head<4>();
    setInliers(std::move(inliers));
}

SemanticPlane::SemanticPlane(const cv::Mat& semantic_image,
                             Camera camera,
                             std::set<int> groundplane_labels,
                             double inlier_threshold)
        : semantic_image_(semantic_image), camera_(camera), groundplane_labels_(std::move(groundplane_labels)),
          inlier_threshold_(inlier_threshold) {
}

void SemanticPlane::calculateInliersPlaneImpl(const Cloud::ConstPtr& cloud) {
    // Project the cloud into the image and keep the points on ground-plane labels.
    Cloud::Ptr transformed_cloud(new Cloud());
    pcl::transformPointCloud(*cloud, *transformed_cloud, camera_.transform_cam_lidar);

    const Eigen::Matrix3d intrinsics = camera_.getIntrinsics();
    std::vector<int> ground_indices;
    for (int index = 0; index < static_cast<int>(transformed_cloud->points.size()); ++index) {
        const auto& p_in = transformed_cloud->points[static_cast<size_t>(index)];
        Eigen::Vector3d p = intrinsics * Eigen::Vector3d{p_in.x, p_in.y, p_in.z};
        p /= p.z();

        const cv::Point pixel(static_cast<int>(p.x()), static_cast<int>(p.y()));
        const bool in_image = pixel.x >= 0 && pixel.x < semantic_image_.cols && pixel.y >= 0 &&
                              pixel.y < semantic_image_.rows;
        if (!in_image) {
            continue;
        }
        const int label = static_cast<int>(semantic_image_.at<uchar>(pixel));
        if (groundplane_labels_.contains(label)) {
            ground_indices.push_back(index);
        }
    }

    if (ground_indices.size() < 3) {
        throw ExceptionPclInvalid();
    }

    // Fit a plane through the labeled ground points, then refine on all points near that plane.
    auto model = std::make_shared<pcl::SampleConsensusModelPlane<Point>>(cloud);
    Eigen::VectorXf model_coeffs;
    {
        Eigen::VectorXf initial_coeffs(4);
        initial_coeffs << 0.f, 0.f, 1.f, 0.f;
        model->optimizeModelCoefficients(ground_indices, initial_coeffs, model_coeffs);
    }

    std::vector<int> inliers_refined;
    Eigen::VectorXf model_coeffs_refined;
    model->selectWithinDistance(model_coeffs, inlier_threshold_, inliers_refined);
    model->optimizeModelCoefficients(inliers_refined, model_coeffs, model_coeffs_refined);

    model_coeffs_ = model_coeffs_refined.head<4>();
    setInliers(std::move(inliers_refined));
}

} // namespace mono_lidar
