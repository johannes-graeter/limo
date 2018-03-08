// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#include "internal/landmark_selection_scheme_voxel.hpp"

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include "internal/landmark_selection_scheme_helpers.hpp"

#include <chrono>

namespace keyframe_bundle_adjustment {

// TypeDefs
using Point = pcl::PointXYZL;
using Cloud = pcl::PointCloud<Point>;

std::set<LandmarkId> LandmarkSelectionSchemeVoxel::getSelection(const LandmarkMap& landmarks,
                                                                const KeyframeMap& keyframes) const {
    auto categorized_lms = getCategorizedSelection(landmarks, keyframes);
    std::set<LandmarkId> out;

    for (const auto& el : categorized_lms) {
        out.insert(el.first);
    }
    return out;
}

namespace {
void filterXYZ(const Cloud::Ptr& cloudInput,
               const std::array<double, 3>& bounds,
               Cloud::Ptr& cloudFiltered,
               std::set<int>& removed_labels) {

    // Indices needed for filtering.
    auto indices_x = boost::make_shared<std::vector<int>>();
    auto indices_xy = boost::make_shared<std::vector<int>>();

    // Filter roi by x,y,z to apply voxelization (otherwise leaf size will be bad)
    pcl::PassThrough<Point> pass(true);
    // Filter x.
    pass.setInputCloud(cloudInput);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-bounds[0] / 2., bounds[0] / 2.);
    pass.filter(*indices_x);
    // Add removed ids to far away category.
    for (const auto& el : *pass.getRemovedIndices()) {
        auto l = cloudInput->points[el].label;
        removed_labels.insert(l);
    }
    // Filter y.
    pass.setIndices(indices_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-bounds[1] / 2., bounds[1] / 2.);
    pass.filter(*indices_xy);
    // Add removed ids to far away category.
    for (const auto& el : *pass.getRemovedIndices()) {
        auto l = cloudInput->points[el].label;
        removed_labels.insert(l);
    }
    // Filter z.
    pass.setIndices(indices_xy);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-bounds[2] / 2., bounds[2] / 2.);
    pass.filter(*cloudFiltered);
    // Add removed ids to far away category.
    for (const auto& el : *pass.getRemovedIndices()) {
        auto l = cloudInput->points[el].label;
        removed_labels.insert(l);
    }
}
}
std::map<LandmarkId, LandmarkCategorizatonInterface::Category> LandmarkSelectionSchemeVoxel::getCategorizedSelection(
    const LandmarkMap& lms, const KeyframeMap& keyframes) const {

    // Get current position
    auto it = std::max_element(keyframes.cbegin(), keyframes.cend(), [](const auto& a, const auto& b) {
        return a.second->timestamp_ < b.second->timestamp_;
    });
    Eigen::Isometry3d cur_pos = it->second->getEigenPose();
    // lookup for ids (uint32t may be too small for global ids)
    std::map<uint32_t, LandmarkId> lut;
    uint32_t ind = 0;

    auto start_time_pcl = std::chrono::steady_clock::now();
    // Copy input relative to current keyframe
    ///@todo Can I economize that copy?
    Cloud::Ptr cloudInput(new Cloud);
    for (const auto& id_lm : lms) {
        Eigen::Vector3d p_eig(id_lm.second->pos[0], id_lm.second->pos[1], id_lm.second->pos[2]);
        p_eig = cur_pos * p_eig;
        Point p;
        p.x = p_eig[0];
        p.y = p_eig[1];
        p.z = p_eig[2];
        p.label = ind;

        lut[ind] = id_lm.first;
        ind++;

        cloudInput->points.push_back(p);
    }

    std::cout << "Duration copy="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_pcl)
                     .count()
              << " ms" << std::endl;

    std::cout << "Size before voxelization=" << lms.size() << std::endl;

    // Get roi in xyz for voxelization
    Cloud::Ptr cloud_filtered(new Cloud);
    std::set<int> labels_far;
    filterXYZ(cloudInput, params_.roi_far_xyz, cloud_filtered, labels_far);
    std::cout << "labels_far " << labels_far.size() << std::endl;

    // Voxelize the cloud.
    pcl::VoxelGrid<Point> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setLeafSize(params_.voxel_size_xyz[0], params_.voxel_size_xyz[1], params_.voxel_size_xyz[2]);
    sor.filter(*cloud_filtered);


    // Filter XYZ with smaller roi for categorization.
    Cloud::Ptr near_cloud(new Cloud);
    std::set<int> labels_middle;
    filterXYZ(cloud_filtered, params_.roi_middle_xyz, near_cloud, labels_middle);
    std::cout << "labels_middle " << labels_middle.size() << std::endl;

    std::cout << "Duration pcl stuff="
              << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                       start_time_pcl)
                     .count()
              << " ms" << std::endl;
    // Add near landmarks labels.
    // Calc flow.
    auto map_flow = landmark_helpers::calcMeanFlow(lms, keyframes);

    // Convert from label to id.
    std::vector<LandmarkId> ids_near;
    ids_near.reserve(near_cloud->points.size());
    for (const auto& p : near_cloud->points) {
        ids_near.push_back(lut.at(p.label));
    }
    std::cout << "num lms near=" << ids_near.size() << std::endl;

    // Add to output.
    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> out;
    for (const auto& id : landmark_helpers::chooseNearLmIds(params_.max_num_landmarks_near, ids_near, map_flow)) {
        out[id] = LandmarkCategorizatonInterface::Category::NearField;
    }

    // Add middle labels.
    std::vector<LandmarkId> ids_middle;
    ids_middle.reserve(labels_middle.size());
    for (const auto& l : labels_middle) {
        ids_middle.push_back(lut.at(l));
    }
    for (const auto& id : landmark_helpers::chooseMiddleLmIds(params_.max_num_landmarks_middle, ids_middle)) {
        out[id] = LandmarkCategorizatonInterface::Category::MiddleField;
    }

    // Add far labels
    std::vector<LandmarkId> ids_far;
    ids_far.reserve(labels_far.size());
    for (const auto& l : labels_far) {
        ids_far.push_back(lut.at(l));
    }
    for (const auto& id : landmark_helpers::chooseFarLmIds(params_.max_num_landmarks_far, ids_far, keyframes)) {
        out[id] = LandmarkCategorizatonInterface::Category::FarField;
    }

std::cout << "AFter voxelization: near="
          << std::count_if(
                 out.cbegin(),
                 out.cend(),
                 [](const auto& a) { return a.second == LandmarkCategorizatonInterface::Category::NearField; })
          << " middle="
          << std::count_if(
                 out.cbegin(),
                 out.cend(),
                 [](const auto& a) { return a.second == LandmarkCategorizatonInterface::Category::MiddleField; })
          << " far=" <<
    std::count_if(out.cbegin(),
                  out.cend(),
                  [](const auto& a) { return a.second == LandmarkCategorizatonInterface::Category::FarField; })
            << std::endl;


    return out;
}
}
