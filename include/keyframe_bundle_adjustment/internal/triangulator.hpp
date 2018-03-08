/*
 * Copyright 2016. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */

#pragma once

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>

#include "definitions.hpp"

namespace keyframe_bundle_adjustment {
/**
*  @class Triangulator
* The reconstructed points are defined in the coordinate system in which poses are defined
*
*/
template <typename T>
class Triangulator {
public: // public classes/enums/types etc...
    using v3T = Eigen::Matrix<T, 3, 1>;
    using t3T = Eigen::Transform<T, 3, Eigen::Isometry>;
    using PoseAndRay = std::pair<std::shared_ptr<t3T>, v3T>; // so that pose can be stored somewhere else

    using RayTracklet = std::vector<std::pair<v3T, keyframe_bundle_adjustment::PoseId>>;

public: // attributes
public: // public methods
        /*
        * default constructor
        */
    Triangulator() = default;

    ///@brief get landmark from rays and poses of 1 track, reference is poses_rays[0]
    v3T triangulate_rays(const std::vector<PoseAndRay>& poses_rays) {
        assert(poses_rays.size() > 1);

        Eigen::Matrix<T, 3, 3> sum_rrt;
        v3T rhs;
        sum_rrt.setZero();
        rhs.setZero();
        Eigen::Matrix<T, 3, 3> id_3 = Eigen::Matrix<T, 3, 3>::Identity();

        for (const auto& p_r : poses_rays) {
            const auto& pose_origin_camera = *p_r.first;
            const auto& ray_camera = p_r.second;

            v3T r_trans = pose_origin_camera.rotation() * ray_camera;

            Eigen::Matrix<T, 3, 3> cur_rrt = id_3 - r_trans * r_trans.transpose();
            sum_rrt += cur_rrt;
            rhs += cur_rrt * pose_origin_camera.translation(); // for svp else: rhs+= cur_rrt * pose * p0
        }

        // this is the reconstructed point
        v3T p = sum_rrt.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(rhs);

        return p;
    }

    ///@brief wrapper for triangulation to pair poses and tracklets
    v3T process(const std::vector<std::pair<v3T, keyframe_bundle_adjustment::PoseId>>& t,
                const std::map<keyframe_bundle_adjustment::PoseId, t3T>& map_poses) {
        std::vector<PoseAndRay> poses_and_rays;
        poses_and_rays.reserve(t.size());
        for (const auto& m : t) {
            const auto& cur_pose = map_poses.at(m.second);
            //@todo don't copy landmarks
            poses_and_rays.push_back(std::make_pair(std::make_shared<t3T>(cur_pose), m.first));
        }

        return triangulate_rays(poses_and_rays);
    }

    ///@brief triangulate all tracks
    std::map<keyframe_bundle_adjustment::LandmarkId, v3T> process(
        const std::map<keyframe_bundle_adjustment::LandmarkId, RayTracklet>& tracklets,
        const std::map<keyframe_bundle_adjustment::PoseId, t3T>& map_poses) {
        std::map<keyframe_bundle_adjustment::LandmarkId, v3T> out;

        for (const auto& t : tracklets) {
            if (t.second.size() > 1) {
                auto p = process(t.second, map_poses);
                out[t.first] = p;
            }
        }

        return out;
    }
};
}
