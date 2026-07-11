/*
 * Copyright 2014. All rights reserved.
 * Institute of Measurement and Control Systems
 * Karlsruhe Institute of Technology, Germany
 *
 * authors:
 *  Johannes Graeter (johannes.graeter@kit.edu)
 *  and others
 */
#pragma once

#include <cassert>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include "RansacBase.h"

namespace ransac {
// struct PlaneCandidate : public CandidateBase<Eigen::Vector3d, std::pair<Eigen::Vector3d, double>> {
//    PlaneCandidate() = default;

//    void calculate_candidate(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>&
//    MinimalSamples) {
//        Eigen::Vector3d V0 = MinimalSamples[0] - MinimalSamples[1];
//        Eigen::Vector3d V1 = MinimalSamples[0] - MinimalSamples[2];
//        Eigen::Vector3d Normal = V0.cross(V1);

//        assert(Normal.norm() != 0. && "Normal has norm zero");
//        Normal = Normal / Normal.norm();
//        double d = double(Normal.dot(MinimalSamples[0]));

//        CandidateModel.first = Normal;
//        CandidateModel.second = d;
//    }
//    double evaluate_candidate(const Eigen::Vector3d& Point) {
//        float Cur = CandidateModel.first.dot(Point);
//        double Residuum = double(Cur) - CandidateModel.second;
//        return std::abs(Residuum);
//    }
//};

///**
//*  @class RANSACPlaneFitter
//*  @par
//*
//*  concrete implementation of a Ransac plane fit
//*/
// class RansacPlaneModel : public RansacBase<Eigen::Vector3d, PlaneCandidate> {
// public: /* public classes/enums/types etc... */
// public: /* public methods */
//    /**
//  * default constructor
//  */
//    RansacPlaneModel() : RansacBase(3) {
//        // MinNUmberSamples=3
//        // this->MinNumberSamples=3
//        ;
//    }

//    /**
//  * default destructor
//  */
//    ~RANSACPlaneFitter() = default;
//};
struct PlaneCandidate : public CandidateBase<Eigen::Vector4f, 3> {
    PlaneCandidate() = default;

    void calculate_candidate(const CandidateBase::MatrixConstRef& MinimalSamples) {
        ///@todo do this with Eigen::Map
        Eigen::Vector3f V0 = MinimalSamples.col(0) - MinimalSamples.col(1);
        Eigen::Vector3f V1 = MinimalSamples.col(0) - MinimalSamples.col(2);
        Eigen::Vector3f Normal = V0.cross(V1);

        assert(Normal.norm() != 0. && "Normal has norm zero");
        Normal = Normal / Normal.norm();
        float sign = Normal.z() > 0. ? 1. : -1.;
        float d = sign * Normal.dot(MinimalSamples.col(0));

        // z always positive
        Normal *= sign;
        d *= sign;

        model[0] = Normal[0];
        model[1] = Normal[1];
        model[2] = Normal[2];
        model[3] = d;
    }
    Eigen::VectorXf evaluate_candidates(const CandidateBase::MatrixConstRef& Points) {
        return (model.head<3>().transpose() * Points).transpose().array() + model[3];
    }
};

/**
*  @class RANSACPlaneFitter
*  @par
*
*  concrete implementation of a Ransac plane fit
*/
class RansacPlaneModel : public RansacBase<PlaneCandidate, 3> {
public: /* public classes/enums/types etc... */
public: /* public methods */
    /**
  * default constructor
  */
    RansacPlaneModel() : RansacBase(3) {
        // MinNUmberSamples=3
        // this->MinNumberSamples=3
        ;
    }

    /**
  * default destructor
  */
    ~RansacPlaneModel() = default;
};
}
