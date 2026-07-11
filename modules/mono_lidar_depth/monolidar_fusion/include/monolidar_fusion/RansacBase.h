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
#include <ctime>
#include <iostream>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <tuple>
#include <vector>
#include <Eigen/Eigen>

namespace ransac {
template <class Model, int SizeDataPoint>
struct CandidateBase {
    // Just in case InputType should be an eigen type
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using MatrixType = Eigen::Matrix<float, SizeDataPoint, Eigen::Dynamic>;
    using MatrixConstRef = Eigen::Ref<const Eigen::Matrix<float, SizeDataPoint, Eigen::Dynamic>>;

    CandidateBase() = default;

    Model model;
    std::vector<size_t> InlierIndices{};
    std::vector<size_t> OutlierIndices{};

    virtual Eigen::VectorXf evaluate_candidates(const MatrixConstRef& Data) = 0;
    virtual void calculate_candidate(const MatrixConstRef& MinimalSamples) = 0;
    /**
         * @brief reset all parameters in candidate, will be called for each new ransac iteration
         */
    virtual void reset() {
        // note that vector capacity is still high after clear (better for performance in ransac?) swap trick:
        // my_data.swap(std::vector<int>());
        InlierIndices.clear();
        OutlierIndices.clear();
    }
};


/**
*  @class RANSACFitterBase
*  @par
*
*  base class for Ransac implementation
*  Usage: derivate concrete class and implement the evaluattion and candidate calculation methods.
* Input data type ist templated
*/
template <class ConcreteCandidate, int SizeDataPoint>
class RansacBase {
public: /* public classes/enums/types etc... */
    class RandomNumberGenerator {
    public:
        RandomNumberGenerator() : rng(rd()) {
        }

        inline int getRandNum(int limit) {
            return (numbers(rng) % limit);
        }
        std::set<int> getRandSequence(int limit, size_t n) {
            std::set<int> generatedSequence;
            while (generatedSequence.size() < n) {
                generatedSequence.insert(getRandNum(limit));
            }
            return generatedSequence;
        }

    private:
        std::random_device rd;
        std::mt19937 rng;
        std::uniform_int_distribution<int> numbers;
    };

public: /* public methods */
    // just in case ConcreteCandidate should be an eigentype
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using MatrixConstRef = Eigen::Ref<const Eigen::Matrix<float, SizeDataPoint, Eigen::Dynamic>>;
    /**
  * default constructor
  */
    RansacBase(int MinNumberSamples) : MinNumberSamples(MinNumberSamples) {
        ;
    }

    /**
  * default destructor
  */
    virtual ~RansacBase() = default;

    /**
 * @brief calcualte binomial coefficient n over k
 * @return  void
 * @par void
 */
    unsigned long long calc_binomial_coefficient(const uint& n, const uint& k) {
        // formula see https://en.wikipedia.org/wiki/Binomial_coefficient#Multiplicative_formula
        unsigned long long Out = 1;
        for (size_t i = 1; i <= k; ++i) {
            Out = Out * (n + 1 - i) / i;
        }
        return Out;
    }

    /**
     * @brief process
     * @param Data, matrix with data: one data point in each column
     * @param NumberIterations
     * @param InlierThreshold
     * @return
     */
    ConcreteCandidate process(const MatrixConstRef& Data, uint NumberIterations, double InlierThreshold) {
        assert(MinNumberSamples != 0 &&
               "In RANSAC: minimum number samples not set (do it in constructor) of concrete ransac fitter");
        // possible kombinations: n=Data.size(), k=MinNUmSamples: (n–1+k)! / (k!·(n–1)!)
        unsigned long long NumberPermutations =
            calc_binomial_coefficient(Data.cols(), MinNumberSamples); // number of permutations without laying back
        // http://www.brefeld.homepage.t-online.de/stochastik-formeln.html

        unsigned long long NumberIterationsLong = NumberIterations;
        NumberIterationsLong = std::min(NumberIterationsLong, NumberPermutations);
        assert(NumberIterationsLong <= std::numeric_limits<int>::max());
        NumberIterations = int(NumberIterationsLong);

        auto start_time_sample = std::chrono::steady_clock::now();
        std::vector<std::set<int>> VectorOfDrawnNumbers;
        {
            // generate set of drawn numbers
            std::set<std::set<int>> SetOfDrawnNumbers;
            RandomNumberGenerator RandGen;

            while (SetOfDrawnNumbers.size() < NumberIterations) {
                SetOfDrawnNumbers.insert(RandGen.getRandSequence(Data.cols(), MinNumberSamples));
            }

            VectorOfDrawnNumbers.reserve(NumberIterations);
            for (const auto& el : SetOfDrawnNumbers) {
                VectorOfDrawnNumbers.push_back(el);
            }
        }
        std::cout << "Duration sample candidates="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_time_sample)
                         .count()
                  << " ms" << std::endl;

        auto start_time_eval = std::chrono::steady_clock::now();

        std::vector<ConcreteCandidate> BestCandidates;
        BestCandidates.resize(VectorOfDrawnNumbers.size());
        //#pragma omp parallel for
        for (int i = 0; i < int(VectorOfDrawnNumbers.size()); ++i) {
            const auto& RandomNumbers = VectorOfDrawnNumbers[i];
            // only reset, because parameters are possibly stored on PrototypeCandidate that are needed
            ConcreteCandidate PrototypeCandidate;

            //            for (auto& el : RandomNumbers) {
            //                assert(el != -1);
            //            }

            // Calc plane candidate
            Eigen::Matrix<float, SizeDataPoint, Eigen::Dynamic> MinimalSet;
            MinimalSet.resize(SizeDataPoint, RandomNumbers.size());
            {
                int index = 0;
                for (const auto& el : RandomNumbers) {
                    MinimalSet.col(index) = Eigen::VectorXf(Data.col(el));
                    index++;
                }
            }

            PrototypeCandidate.calculate_candidate(MinimalSet);

            // calc inliers
            Eigen::VectorXf Residuals = PrototypeCandidate.evaluate_candidates(Data);
            Eigen::Matrix<bool, Eigen::Dynamic, 1> is_inlier = Residuals.array().abs() < InlierThreshold;
            for (int index = 0; index < is_inlier.rows(); ++index) {
                if (is_inlier[index]) {
                    PrototypeCandidate.InlierIndices.push_back(index);
                } else {
                    PrototypeCandidate.OutlierIndices.push_back(index);
                }
            }

            BestCandidates[i] = PrototypeCandidate;
        }

        std::cout << "Duration eval="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                           start_time_eval)
                         .count()
                  << " ms" << std::endl;

        auto it = std::max_element(BestCandidates.cbegin(), BestCandidates.cend(), [](const auto& a, const auto& b) {
            return a.InlierIndices.size() < b.InlierIndices.size();
        });
        return *it;
    }


public: /*attributes*/
    //    ///@brief Candidate that stores current canidiates for ransac processing. Needs to be attribute because
    //    parameters
    //    /// from derived classes may be stored here
    //    ConcreteCandidate PrototypeCandidate;

    int MinNumberSamples;
};
}
