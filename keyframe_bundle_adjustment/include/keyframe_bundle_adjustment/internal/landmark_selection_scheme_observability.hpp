// Copyright 2017. All rights reserved.
// Institute of Measurement and Control Systems
// Karlsruhe Institute of Technology, Germany
//
// authors:
//  Johannes Graeter (johannes.graeter@kit.edu)
//  and others

#pragma once

#include "landmark_categorization_interface.hpp"
#include "landmark_selection_scheme_base.hpp"

namespace keyframe_bundle_adjustment {

/**
*  @class Landmark selection scheme observability
*  @par
*
* Concrete LandmarkSelectionScheme; select landmarks in function of their difference in angle.
* All angles of the viewing rays of the measruements between keyframes will be calculated and put in
* a histogram with 3 main bins.
* 3 bins are called: near field, middle field, far field and describe the observability of the
* measurements. Near features are good for translation, middle can be used for both rotation and
* translation and far away features are important for good rotation estimation.
* Take a fix number of measurements ineach of the bins, as defined in parameter struct.
*/
class LandmarkSparsificationSchemeObservability : public LandmarkSparsificationSchemeBase,
                                                  public LandmarkCategorizatonInterface {
public: // public classes/enums/types etc...
    struct BinParameters {
        unsigned int max_num_landmarks_near{300};   ///< maximum number of landmarks in near bin
        unsigned int max_num_landmarks_middle{300}; ///< maximum number of landmarks in middle bin
        unsigned int max_num_landmarks_far{300};    ///< maximum number of landmarks in far bin
        double bound_near_middle = 0.4;             ///< bound between near field and middle field
        double bound_middle_far = 0.2;              ///< bound between middle field and far field
    };

    struct Parameters {
        Parameters() {
            histogram_cache_size = 500;
        }

        int histogram_cache_size; ///< cache size of accumulator with which the histogram
                                  /// is calculated. Bins are determind by those
        /// measruemetns afterwards they are only assigned.
        BinParameters bin_params_;
    };

public: // public methods
    // default constructor
    LandmarkSparsificationSchemeObservability(Parameters p) : params_(p) {
        ;
    }

    // default destructor
    virtual ~LandmarkSparsificationSchemeObservability() = default;

    // default move
    LandmarkSparsificationSchemeObservability(LandmarkSparsificationSchemeObservability&& other) = default;
    LandmarkSparsificationSchemeObservability& operator=(LandmarkSparsificationSchemeObservability&& other) = default;

    // default copy
    LandmarkSparsificationSchemeObservability(const LandmarkSparsificationSchemeObservability& other) = default;
    LandmarkSparsificationSchemeObservability& operator=(const LandmarkSparsificationSchemeObservability& other) =
        default;

    std::set<LandmarkId> getSelection(const LandmarkMap& new_frame, const KeyframeMap& keyframes) const override;

    std::map<LandmarkId, LandmarkCategorizatonInterface::Category> getCategorizedSelection(
        const LandmarkMap& new_frame, const KeyframeMap& keyframes) const override;

    //////////////////////////////////////////////////
    /// \brief createConst, factory function for const pointer
    /// \return
    ///
    static ConstPtr createConst(Parameters p);

    //////////////////////////////////////////////////
    /// \brief create, factory function
    /// \return
    ///
    static Ptr create(Parameters p);


public:                 // attributes
    Parameters params_; ///< paramters
};
}
