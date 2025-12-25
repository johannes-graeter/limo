#pragma once

// Manages the parameters for the processors.
namespace image_preproc {

struct GammaCorrectorParams {
    double gamma = 1.0;
    bool auto_gamma = false;
};

}