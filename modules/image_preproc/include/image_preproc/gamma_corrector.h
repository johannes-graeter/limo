#pragma once

#include <opencv2/core/core.hpp>
#include <image_preproc/image_preproc_params.h>

namespace image_preproc {

class GammaCorrector {
public:
  GammaCorrector();
  void processImage(const cv::Mat& src, cv::OutputArray dst);
  inline void setParams(const GammaCorrectorParams& params) {params_ = params;};

private:
  GammaCorrectorParams params_;
};

} // namespace
