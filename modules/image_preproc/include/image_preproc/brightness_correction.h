#pragma once

#include <opencv2/core/core.hpp>

namespace image_preproc {

class BrightnessCorrection {
public:
  static void correctGamma(cv::InputArray src, cv::OutputArray dst, const double gamma);
  static void stretchHist(cv::InputArray src, cv::OutputArray dst, const double percentage);

  static double computeOptimalGamma(cv::InputArray src);
private:
  // forbid object creation
  BrightnessCorrection();
  BrightnessCorrection(const BrightnessCorrection&);
};

} // namespace

