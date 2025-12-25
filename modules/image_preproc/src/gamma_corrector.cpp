#include <image_preproc/brightness_correction.h>
#include <image_preproc/gamma_corrector.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace image_preproc {
GammaCorrector::GammaCorrector() {}

void GammaCorrector::processImage(const cv::Mat& src, cv::OutputArray dst) {
  if (params_.auto_gamma) {
    cv::Mat img;
    if (src.channels() == 3) {
      cv::cvtColor(src, img, cv::COLOR_BGR2GRAY);
    } else if (src.channels() == 1) {
      img = src;
    }
    double gamma =
        image_preproc::BrightnessCorrection::computeOptimalGamma(img);
    image_preproc::BrightnessCorrection::correctGamma(src, dst,
                                                      gamma * params_.gamma);

  } else {
    image_preproc::BrightnessCorrection::correctGamma(src, dst, params_.gamma);
  }
}

}  // namespace image_preproc