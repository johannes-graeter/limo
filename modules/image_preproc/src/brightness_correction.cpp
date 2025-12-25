#include "brightness_correction.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
namespace image_preproc {

void BrightnessCorrection::correctGamma(cv::InputArray src, cv::OutputArray dst, const double gamma) {
  cv::Mat gammaLUT(1, 256, CV_8UC1 );
  uchar * ptr = gammaLUT.ptr();
  for( int i = 0; i < 256; i++ )
  {
    ptr[i] = (int)( pow( (double) i / 255.0, gamma) * 255.0 );
  }

  cv::LUT( src, gammaLUT, dst );
}

double BrightnessCorrection::computeOptimalGamma(cv::InputArray src) {
  // check image
  // TODO
  cv::Mat img = src.getMat().clone();


  // compute hist
  cv::Mat hist;
  int histSize = 256;
  cv::calcHist(&img, 1, 0, cv::Mat(), hist, 1, &histSize, 0);

  std::vector<double> histCorrect(histSize);
  std::vector<double> cdfCorrect(histSize-3);
  // compute gamma
  double gamma = 1.;
  double delta = 1.;
  int steps = 0;

  // compute new hist
  std::fill(histCorrect.begin(), histCorrect.end(), 0);
  for(int i = 0; i < histSize; i++) {
    //double correctedVal = std::pow(double(i)/255., gamma)*255.;
    histCorrect[i] += hist.at<float>(i);
  }
  // compute cdf
  cdfCorrect[0] = histCorrect[1];
  for(int i = 2; i < histSize-2; i++) {
    cdfCorrect[i-1] = cdfCorrect[i-2] + histCorrect[i+1];
  }

  for(uint32_t i = 0; i < cdfCorrect.size(); i++) {
    cdfCorrect[i] = cdfCorrect[i]/cdfCorrect.back();
  }

  while(std::abs(delta) > 1e-4 || steps > 50) {
    double Jsq = 0.;
    double Je = 0.;
    for(uint32_t i = 0; i < cdfCorrect.size();i++) {
      double thisVal = double(i+2)/255.;
      double thisValRemapped = std::pow(thisVal, gamma);

      double thisJ = std::log(thisVal)*thisValRemapped;
      Jsq += thisJ*thisJ;
      Je += thisJ*(thisValRemapped - cdfCorrect[i]);
    }

    delta = Je/2./Jsq;

    gamma -= delta;
    steps++;
  }

  return gamma;
}

void BrightnessCorrection::stretchHist(cv::InputArray src, cv::OutputArray dst, const double percentage) {
  // check image
  // TODO
  cv::Mat img = src.getMat().clone();

  // compute hist
  cv::Mat hist;
  int histSize = 256;
  cv::calcHist(&img, 1, 0, cv::Mat(), hist, 1, &histSize, 0);

  double size = img.rows*img.cols;
  double cumsum = 0;
  double limit = percentage*size;
  int index = 0;
  while(cumsum < limit && index < histSize) {
    cumsum += hist.at<float>(index++);
  }

  dst.getMatRef() = img.clone();
  dst.getMatRef() = dst.getMatRef()*255./double(index-1);
}


}
