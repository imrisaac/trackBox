
#ifndef MOTIONDEBLUR_H
#define MOTIONDEBLUR_H

#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

class MotionDeblur{
public:
  MotionDeblur(int len = 5, double theta = 0, int snr = 10);
  void Deblur(cv::InputArray in_frame, cv::OutputArray out_frame);

private:
  void CalcPSF(cv::Mat& outputImg, cv::Size filterSize, int len, double theta);
  void FFTShift(const cv::Mat& inputImg, cv::Mat& outputImg);
  void Filter2DFreq(const cv::Mat& inputImg, cv::Mat& outputImg, const cv::Mat& H);
  void CalcWnrFilter(const cv::Mat& input_h_PSF, cv::Mat& output_G, double nsr);
  void Edgetaper(const cv::Mat& inputImg, cv::Mat& outputImg, double gamma = 5.0, double beta = 0.2);
  int len_;
  double theta_;
  int snr_;
};

#endif // MOTIONDEBLUR_H