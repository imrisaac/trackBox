#include "motionDeblur.h"

using namespace cv;
using namespace std;

MotionDeblur::MotionDeblur(int len, double theta, int snr){
  len_ = len;
  theta_ = theta;
  snr_ = snr;
}

void MotionDeblur::Deblur(cv::InputArray in_frame, cv::OutputArray out_frame){
  if(in_frame.getMat().empty()){
    return;
  }
  Mat in_image = in_frame.getMat();

  Mat in_gray;
  cvtColor(in_image, in_gray, COLOR_RGB2GRAY );

  // it needs to process even image only
  Rect roi = Rect(0, 0, in_gray.cols & -2, in_gray.rows & -2);

  //Hw calculation (start)
  Mat Hw, h;
  CalcPSF(h, roi.size(), len_, theta_);
  CalcWnrFilter(h, Hw, 1.0 / double(snr_));

  in_gray.convertTo(in_gray, CV_32F);
  Edgetaper(in_gray, in_gray);

  Mat out_img;
  Filter2DFreq(in_gray(roi), out_img, Hw);
  out_img.convertTo(out_img, CV_8U);
  normalize(out_img, out_img, 0, 255, NORM_MINMAX);
  out_img.copyTo(out_frame);
  return;
}

void MotionDeblur::CalcPSF(cv::Mat& outputImg, cv::Size filterSize, int len, double theta){
  Mat h(filterSize, CV_32F, Scalar(0));
  Point point(filterSize.width / 2, filterSize.height / 2);
  ellipse(h, point, Size(0, cvRound(float(len) / 2.0)), 90.0 - theta, 0, 360, Scalar(255), FILLED);
  Scalar summa = sum(h);
  outputImg = h / summa[0];
}

void MotionDeblur::FFTShift(const cv::Mat& inputImg, cv::Mat& outputImg){
  outputImg = inputImg.clone();
  int cx = outputImg.cols / 2;
  int cy = outputImg.rows / 2;
  Mat q0(outputImg, Rect(0, 0, cx, cy));
  Mat q1(outputImg, Rect(cx, 0, cx, cy));
  Mat q2(outputImg, Rect(0, cy, cx, cy));
  Mat q3(outputImg, Rect(cx, cy, cx, cy));
  Mat tmp;
  q0.copyTo(tmp);
  q3.copyTo(q0);
  tmp.copyTo(q3);
  q1.copyTo(tmp);
  q2.copyTo(q1);
  tmp.copyTo(q2);
}

void MotionDeblur::Filter2DFreq(const cv::Mat& inputImg, cv::Mat& outputImg, const cv::Mat& H){
  Mat planes[2] = { Mat_<float>(inputImg.clone()), Mat::zeros(inputImg.size(), CV_32F) };
  Mat complexI;
  merge(planes, 2, complexI);
  dft(complexI, complexI, DFT_SCALE);

  Mat planesH[2] = { Mat_<float>(H.clone()), Mat::zeros(H.size(), CV_32F) };
  Mat complexH;
  merge(planesH, 2, complexH);
  Mat complexIH;
  mulSpectrums(complexI, complexH, complexIH, 0);

  idft(complexIH, complexIH);
  split(complexIH, planes);
  outputImg = planes[0];
}

void MotionDeblur::CalcWnrFilter(const cv::Mat& input_h_PSF, cv::Mat& output_G, double nsr){
  Mat h_PSF_shifted;
  FFTShift(input_h_PSF, h_PSF_shifted);
  Mat planes[2] = { Mat_<float>(h_PSF_shifted.clone()), Mat::zeros(h_PSF_shifted.size(), CV_32F) };
  Mat complexI;
  merge(planes, 2, complexI);
  dft(complexI, complexI);
  split(complexI, planes);
  Mat denom;
  pow(abs(planes[0]), 2, denom);
  denom += nsr;
  divide(planes[0], denom, output_G);
}

void MotionDeblur::Edgetaper(const cv::Mat& inputImg, cv::Mat& outputImg, double gamma, double beta){
  int Nx = inputImg.cols;
  int Ny = inputImg.rows;
  Mat w1(1, Nx, CV_32F, Scalar(0));
  Mat w2(Ny, 1, CV_32F, Scalar(0));

  float* p1 = w1.ptr<float>(0);
  float* p2 = w2.ptr<float>(0);
  float dx = float(2.0 * CV_PI / Nx);
  float x = float(-CV_PI);
  for (int i = 0; i < Nx; i++)
  {
      p1[i] = float(0.5 * (tanh((x + gamma / 2) / beta) - tanh((x - gamma / 2) / beta)));
      x += dx;
  }
  float dy = float(2.0 * CV_PI / Ny);
  float y = float(-CV_PI);
  for (int i = 0; i < Ny; i++)
  {
      p2[i] = float(0.5 * (tanh((y + gamma / 2) / beta) - tanh((y - gamma / 2) / beta)));
      y += dy;
  }
  Mat w = w2 * w1;
  multiply(inputImg, w, outputImg);
}
