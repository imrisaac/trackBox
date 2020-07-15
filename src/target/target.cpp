#include "target.h"
#include <iostream>

using namespace std;
using namespace cv;

Target::Target(){
  Init(Rect(0, 0, 0, 0), Rect(0, 0, 0, 0), cv::Rect(99999, 99999, 99999, 99999));
}

Target::Target(cv::Rect roi, cv::Rect2d origin, cv::Rect max_size){
  Init(roi, origin, max_size);
}

void Target::Init(cv::Rect roi, cv::Rect2d origin, cv::Rect max_size){
  origin_ = origin;
  roi_ = roi;
  max_size_ = max_size;
  intersection_count_ = 0;
}

void Target::SetOrigin(cv::Rect2d origin){
  origin_ = origin;
}

void Target::SetROI(cv::Rect roi){
  roi_ = roi;
}

void Target::SetROIOffset(cv::Rect2d offset){
  roi_.x = roi_.x + offset.x;
  roi_.y = roi_.y + offset.y;
  origin_.x = origin_.x + offset.x;
  origin_.y = origin_.y + offset.y; 
}

void Target::SetMaxSize(cv::Rect max_size){
  max_size_ = max_size;
}

void Target::SetPlateScale(cv::Point2f plate_scale){
  plate_scale_ = plate_scale;
}

bool Target::IsOversize(){
  if(roi_.area() > max_size_.area()){
    return true;
  }
  return false;
}

bool Target::IsIntersected(){
  cout << "intersection count " << intersection_count_ << endl;
  if(intersection_count_ > 2){
    cout << "remove" << endl;
    return true;
  }
  return false;
}

bool Target::TargetTrackInit(cv::InputArray frame){
  return track_.Init(frame, roi_);
}

bool Target::TargetTrackUpdate(cv::InputArray frame){
  return track_.Update(frame, roi_);
}

void Target::DrawViz(cv::InputArray &frame){
  rectangle(frame.getMat(), roi_.tl(), roi_.br(), Scalar(255, 0, 0), 2);
}

bool Target::SetIntersected(){
  intersection_count_ ++;
}

cv::Rect2d Target::GetOrigin(){
  return origin_;
}

cv::Rect Target::GetROI(){
  return roi_;
}

cv::Rect Target::GetMaxSize(){
  return max_size_;
}

cv::Point Target::GetPixelDisplacement(){
  Point2d delta_pixel = origin_.tl() - roi_.tl();
  double euclid = norm(origin_.tl() - roi_.tl());
  return delta_pixel;
}

cv::Point2f Target::GetTargetPositionAngle(){
  Point2f delta_pixel = GetPixelDisplacement();
  Point2f delta_angle = Point2f(delta_pixel.x * plate_scale_.x, 
                                delta_pixel.y * plate_scale_.y);
  return delta_angle;
}