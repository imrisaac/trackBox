#include "target.h"
#include <iostream>
using namespace std;
using namespace cv;

Target::Target(){
  Init(Rect(0, 0, 0, 0), Point(0, 0), cv::Rect(99999, 99999, 99999, 99999));
}

Target::Target(cv::Rect roi, cv::Point origin, cv::Rect max_size){
  Init(roi, origin, max_size);
}

void Target::Init(cv::Rect roi, cv::Point origin, cv::Rect max_size){
  origin_ = origin;
  roi_ = roi;
  max_size_ = max_size;
  intersection_count_ = 0;
}

void Target::SetOrigin(cv::Point origin){
  origin_ = origin;
}

void Target::SetROI(cv::Rect roi){
  roi_ = roi;
}

void Target::SetROIOffset(cv::Rect2d offset){
  roi_.x = roi_.x + offset.x;
  roi_.y = roi_.y + offset.y; 
}

void Target::SetMaxSize(cv::Rect max_size){
  max_size_ = max_size;
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

cv::Point Target::GetOrigin(){
  return origin_;
}

cv::Rect Target::GetROI(){
  return roi_;
}

cv::Rect Target::GetMaxSize(){
  return max_size_;
}
