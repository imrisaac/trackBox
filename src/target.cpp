#include "target.h"
#include <iostream>
using namespace std;
using namespace cv;

Target::Target(){
  origin_ = Point(0, 0);
  roi_ = Rect(0, 0, 0, 0);
}

Target::Target(cv::Rect roi, cv::Point origin, cv::Rect max_size){
  origin_ = origin;
  roi_ = roi;
  max_size_ = max_size;
}

void Target::SetOrigin(cv::Point origin){
  origin_ = origin;
}

void Target::SetROI(cv::Rect roi){
  roi_ = roi;
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
