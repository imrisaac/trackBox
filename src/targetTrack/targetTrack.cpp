
#include "targetTrack.h"

using namespace std;
using namespace cv;

TargetTrack::TargetTrack(){
  is_init_ = false;
  tracker_ = TrackerCSRT::create();
}

bool TargetTrack::Init(cv::InputArray frame, const Rect2d &bounding_box){
  if(is_init_){
    return false;
  }

  if(tracker_ != NULL){
    return false;
  }

  if(frame.empty()){
    return false;
  }

  is_init_ = tracker_->init(frame.getMat(), bounding_box);
  return is_init_;
}

bool TargetTrack::Update(InputArray frame, Rect2d& boundingBox){
  if(!is_init_){
    return false;
  }
  if(frame.empty()){
    return false;
  }
  return tracker_->update(frame.getMat(), boundingBox);
}

