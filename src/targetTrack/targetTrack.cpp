#include "targetTrack.h"

using namespace std;
using namespace cv;

TargetTrack::TargetTrack(cv::Ptr<cv::Tracker> tracker){
  // If not specified use the default tracker
  if(tracker == NULL){
    is_init_ = false;
    TrackerCSRT::Params params;
    params.use_gray = false;
    params.use_rgb = true;
    params.use_hog = true;
    params.template_size = 200;
    tracker_ = TrackerCSRT::create(params);
    //tracker_ = TrackerGOTURN::create();
  }else{
    tracker_ = tracker;
  }
}

bool TargetTrack::Init(cv::InputArray frame, const cv::Rect2d &bounding_box){
  if(is_init_){
    return false;
  } 

  if(frame.empty()){
    return false;
  }

  is_init_ = tracker_->init(frame.getMat(), bounding_box);
  return is_init_;
}

bool TargetTrack::Update(cv::InputArray frame, cv::Rect2d &bounding_box){
  if(!is_init_){
    return false;
  }
  if(frame.empty()){
    return false;
  }
  return tracker_->update(frame.getMat(), bounding_box);
}
