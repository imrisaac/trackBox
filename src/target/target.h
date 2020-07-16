
#ifndef TARGET_H
#define TARGET_H

#include <opencv2/core.hpp>
#include <opencv2/tracking.hpp>
#include "targetTrack.h"

class Target{
public:
  Target();
  Target(cv::Rect roi, cv::Rect2d origin, cv::Rect max_size = cv::Rect(99999, 99999, 99999, 99999));
  void SetOrigin(cv::Rect2d origin);
  void SetROI(cv::Rect roi);
  void SetROIOffset(cv::Rect2d offset);
  void SetMaxSize(cv::Rect);
  bool IsOversize();
  bool IsIntersected();
  bool SetIntersected();
  bool TargetTrackInit(cv::InputArray frame);
  bool TargetTrackUpdate(cv::InputArray frame);
  void DrawViz(cv::InputArray &frame);
  void SetPlateScale(cv::Point2f plate_scale);
  cv::Rect2d GetOrigin();
  cv::Rect GetROI();
  cv::Rect GetMaxSize();
  cv::Point GetPixelDisplacement();
  cv::Point2f GetTargetPositionAngle();
  
private:
  void Init(cv::Rect roi, cv::Rect2d origin, cv::Rect max_size);
  cv::Rect2d roi_;
  cv::Rect2d roi_offset_;
  cv::Rect2d origin_;
  cv::Rect max_size_;
  int max_intersection_area_;
  TargetTrack track_;
  int intersection_count_;
  cv::Point2f plate_scale_;
};

#endif // TARGET_H