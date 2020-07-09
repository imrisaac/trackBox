
#ifndef TARGET_H
#define TARGET_H

#include "opencv2/core.hpp"
#include <opencv2/tracking.hpp>
#include "targetTrack.h"

class Target{
public:
  Target();
  Target(cv::Rect roi, cv::Point origin, cv::Rect max_size = cv::Rect(99999, 99999, 99999, 99999));
  void SetOrigin(cv::Point origin);
  void SetROI(cv::Rect roi);
  void SetMaxSize(cv::Rect);
  bool IsOversize();
  bool IsIntersected();
  cv::Point GetOrigin();
  cv::Rect GetROI();
  cv::Rect GetMaxSize();
  
private:
  cv::Rect roi_;
  cv::Point origin_;
  cv::Rect max_size_;
  int max_intersection_area_;
  TargetTrack track_;

};

#endif // TARGET_H