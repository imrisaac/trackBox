
#ifndef TARGETTRACK_H
#define TARGETTRACK_H

#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/tracking.hpp>
#include "opencv2/ximgproc/segmentation.hpp"
#include <opencv2/ximgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <glib.h>

class TargetTrack{
public:
  TargetTrack(cv::Ptr<cv::Tracker> tracker = NULL);
  bool Init(cv::InputArray frame, const cv::Rect2d &bounding_box);
  bool Update(cv::InputArray frame, cv::Rect2d &bounding_box);

private:
  cv::Ptr<cv::Tracker> tracker_;
  cv::Rect2d tracker_box_;
  cv::Point2d sensor_center_;

  bool is_init_;
};

#endif // TARGETTRACK_H