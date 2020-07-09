
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp" // for imwrite apparently
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc/segmentation.hpp"
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include "target.h"
#include <glib.h>

class TrackBoxParams{
public:
  TrackBoxParams();
  int edgeboxes_max_num;
  int edgeboxes_max_dist_from_center;
  int edgeboxes_min_box_area;
  int edgeboxes_min_box_aspect_ratio;
  cv::Rect favored_box_size;
  cv::Rect maximum_box_size;
  int maximum_box_size_factor;
};

class TrackBox{
public:

  TrackBox();

  // Find the most suitable roi for tracking within the given frame
  bool GetTrackBox(cv::Mat roi, GList **tracks); 

  // Find the a selection of roi's for tracking within the given frame
  bool GetTrackBoxes(cv::Mat roi, GList **tracks);

  // Get the roi visualization
cv::Mat GetViz();

private:
  void DetectFeatures(cv::Mat *roi);
  void StructuredEdgeDetection(cv::InputArray src, cv::OutputArray dst);

  // Make a copy of the roi mat and draw vizualization on it 
  void DrawViz(cv::Mat roi);

  // Save the draw viz image to disk
  void SaveVizImage(cv::Mat roi);

  // Calculate the mean of a vector of points
  void MeanPointVector(const std::vector<cv::Point2f> points, cv::Point2f &mean);

  // Calculate the standard deviation of a vector of points
  void StandardDeviationPointVector(std::vector<cv::Point2f> points, cv::Point2f &sigma);

  // Calculate the standard deviation of a vector of Rect's
  void StandardDeviationRectVector(std::vector<cv::Rect> &points, 
      cv::Point2f &sigma_pos, cv::Point2f &sigma_size, cv::Point2f &sigma_area);
  
  TrackBoxParams params_;
  cv::Mat rgb_frame_;
  cv::Mat gray_frame_;
  cv::Mat viz_image_;
  cv::Ptr<cv::ximgproc::StructuredEdgeDetection> sedge_;
  cv::Ptr<cv::ximgproc::EdgeBoxes> edgeboxes_;
  std::vector<cv::Rect> boxes_;
  GList *targets_;
};
