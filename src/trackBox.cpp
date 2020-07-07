#include "trackBox.h"

using namespace std;
using namespace cv;

TrackBoxParams::TrackBoxParams(){
  edgeboxes_max_num = 25;
  edgeboxes_max_dist_from_center = 1000;
  edgeboxes_min_box_area = 1000;
  edgeboxes_min_box_aspect_ratio = 3;
}

TrackBox::TrackBox(){
  sedge_ = ximgproc::createStructuredEdgeDetection("../model.yml");
  edgeboxes_ = ximgproc::createEdgeBoxes();
}

bool TrackBox::GetTrackBox(cv::Mat roi){
  StructuredEdgeDetection(roi, roi);
}

bool TrackBox::GetTrackBoxes(cv::Mat roi){
  StructuredEdgeDetection(roi, roi);
}

void TrackBox::DetectFeatures(cv::Mat *roi){
}

void TrackBox::StructuredEdgeDetection(cv::InputArray src, cv::OutputArray dst){
  // CV_Assert(src.isMat());
  // Mat src_mat = src.getMat();
  // CV_Assert( src.type() == CV_32FC2);
  cout << src.size().width/2 <<  " " << src.size().height/2 << endl;
  Mat rgb_im;
  cvtColor(src, rgb_im, COLOR_BGR2RGB);
  rgb_im.convertTo(rgb_im, CV_32F, 1.0 / 255.0f);
  Mat edge_im;
  cout << "run sedge" << endl;
  sedge_->detectEdges(rgb_im, edge_im);
  Mat orin;
  cout << "compute orientation" << endl;
  sedge_->computeOrientation(edge_im, orin);
  Mat edge_nms;
  sedge_->edgesNms(edge_im, orin, edge_nms);
  cout << "compute edgeboxes" << endl;
  edgeboxes_->setMaxBoxes(params_.edgeboxes_max_num);
  edgeboxes_->getBoundingBoxes(edge_nms, orin, boxes_);
  Point2d roi_center = Point2d(src.size().width/2, src.size().height/2);

  Point2f sigma_center;
  Point2f sigma_sizzle;
  cout << "compute standard dev" << endl;
  StandardDeviationRectVector(boxes_, sigma_center, sigma_sizzle);

  Point2f pos_filter = sigma_center * 2;

  // Position filter
  cout << "filter boxes" << endl;
  for(int i = 0; i < (int)boxes_.size(); i++){
    if(boxes_[i].x > pos_filter.x || boxes_[i].y > pos_filter.y){
      boxes_.erase(boxes_.begin() + i -1);
    }
  }
  cout << "Total rois found: " << boxes_.size() << endl;
  DrawViz(src.getMat());
  SaveVizImage(src.getMat());
}

void TrackBox::SaveVizImage(cv::Mat roi){
  imwrite("trackbox_last_run.jpg", viz_image_);
}

void TrackBox::DrawViz(cv::Mat roi){
  viz_image_ = roi.clone();
  for(int i = 0; i < (int)boxes_.size(); i++){
    Point p1(boxes_[i].x, boxes_[i].y), p2(boxes_[i].x + boxes_[i].width, boxes_[i].y + boxes_[i].height);
    rectangle(viz_image_, p1, p2, Scalar(255, 0, 0), 2);
  }
}

void TrackBox::MeanPointVector(const std::vector<cv::Point2f> points, cv::Point2f mean){
  cout << "calculate vector mean" << endl;
  for(int i = 0; i < (int)points.size(); i++){
    mean.x += points[i].x;
    mean.y += points[i].y;
  }
  mean.x = mean.x / points.size();
  mean.y = mean.y / points.size();
}

void TrackBox::StandardDeviationPointVector(std::vector<cv::Point2f> points, cv::Point2f sigma){
  Point2f mean(0, 0);
  vector<Point2f> sigmav;

  // calculate mean of input point vector
  MeanPointVector(points, mean);
  cout << "calculate sum" << endl;
  // calculate the sum(xi -u)^2
  for(int i = 0; i < (int)points.size(); i++){
    Point2f sum = Point2f((points[i].x - mean.x), (points[i].y - mean.y));
    sigmav.push_back(Point2f(pow(sum.x, 2), pow(sum.y, 2)));
  }

  // calculate the mean of the sum
  MeanPointVector(sigmav, sigma);

  // calculate standard dev
  sigma = Point2f(sqrt(sigma.x), sqrt(sigma.y));
}

void TrackBox::StandardDeviationRectVector(std::vector<cv::Rect> points, 
    cv::Point2f sigma_center, cv::Point2f sigma_size){
  Rect mean(0, 0, 0, 0);

  // calculate box centers and calc the std-dev of their position
  vector<Point2f> boxes_center;
  for(int i = 0; i < boxes_.size(); i++){
    Point2f center = (boxes_[i].tl() + boxes_[i].br()) * 0.5;
    boxes_center.push_back(center);
  }
  cout << "calculate box center std dev" << endl;
  StandardDeviationPointVector(boxes_center, sigma_center);

  // calc std-dev of box sizes
  vector<Point2f> boxes_size;
  for(int i = 0; i < boxes_.size(); i++){
    Point2f center = Point2f(boxes_[i].width, boxes_[i].height);
    boxes_size.push_back(center);
  }
  cout << "calculate box size std dev" << endl;
  StandardDeviationPointVector(boxes_size, sigma_size);

  // TODO: Area perhaps
  //rect.area
}