#include "trackBox.h"

using namespace std;
using namespace cv;

TrackBoxParams::TrackBoxParams(){
  edgeboxes_max_num = 1;
  edgeboxes_max_dist_from_center = 1000;
  edgeboxes_min_box_area = 1000;
  edgeboxes_min_box_aspect_ratio = 3;
  favored_box_size = Rect(0, 0, 100, 100);
  maximum_box_size = Rect(0, 0, 150, 150);
  maximum_box_size_factor = 4;
}

TrackBox::TrackBox(){
  sedge_ = ximgproc::createStructuredEdgeDetection("../model.yml");
  edgeboxes_ = ximgproc::createEdgeBoxes();
  edgeboxes_->setMaxBoxes(params_.edgeboxes_max_num);
  targets_ = NULL; 
}

bool TrackBox::GetTrackBox(cv::Mat roi, GList **tracks){
  StructuredEdgeDetection(roi, roi);
}

bool TrackBox::GetTrackBoxes(cv::Mat roi, GList **tracks){
  StructuredEdgeDetection(roi, roi);
  *tracks = targets_;
}

void TrackBox::DetectFeatures(cv::Mat *roi){
}

void TrackBox::StructuredEdgeDetection(cv::InputArray src, cv::OutputArray dst){
  CV_Assert(src.isMat());

  // params_.maximum_box_size = Rect(0, 0, src.cols()/params_.maximum_box_size_factor, 
  //     src.rows()/params_.maximum_box_size_factor);

  cout << "src size: " << src.size().width <<  " " << src.size().height << endl;
  Mat rgb_im;
  cvtColor(src, rgb_im, COLOR_BGR2RGB);
  rgb_im.convertTo(rgb_im, CV_32F, 1.0 / 255.0f);
  Mat edge_im;

  // clear previous results
  boxes_.clear();
  targets_ = NULL;

  sedge_->detectEdges(rgb_im, edge_im);
  Mat orin;
  sedge_->computeOrientation(edge_im, orin);
  Mat edge_nms;
  sedge_->edgesNms(edge_im, orin, edge_nms);
  edgeboxes_->setMinScore(0.01);
  edgeboxes_->getBoundingBoxes(edge_nms, orin, boxes_);

  Point2f sigma_pos;
  Point2f sigma_sizzle;
  Point2f sigma_area;
  StandardDeviationRectVector(boxes_, sigma_pos, sigma_sizzle, sigma_area);

  Point2f pos_filter = sigma_pos * 2;
  Point2f sizzle_filter = sigma_sizzle * 2;
  Point2f area_filter = sigma_area * 2;

  // Put vector of boxes into a glist
  for(int i = 0; i < (int)boxes_.size(); i++){
    Target *new_target = new Target(boxes_[i], boxes_[i].tl(), 
        params_.maximum_box_size);
    targets_ = g_list_append(targets_, new_target);
  }

  int total_found = g_list_length(targets_);

  // remove oversize and overly intersected boxes
  GList *l = targets_;
  while (l != NULL){
    GList *next = l->next;
    Target *t = (Target*)l->data;
    CheckIntersections(l);
    if (t->IsOversize() || t->IsIntersected()){
      targets_ = g_list_delete_link (targets_, l);
    }
    l = next;
  }

  cout << "Total rois found: " << total_found << endl;
  cout << "Reduced rois: " << g_list_length(targets_) << endl;

  // list final boxes
  for(int i = 0; i < (int)g_list_length(targets_); i++){
    Target *t = (Target*)g_list_nth_data(targets_, i);
  }

  DrawViz(src.getMat());
  SaveVizImage(src.getMat());
}

void TrackBox::SaveVizImage(cv::Mat roi){
  imwrite("trackbox_last_run.jpg", viz_image_);
}

void TrackBox::DrawViz(cv::Mat roi){
  viz_image_ = roi.clone();
  for(int i = 0; i < (int)g_list_length(targets_); i++){
    Target* t = (Target*)g_list_nth_data(targets_, i);
    Point p1(t->GetROI().x, t->GetROI().y),
      p2(t->GetROI().x + t->GetROI().width, t->GetROI().y + t->GetROI().height);
    rectangle(viz_image_, p1, p2, Scalar(255, 0, 0), 2);
  }
}

void TrackBox::MeanPointVector(const std::vector<cv::Point2f> points, cv::Point2f &mean){
  for(int i = 0; i < (int)points.size(); i++){
    mean.x += points[i].x;
    mean.y += points[i].y;
  }
  mean.x = mean.x / points.size();
  mean.y = mean.y / points.size();
}

void TrackBox::StandardDeviationPointVector(std::vector<cv::Point2f> points, cv::Point2f &sigma){
  Point2f mean(0, 0);
  vector<Point2f> sigmav;

  // calculate mean of input point vector
  MeanPointVector(points, mean);

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

void TrackBox::StandardDeviationRectVector(std::vector<cv::Rect> &points, 
    cv::Point2f &sigma_pos, cv::Point2f &sigma_size, cv::Point2f &sigma_area){
  Rect mean(0, 0, 0, 0);

  // calculate box position std dev
  vector<Point2f> boxes_pos;
  for(int i = 0; i < boxes_.size(); i++){
    Point2f pos = boxes_[i].tl();
    boxes_pos.push_back(pos);
  }

  StandardDeviationPointVector(boxes_pos, sigma_pos);

  // calc std-dev of box sizes
  vector<Point2f> boxes_size;
  for(int i = 0; i < boxes_.size(); i++){
    Point2f size = Point2f(boxes_[i].width, boxes_[i].height);
    boxes_size.push_back(size);
  }

  StandardDeviationPointVector(boxes_size, sigma_size);

  // calc std-dev of box areas
  vector<Point2f> boxes_area;
  for(int i = 0; i < boxes_.size(); i++){
    Point2f area = Point2f(boxes_[i].area(), 0);
    boxes_area.push_back(area);
  }

  StandardDeviationPointVector(boxes_area, sigma_area);
}

bool TrackBox::CheckIntersections(GList *target){
  Target *tt = (Target*)target->data;

  GList *l = targets_;
  while (l != NULL){
    GList *next = l->next;
    Target *t = (Target*)l->data;
    cout << "checking intersection" << endl;
    int i_area = (tt->GetROI() & t->GetROI()).area();
    int t_area = t->GetROI().area();
    float cut = t_area * 0.1f;
    cout << "intersection area " << i_area << endl;
    if( i_area > 0 && i_area != t_area){
      cout << "intersection found" << endl;
      t->SetIntersected();
    }

    l = next;
  }
  cout << "iter chexk complete" << endl;
}