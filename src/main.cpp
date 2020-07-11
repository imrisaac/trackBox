#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

#define WITH_REALSENSE
#ifdef WITH_REALSENSE
#include <librealsense2/rs.hpp>
#endif

#include "target.h"
#include "targetTrack.h"
#include "trackBox.h"
#include "motionDeblur.h"
#include "lowPassFilter.hpp"

#include <glib.h>
#include <algorithm>

using namespace std;
using namespace cv;

enum SourceType{
  JPG_IMG,
  PNG_IMG,
  MP4_VID,
  V4L2_VID,
  REALSENSE,
  IMAGE_SEQUENCE,
  VIDEO_SEQUENCE
};

Rect selection(0, 0, 0, 0);
Point selection_origin(0, 0);
int new_selection = 0;
bool selectObject = false;
MotionDeblur motionDeblur;

// kf
int kf_stateSize = 6;
int kf_measSize = 4;
int kf_contrSize = 0;
unsigned int kf_type = CV_32F;
KalmanFilter kf(kf_stateSize, kf_measSize, 0, kf_type);
cv::Mat state(kf_stateSize, 1, kf_type);  // [x,y,v_x,v_y,w,h]
cv::Mat meas(kf_measSize, 1, kf_type);    // [z_x,z_y,z_w,z_h]

std::string GetCameraPipeline(std::string video_device = "/dev/video0"){
  string pipeline;
  // pipeline = "nvarguscamerasrc ! " 
  //           "video/x-raw(memory:NVMM), " 
  //               "width=(int)" + std::to_string(960) + ", " 
  //               "height=(int)" + std::to_string(720) + ", " 
  //               "format=(string)NV12, " 
  //               "framerate=(fraction)" + std::to_string(30) + "/1 ! " 
  //           "nvvidconv flip-method=0 ! " 
  //           "video/x-raw, format=(string)BGRx ! " 
  //           "videoconvert ! " 
  //           "video/x-raw, " 
  //               "format=(string)BGR ! " 
  //           "appsink ";

  pipeline = "v4l2src "
                "device="+ video_device +" ! "
             "videoconvert ! " 
             "video/x-raw, " 
                 "format=BGR ! " 
             "appsink ";

  return pipeline;
}

void GetNextSequenceImage(){
  
}

static void onMouse(int event, int x, int y, int, void *user_data){
  Mat *image = (Mat*)user_data;
  if( selectObject ){
    selection.x = MIN(x, selection_origin.x);
    selection.y = MIN(y, selection_origin.y);
    selection.width = std::abs(x - selection_origin.x);
    selection.height = std::abs(y - selection_origin.y);
    selection &= Rect(0, 0, image->cols, image->rows);
  }

  switch( event ){
  case EVENT_LBUTTONDOWN:
    selection_origin = Point(x,y);
    selection = Rect(x,y,0,0);
    selectObject = true;
    break;

  case EVENT_LBUTTONUP:
    selectObject = false;

    if( selection.width > 0 && selection.height > 0 ){
      new_selection = -1;   // Set up CAMShift properties in main() loop
    }

    break;
  }
}

int main(int argc, char **argv){
  int input_type_flag = 0;
  char *input_type_value = NULL;
  int bflag = 0;
  char *cvalue = NULL;
  int index;
  int c;
  bool mirror = false;
  TrackBox trackBox;
  GList *targets = NULL;
  LowPassFilterInt lpf_x(0.01, 30);
  LowPassFilterInt lpf_y(0.01, 30);

  // Transition state matrix A
  cv::setIdentity(kf.transitionMatrix);

  // measure matrix H
  kf.measurementMatrix = cv::Mat::zeros(kf_measSize, kf_stateSize, kf_type);
  kf.measurementMatrix.at<float>(0) = 1.0f;
  kf.measurementMatrix.at<float>(7) = 1.0f;
  kf.measurementMatrix.at<float>(16) = 1.0f;
  kf.measurementMatrix.at<float>(23) = 1.0f;

  // process noise coveriance matrix Q
  kf.processNoiseCov.at<float>(0) = 1e-2;
  kf.processNoiseCov.at<float>(7) = 1e-2;
  kf.processNoiseCov.at<float>(14) = 5.0f;
  kf.processNoiseCov.at<float>(21) = 5.0f;
  kf.processNoiseCov.at<float>(28) = 1e-2;
  kf.processNoiseCov.at<float>(35) = 1e-2;

  // Measures Noise Covariance Matrix R
  cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

  enum SourceType source_type = V4L2_VID;

  opterr = 0;

  while ((c = getopt (argc, argv, "i:bc:m")) != -1){
    switch(c){
      case 'i':
        input_type_flag = 1;
        input_type_value = optarg;
        break;
      case 'b':
        bflag = 1;
        break;
      case 'c':
        cvalue = optarg;
        break;
      case 'm':
        mirror = true;
        break;
      case '?':
        break;
      default:
        break;
    }
  }
  // printf ("input_type_flag = %d, bflag = %d, input_type_value = %s\n",
  //         input_type_flag, bflag, input_type_value);

  for (index = optind; index < argc; index++)
    printf ("Non-option argument %s\n", argv[index]);

  if(input_type_flag){
    string input_type(input_type_value);
    string input_key(".jpg");
    size_t found = input_type.find(input_key);
    if(found != string::npos){
      cout << "input type detected as jpg image" << endl;
      source_type = JPG_IMG;
    }
    input_key = string(".png");
    found = input_type.find(input_key);
    if(found != string::npos){
      cout << "input type detected as png image" << endl;
      source_type = PNG_IMG;
    }
    input_key = string(".mp4");
    found = input_type.find(input_key);
    if(found != string::npos){
      cout << "input type detected as mp4 video" << endl;
      source_type = MP4_VID;
    }
    input_key = string("/dev/video");
    found = input_type.find(input_key);
    if(found != string::npos){
      cout << "input type detected as v4l2 video device" << endl;
      source_type = V4L2_VID;
    }
    input_key = string("realsense");
    found = input_type.find(input_key);
    if(found != string::npos){
      cout << "input type detected as realsense camera" << endl;
      source_type = REALSENSE;
    }
  }

  // Input initilization
  VideoCapture cap;
  Mat input_frame;

#ifdef WITH_REALSENSE
  rs2::colorizer color_map;
  rs2::pipeline pipe;
  rs2::config cfg;
  rs2::frameset frames;
#endif

  cout << "configuring input type " << source_type << endl;
  switch(source_type){
    case JPG_IMG:
    case PNG_IMG:
      input_frame = imread(input_type_value);
      break;
    case MP4_VID:
      cap.open(input_type_value);
      break;  
    case V4L2_VID:
      cap.open(GetCameraPipeline(input_type_value));
      if(!cap.isOpened()){
        cout << "Failed to open video capture pipeline" << endl;
      }
      break;
    case REALSENSE:
#ifdef WITH_REALSENSE
      cout << "configuring realsense input" << endl;
      cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
      pipe.start(cfg);
      // Camera warmup - dropping several first frames to let auto-exposure stabilize
      for(int i = 0; i < 30; i++){
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
      }
#endif
      break;
    case IMAGE_SEQUENCE:
    case VIDEO_SEQUENCE:
    default:
      break;
  }

  int w, h;
  #ifdef WITH_REALSENSE
    rs2::frameset data;
    rs2::frame depth;
  #endif
  namedWindow("TrackBox", 0);

  setMouseCallback("TrackBox", onMouse, &input_frame);

  Mat selection_mask = Mat::zeros(200, 320, CV_8UC3);

  double ticks = 0;


  while(1){

    double precTick = ticks;
    ticks = (double) cv::getTickCount();

    double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

    // frame capture
    switch(source_type){
      case JPG_IMG:
      case PNG_IMG:
        break;
      case MP4_VID:
        cap >> input_frame;
        break;  
      case V4L2_VID:
        cap >> input_frame;
        break;
      case REALSENSE:
  #ifdef WITH_REALSENSE
        data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        depth = data.get_color_frame().apply_filter(color_map);

        // Query frame size (width and height)
        w = depth.as<rs2::video_frame>().get_width();
        h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        input_frame =  Mat(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
  #endif
        break;
      default:
        break;
    }

    if(input_frame.empty()){
      cout << "empty frame" << endl;
      break;
    }

    // mirror for usability
    if(mirror){
      flip(input_frame, input_frame, +1);
    }

    // selection viz
    if(new_selection){
      
      Mat hsv, hue = Mat::zeros(input_frame.cols, input_frame.rows, CV_8UC3);
      cvtColor(input_frame, hsv, COLOR_BGR2HSV);
      int _vmin = 30, _vmax = 30;
      inRange(hsv, Scalar(0, 30, MIN(_vmin,_vmax)),
        Scalar(180, 256, MAX(_vmin, _vmax)), selection_mask);
      hue.create(hsv.size(), hsv.depth());
      int ch[] = {0, 0};
      mixChannels(&hsv, 1, &hue, 1, ch, 1);

      if(new_selection < 0){
        Mat roi(hue, selection), mask_roi(selection_mask, selection);
        trackBox.GetTrackBoxes(input_frame(selection), &targets);
        cout << "starting new tracks " << g_list_length(targets) << endl;
        Rect2d roi_offset = Rect2d(selection.x, selection.y, 0, 0);
        GList *li;
        for(li = targets; li != NULL; li = li->next){
          Target *target  = (Target*)li->data;
          target->SetROIOffset(roi_offset);
          cout << "track init: " << target->TargetTrackInit(input_frame) << endl;
        }
        new_selection = 1;

        kf.errorCovPre.at<float>(0) = 1; // px
        kf.errorCovPre.at<float>(7) = 1; // px
        kf.errorCovPre.at<float>(14) = 1;
        kf.errorCovPre.at<float>(21) = 1;
        kf.errorCovPre.at<float>(28) = 1; // px
        kf.errorCovPre.at<float>(35) = 1; // px

        state.at<float>(0) = meas.at<float>(0);
        state.at<float>(1) = meas.at<float>(1);
        state.at<float>(2) = 0;
        state.at<float>(3) = 0;
        state.at<float>(4) = meas.at<float>(2);
        state.at<float>(5) = meas.at<float>(3);
        kf.statePost = state;

      }

    }

    if( selectObject && selection.area() > 5){
      Mat roi(input_frame, selection);
      bitwise_not(roi, roi);
    }

    GList *li;
    for(li = targets; li != NULL; li = li->next){
      Target *target  = (Target*)li->data;
      target->TargetTrackUpdate(input_frame);
      Rect2d target_roi = target->GetROI();

      target_roi.x = lpf_x.apply(target_roi.x);
      target_roi.y = lpf_y.apply(target_roi.y);

      if(target_roi.x + target_roi.width < input_frame.cols
          && target_roi.x > 0 
          && target_roi.y + target_roi.height < input_frame.rows
          && target_roi.y > 0){
        Mat target_crop = input_frame(target_roi);
        //motionDeblur.Deblur(target_crop, target_crop);
        resize(target_crop, target_crop, Size(), 4.0, 4.0, INTER_CUBIC);
        imshow("Target", target_crop);
      }

      target->DrawViz(input_frame);

      kf.transitionMatrix.at<float>(2) = dT;
      kf.transitionMatrix.at<float>(9) = dT;
      state = kf.predict();

      cv::Point center;
      center.x = state.at<float>(0);
      center.y = state.at<float>(1);

      cv::Rect predRect;
      predRect.width = state.at<float>(4);
      predRect.height = state.at<float>(5);
      predRect.x = state.at<float>(0) - predRect.width / 2;
      predRect.y = state.at<float>(1) - predRect.height / 2;
      cv::rectangle(input_frame, predRect, CV_RGB(255,0,0), 2);

      meas.at<float>(0) = target_roi.x + target_roi.width / 2;
      meas.at<float>(1) = target_roi.y + target_roi.height / 2;
      meas.at<float>(2) = (float)target_roi.width;
      meas.at<float>(3) = (float)target_roi.height;

      kf.correct(meas); // Kalman Correction
    }
    
    imshow("TrackBox", input_frame);
    waitKey(33);
  }

  cout << "end" << endl;
  return 0;
}