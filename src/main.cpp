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
#include <opencv2/viz/types.hpp>
#include <opencv2/viz/vizcore.hpp>
#include "target.h"
#include "targetTrack.h"
#include "trackBox.h"
#include "motionDeblur.h"
#include "lowPassFilter.hpp"

#include <glib.h>
#include <algorithm>

#define WITH_REALSENSE
#ifdef WITH_REALSENSE
#include <librealsense2/rs.hpp>
#endif

#define WITH_ROS
#ifdef WITH_ROS
#include <ros/ros.h>
#endif

#define ARC_SECS_PER_RADIAN 206265

using namespace std;
using namespace cv;

enum SourceType{
  JPG_IMG,
  PNG_IMG,
  MP4_VID,
  V4L2_VID,
  REALSENSE,
  IMAGE_SEQUENCE,
  VIDEO_SEQUENCE,
  UDP_GAZEBO
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
  LowPassFilterInt lpf_x(2.01, 30);
  LowPassFilterInt lpf_y(2.01, 30);

#ifdef WITH_ROS
  // ros::init(0, 0, "trackBox");
  // ros::NodeHandle nh;
#endif

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
    input_key = string("udp");
    found = input_type.find(input_key);
    if(found != string::npos){
      cout << "input type detected as udp gazebo" << endl;
      source_type = UDP_GAZEBO;
    }    
  }

  // Input initilization
  VideoCapture cap;
  Mat input_frame;

  float sensor_pixel_pitch_x;
  float sensor_pixel_pitch_y;
  float lens_focal_length;
  Point2f plate_scale;

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
      // binned mode, pixels are double size
      sensor_pixel_pitch_x = 1.4 * 2;
      sensor_pixel_pitch_y = sensor_pixel_pitch_x;
      lens_focal_length = 1.88;
      plate_scale = Point2f((ARC_SECS_PER_RADIAN * sensor_pixel_pitch_x) / lens_focal_length,
                            (ARC_SECS_PER_RADIAN * sensor_pixel_pitch_x) / lens_focal_length);
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
      // half res 2x2 binned
      cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
      pipe.start(cfg);
      // Camera warmup - dropping several first frames to let auto-exposure stabilize
      for(int i = 0; i < 30; i++){
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
      }
      // binned mode, pixels are double size
      sensor_pixel_pitch_x = 1.4 * 2;
      sensor_pixel_pitch_y = sensor_pixel_pitch_x;
      lens_focal_length = 1.88;
      plate_scale = Point2f((ARC_SECS_PER_RADIAN * sensor_pixel_pitch_x) / lens_focal_length,
                            (ARC_SECS_PER_RADIAN * sensor_pixel_pitch_x) / lens_focal_length);
#endif
      break;
    case IMAGE_SEQUENCE:
    case VIDEO_SEQUENCE:
      break;
    case UDP_GAZEBO:
      cap.open("udpsrc port=5600 ! "
               "application/x-rtp ! "
               "rtph264depay ! avdec_h264 ! "
               "videoconvert ! video/x-raw ! " 
               "appsink sync=true async=false drop=true");
      break;
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

  viz::Viz3d myWindow("Coordinate Frame");
  // the origin visulization if you will
  //myWindow.showWidget("Coordinate Frame", viz::WCoordinateSystem());
  Vec3f cam_pos(3.0f,3.0f,3.0f); 
  Vec3f cam_focal_point(0.0f,0.0f,0.0f); 
  Vec3f cam_y_dir(1.0f,0.0f,0.0f);
  Affine3f cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
  Affine3f transform = viz::makeTransformToGlobal(Vec3f(0.0f,-1.0f,0.0f),
                       Vec3f(-1.0f,0.0f,0.0f), Vec3f(0.0f,0.0f,-1.0f), cam_pos);

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
      case UDP_GAZEBO:
        cap >> input_frame;
        break;
      default:
        break;
    }

    if(input_frame.empty()){
      cout << "empty frame" << endl;
      break;
    }

   // imshow("TrackBox", input_frame);

    resize(input_frame, input_frame, Size(), 0.5, 0.5, INTER_CUBIC);

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
          target->SetPlateScale(plate_scale);
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

    Vec3f target_position_angle(0, 0, 0);

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
        cv::rectangle(input_frame, target->GetOrigin(), CV_RGB(0,255,0), 2);
        Point2f angle_delta = target->GetTargetPositionAngle();
        angle_delta = angle_delta/ARC_SECS_PER_RADIAN/1000;
        cout << "Position Angle " << angle_delta << endl;
        // motion compensation
        cam_pose = Vec3f(-angle_delta.y, angle_delta.x, 0);
        
        target->DrawViz(input_frame);
      }

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

      viz::WCameraPosition cpw(0.5); // Coordinate axes
      viz::WCameraPosition cpw_frustum(Vec2f(69.4 * PI / 180, 42.5 * PI / 180), input_frame); // Camera frustum

      myWindow.showWidget("CPW", cpw, cam_pose);
      myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
      myWindow.spinOnce(1, true);

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