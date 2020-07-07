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

#include "trackBox.h"

using namespace std;
using namespace cv;

enum SourceType{
  JPG_IMG,
  PNG_IMG,
  MP4_VID,
  V4L2_VID,
  REALSENSE
};

Rect selection(0, 0, 0, 0);
Point selection_origin(0, 0);
int new_selection = 0;
bool selectObject = false;

std::string GetCameraPipeline(){
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

  pipeline = "v4l2src ! " 
            "videoconvert ! " 
            "video/x-raw, " 
                "format=BGR ! " 
            "appsink ";

  return pipeline;
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
  TrackBox trackBox;

  enum SourceType source_type = V4L2_VID;

  opterr = 0;

  while ((c = getopt (argc, argv, "i:bc:")) != -1){
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
      break;  
    case V4L2_VID:
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

  while(1){

  // frame capture
  switch(source_type){
    case JPG_IMG:
    case PNG_IMG:
      break;
    case MP4_VID:
      break;  
    case V4L2_VID:
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
        trackBox.GetTrackBox(input_frame(selection));
        new_selection = 1;
      }

    }

    if( selectObject && selection.area() > 5){
      Mat roi(input_frame, selection);
      bitwise_not(roi, roi);
    }

    imshow("TrackBox", input_frame);
    waitKey(33);
  }


  cout << "end" << endl;
  return 0;
}