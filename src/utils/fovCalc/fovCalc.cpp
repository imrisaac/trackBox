//
//  fovCalc.cpp
//
//  Created by Isaac Reed on 4/1/20
//

#include "fovCalc.h"

using namespace std;

FOVCalc::FOVCalc(){

}

bool FOVCalc::SetSensorParams(int sensor_horizontal_pix, 
                              int sensor_vertical_pix, 
                              float sensor_horizontal_pixel_pitch, 
                              float sensor_vertical_pixel_pitch, 
                              float lens_focal_length){
  sensor_horizontal_pixels_ = sensor_horizontal_pix;
  sensor_vertical_pixels_ = sensor_vertical_pix;
  focal_length_ = lens_focal_length;
  sensor_vertical_pixel_pitch_ = sensor_horizontal_pixel_pitch;
  sensor_horizontal_pixel_pitch_ = sensor_vertical_pixel_pitch;

  CalcAllFOV();
  return true;
}

void FOVCalc::SetSensorFocalLength(float focal_length){
  if(focal_length_ != focal_length){
    focal_length_ = focal_length;
    CalcAllFOV();
  }
  return;
}

void FOVCalc::SetSensorHorizontalPixels(int pixels){
  if(sensor_diagonal_ != pixels){
    sensor_horizontal_ = pixels;
    CalcAllFOV();
  }
}

void FOVCalc::SetSensorVerticalPixels(int pixels){
  if(sensor_vertical_ != pixels){
    sensor_vertical_ = pixels;
    CalcAllFOV();
  }
}

void FOVCalc::SetSensorCropSize(int horizontal_pixels, int vertical_pixels){
  if(sensor_horizontal_pixels_ != horizontal_pixels || 
  sensor_vertical_pixels_ != vertical_pixels){
    sensor_horizontal_pixels_ = horizontal_pixels;
    sensor_vertical_pixels_ = vertical_pixels;
    CalcAllFOV();
  }
}

void FOVCalc::SetSensorHorizontalPixelPitch(float pitch){
  if(sensor_horizontal_pixel_pitch_ != pitch){
    sensor_horizontal_pixel_pitch_ = pitch;
    CalcAllFOV();
  }
}

void FOVCalc::SetSensorVerticalPixelPitch(float pitch){
  if(sensor_vertical_pixel_pitch_ != pitch){
    sensor_vertical_pixel_pitch_ = pitch;
    CalcAllFOV();
  }
}

void FOVCalc::CalcAllFOV(){
  sensor_horizontal_ = (sensor_horizontal_pixels_ * sensor_vertical_pixel_pitch_)/1000;
  sensor_vertical_ = (sensor_vertical_pixels_ * sensor_vertical_pixel_pitch_)/1000;
  sensor_diagonal_ = sqrt(pow(sensor_horizontal_, 2) + 
                          pow(sensor_vertical_, 2));
  CalcDiagonalFOV();
  CalcHorizontalFOV();
  CalcVerticalFOV();
  return;
}

float FOVCalc::GetSensorDiagonalSize(){
  return sensor_diagonal_;
}

float FOVCalc::GetSensorHorizontalSize(){
  return sensor_horizontal_;
}

float FOVCalc::GetSensorVerticalSize(){
  return sensor_vertical_;
}

float FOVCalc::GetDiagonalFOV(){
  return diagonal_fov_;
}

float FOVCalc::GetHorizontalFOV(){
  return horizontal_fov_;
}

float FOVCalc::GetVerticalFOV(){
  return vertical_fov_;
}

void FOVCalc::CalcDiagonalFOV(){
  diagonal_fov_ = 2 * atan((sensor_diagonal_ / 2) / focal_length_);
  diagonal_fov_ = diagonal_fov_ * 180/3.14156;
  diagonal_fov_ = (float)(int)(diagonal_fov_ * 100 + 0.5)/100;
  return;
}

void FOVCalc::CalcHorizontalFOV(){
  horizontal_fov_ = 2 * atan((sensor_horizontal_ / 2) / focal_length_);
  horizontal_fov_ = horizontal_fov_ * 180 / 3.14156;
  horizontal_fov_ = (float)(int)(horizontal_fov_ * 100 + 0.5) / 100;
  return;
}

void FOVCalc::CalcVerticalFOV(){
  vertical_fov_ = 2 * atan((sensor_vertical_ / 2) / focal_length_);
  vertical_fov_ = vertical_fov_ * 180 / 3.14156;
  vertical_fov_ = (float)(int)(vertical_fov_ * 100 + 0.5) / 100;
  return;
}