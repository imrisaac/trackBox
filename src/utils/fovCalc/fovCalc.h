//
//  fovCalc.h
//
//  Created by Isaac Reed on 4/1/20
//


#ifndef FOV_CALC_H
#define FOV_CALC_H

#include <iostream>
#include <cmath>

class FOVCalc{
public:
  FOVCalc();
  bool SetSensorParams(int sensor_horizontal_pix, 
                       int sensor_vertical_pix, 
                       float sensor_horizontal_pixel_pitch, 
                       float sensor_vertical_pixel_pitch, 
                       float lens_focal_length);
  void SetSensorFocalLength(float focal_length);
  void SetSensorHorizontalPixels(int pixels);
  void SetSensorVerticalPixels(int pixels);
  void SetSensorCropSize(int horizontal_pixels, int vertical_pixels);
  void SetSensorHorizontalPixelPitch(float pitch);
  void SetSensorVerticalPixelPitch(float pitch);
  float GetDiagonalFOV();
  float GetHorizontalFOV();
  float GetVerticalFOV();
  float GetSensorHorizontalSize();
  float GetSensorVerticalSize();
  float GetSensorDiagonalSize();
  
private:
  void CalcAllFOV();
  void CalcDiagonalFOV();
  void CalcHorizontalFOV();
  void CalcVerticalFOV();

  int sensor_horizontal_pixels_;
  int sensor_vertical_pixels_;
  float sensor_horizontal_pixel_pitch_;
  float sensor_vertical_pixel_pitch_;
  float sensor_horizontal_;
  float sensor_vertical_;
  float sensor_diagonal_;
  float vertical_fov_;
  float horizontal_fov_;
  float diagonal_fov_;
  float focal_length_;

};

#endif // FOV_CLAC_H