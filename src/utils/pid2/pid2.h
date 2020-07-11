//
//  pid2.h
//
//  Created by Isaac Reed on 6/7/20
//
//  https://e2e.ti.com/blogs_/b/industrial_strength/archive/2013/04/13/teaching-your-pi-controller-to-behave-part-vii
//

#include <iostream>
#include <cmath>

#ifndef PID2_H
#define PID2_H

typedef struct PIDParams{
  float kp;
  float ki;
  float kd;
  float ts;
  float td;
  float limit_max;
  float limit_min;
};

class PID2{
public:
  PID2();
  PID2(PIDParams *params);
  float Update();

private:

  // Gains
  float kp_;
  float ki_;
  float kd_;

  // Sample time
  float ts_;

  // Derivative low-pass time constant
  float td_;

  // Output limits
  float limit_min_;
  float limit_max_;

  float integrator_;
  float previous_error_;
  float differentiator_;
  float previous_mesaurement_;

  float out_;

};

#endif // PID2_H