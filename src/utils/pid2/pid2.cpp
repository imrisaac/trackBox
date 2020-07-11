//
//  pid2.h
//
//  Created by Isaac Reed on 6/7/20
//

#include "pid2.h"

using namespace std;

ostream& operator<<(ostream& os, const PIDParams &params){
  os << "P " << params.kp << endl;
  os << "I " << params.ki << endl;
  os << "D " << params.kd << endl;
  os << "Limit Max "<< params.limit_max << endl;
  os << "Limit Min "<< params.limit_min << endl;
}

PID2::PID2(){
  kp_ = 0;
  ki_ = 0;
  kd_ = 0;
  ts_ = 0;
  td_ = 0;
  integrator_ = 0;
  previous_error_ = 0;
  differentiator_ = 0;
  previous_mesaurement_ = 0;
  out_ = 0;  
}

PID2::PID2(PIDParams *params){
  kp_ = params->kp;
  ki_ = params->ki;
  kd_ = params->kd;
  ts_ = params->ts;
  td_ = params->td;
  limit_max_ = params->limit_max;
  limit_min_ = params->limit_min;
}

float PID2::Update(float setpoint, float measurement){
  float error
}
