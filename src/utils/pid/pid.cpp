#ifndef PID_SOURCE
#define PID_SOURCE

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl{
public:
  PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
  ~PIDImpl();
  double calculate( double setpoint, double pv );
  double calculate( double setpoint, double pv, double dt_);
  double calculate_rotational( double setpoint, double pv );

private:
  double _dt;
  double _max;
  double _min;
  double _Kp;
  double _Kd;
  double _Ki;
  double _pre_error;
  double _integral;
};

PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki ){
  pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}

double PID::calculate( double setpoint, double pv ){
  return pimpl->calculate(setpoint,pv);
}

double PID::calculate_rotational( double setpoint, double pv ){
  return pimpl->calculate_rotational(setpoint,pv);
}

double PID::calculate(double setpoint, double pv, double dt_){
  return pimpl->calculate(setpoint,pv,dt_);
}

PID::~PID() {
  delete pimpl;
}

PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki ) :
  _dt(dt),
  _max(max),
  _min(min),
  _Kp(Kp),
  _Kd(Kd),
  _Ki(Ki),
  _pre_error(0),
  _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv ){

  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = _Kp * error;

  // Integral term
  _integral += error * _dt;
  double Iout = _Ki * _integral;

  // Derivative term
  double derivative = (error - _pre_error) / _dt;
  double Dout = _Kd * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Restrict to max/min
  if( output > _max ){
    output = _max;
  }else if( output < _min ){
    output = _min;
  }

  // Save error to previous error
  _pre_error = error;
  return output;
}

double PIDImpl::calculate_rotational( double setpoint, double pv ){

  // Calculate error
  double error = setpoint - pv;
 
  if(error > M_PI){
    error -= (M_PI * 2);
  }
  else if(error <= -M_PI){
    error += (M_PI * 2);
  }
  //cout<<"ERROR:"<<error<<endl;
  double gain;
  //lazy gain scheduling
  if(abs(error) < M_PI_2){
    gain = .25;
  }
  else{
    gain = 1;
  }

  // Proportional term
  double Pout = _Kp * error; 
  //cout<<"proportional:"<<Pout<<endl;
  // Integral term
  _integral += error * _dt;
  double Iout = _Ki * _integral;
  //cout<<"integral:"<<Iout<<endl;
  // Derivative term
  double derivative = (error - _pre_error) / _dt;
  double Dout = _Kd * derivative;
  //cout<<"derivative:"<<Dout<<endl;
  // Calculate total output
  double output = /*gain * */ (Pout + Iout + Dout);

  // Restrict to max/min
  if( output > _max ){
    output = _max;
  }else if( output < _min ){
    output = _min;
  }
  //output = error;
  // Save error to previous error
  _pre_error = error;
  return output;
}

double PIDImpl::calculate( double setpoint, double pv, double dt_ ){

  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = _Kp * error;

  // Integral term
  _integral += error * dt_;
  double Iout = _Ki * _integral;

  // Derivative term
  double derivative = (error - _pre_error) / dt_;
  double Dout = _Kd * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Restrict to max/min
  if( output > _max ){
    output = _max;
  }else if( output < _min ){
    output = _min;
  }

  // Save error to previous error
  _pre_error = error;
  return output;
}
PIDImpl::~PIDImpl(){
  cout<<"PID destructor"<<endl;
}

#endif //PID_SOURCE