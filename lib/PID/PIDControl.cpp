#include "PIDcontrol.h"

PIDControl::PIDControl(double Kp, double Ki, double Kd)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), setpoint_(0.0), integral_(0.0), prevInput_(0.0) {}

double PIDControl::setSetpoint(double setpoint) {
  setpoint_ = setpoint;
  return setpoint;
}

double PIDControl::compute(double input) {
  double error = setpoint_ - input;
  double proportionalTerm = Kp_ * error;
  integral_ += Ki_ * error;
  double derivativeTerm = Kd_ * (input - prevInput_);
  prevInput_ = input;
  return proportionalTerm + integral_ + derivativeTerm;
}