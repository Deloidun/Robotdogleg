#pragma once

#ifndef PIDCONTROL_H
#define PIDCONTROL_H

class PIDControl {
public:
  PIDControl(double Kp, double Ki, double Kd);
  double setSetpoint(double setpoint);
  double compute(double input);

private:
  double Kp_;
  double Ki_;
  double Kd_;
  double setpoint_;
  double integral_;
  double prevInput_;
};

#endif  // PIDCONTROL_H