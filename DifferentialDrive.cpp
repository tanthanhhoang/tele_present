/*
   DifferentialDrive.cpp

    Created on: Jul 20, 2018
        Author: Thanh Hoang
*/

#include "DifferentialDrive.h"

DDrive::DDrive() {
  _time_cnt = 0;
}

/******
Assign pins for the right motor
5 pins in total
******/
void DDrive::SetRightMotorPins(int encoder_A, int encoder_B, int motor_enable_1, int motor_enable_2, int PWM_pin) {
  _motor_right.SetPins(encoder_A, encoder_B, motor_enable_1, motor_enable_2, PWM_pin, true);
}

/******
Assign pins for the left motor
5 pins in total
******/
void DDrive::SetLeftMotorPins(int encoder_A, int encoder_B, int motor_enable_1, int motor_enable_2, int PWM_pin) {
  _motor_left.SetPins(encoder_A, encoder_B, motor_enable_1, motor_enable_2, PWM_pin, false);
}

/******
Callback function for Timer1 ISR to update position of the right motor
******/
void DDrive::RightMotorInterrupt() {
  _motor_right.EncoderInterrupt();
}

/******
Callback function for Timer1 ISR to update position of the left motor
******/
void DDrive::LeftMotorInterrupt() {
  _motor_left.EncoderInterrupt();
}

void DDrive::TimerInterrupt() {
  // if new signal didn't receive, stop the robot (in case of bad wifi connection)
  if (isReceived) {
    isReceived = false;
  }
  else {
    _motor_right.SetPWM(0, 100);
    _motor_left.SetPWM(0, 100);
  }
}

void DDrive::ParseCommand(unsigned char cmd) {
  // the first 4 bits are used for linear velocity, the next 4 bits are used for angular velocity
  _linear = (((cmd & 0xF0) >> 4) - 6) * 0.1; //-0.6 to 0.8, 0.1 increament
  _angular = ((cmd & 0x0F) - 7) * 0.1; // -0.7 to -0.7, 0.1 increment
  isReceived = true;

  // calculate right and left speed
  _right_speed = (2.0 * _linear + _angular) / 4;
  _left_speed  = (2.0 * _linear - _angular) / 4;
  
  // Calculate the PWM values from the left and right speeds
  _r_pwm = _motor_right.SpeedToPWM(_right_speed, DEBUG);
  _l_pwm = _motor_left.SpeedToPWM(_left_speed , false);

  // Update motor PWM speed
  _motor_right.SetPWM(_r_pwm, 100);
  _motor_left.SetPWM(_l_pwm, 100);
}

void DDrive::ComputeDD() {
}

