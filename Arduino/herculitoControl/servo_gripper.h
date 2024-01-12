#ifndef SERVO_H_
#define SERVO_H_

#include <Arduino.h>
#include "config.h"
#ifdef TH_SERVO
#include <ESP32Servo.h>

class Servo_Gripper : public Servo {
public:
  Servo_Gripper(int pin, int _centerOffset, int _invert, int _homepos);
  void enable();
  void disable();
  void open_absolute(double percent);
  void open_relative(double addPerCent);
  void open_keys(bool positive);
  void turn_absolute(double angle);
  void turn_relative(double addAngle);
  void turn_keys(bool positive);
  int get_angle();
  void home(void);
  int homepos;
private:
  Servo servo_motor;
  int servo_pin;
  int invert;
  int cur_angle;
  int centerOffset;
};
#endif

#endif
