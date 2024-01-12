#include "servo_gripper.h"
#include "config.h"
#include "ESP32Servo.h"
#include "serial.h"

#ifdef DUALSHOCK
#include "dualshock.h"
#endif

#ifdef TH_SERVO
Servo_Gripper::Servo_Gripper(int pin, int _centerOffset, int _invert, int _homepos) {
  servo_pin = pin;
  centerOffset = _centerOffset;
  invert = _invert;
  homepos = _homepos;
  cur_angle = _homepos;
}

// Published values for SG90 servos; adjust if needed
int minUs = 500;   // 800;
int maxUs = 2500;  //2500;

void Servo_Gripper::enable() {
  servo_motor.attach(servo_pin, minUs, maxUs);
  delayMicroseconds(100);
}

void Servo_Gripper::disable() {
  servo_motor.detach();
}


int Servo_Gripper::get_angle() {
  return (cur_angle-homepos-centerOffset);
}

void Servo_Gripper::open_absolute(double percent) {
  double new_angle;
  if (!servo_motor.attached()) return;
  if (invert == 1)
    new_angle = 1.8 * percent;
  else
    new_angle = 1.8 * (100.0 - percent);
  if (new_angle > 180.0) new_angle = 180.0;
  else if (new_angle < 0.0) new_angle = 0.0;
  servo_motor.write((int)round(new_angle));
  cur_angle = round(new_angle);
}

void Servo_Gripper::home(void) {
  Servo_Gripper::open_absolute(homepos);
}

void Servo_Gripper::open_relative(double addPercent) {
  double new_angle;
  if (!servo_motor.attached()) return;
  if (invert == 1)
    new_angle = 1.8 * cur_angle + addPercent;
  else
    new_angle = 1.8 * (100.0 - cur_angle - addPercent);
  if (new_angle > 180.0) new_angle = 180.0;
  else if (new_angle < 0.0) new_angle = 0.0;
  servo_motor.write((int)round(new_angle));
  cur_angle = round(new_angle);
}

void Servo_Gripper::open_keys(bool positive) {
  int rad = round(cur_angle);
  if (!servo_motor.attached()) return;

  if (positive) {
    while (PS4.L2Value()) {
      rad++;
      if (rad > 180) rad = 180;
      servo_motor.write(rad);
      delay(10);
      int val = PS4.L2Value();
      //SPrintln("val ", val);
      if (val < 120) delay(40);
      if (val < 180) delay(20);
      if (val < 255) delay(10);
    }
  } else {
    while (PS4.R2Value()) {
      rad--;
      if (rad < 0) rad = 0;
      servo_motor.write(rad);
      delay(10);
      int val = PS4.R2Value();
      //SPrintln("val ", val);
      if (val < 120) delay(40);
      if (val < 180) delay(20);
      if (val < 255) delay(10);
    }
  }
  cur_angle = rad;
}

void Servo_Gripper::turn_absolute(double angle) {
  double new_angle;
  if (!servo_motor.attached()) return;
  if (invert == 1)
    new_angle = 90.0 + centerOffset - angle;  // midpoint is 0 degrees
  else
    new_angle = angle + 90.0 + centerOffset;  // midpoint is 0 degrees
  if (new_angle > 180.0) new_angle = 180.0;
  else if (new_angle < 0.0) new_angle = 0.0;
  if (new_angle > cur_angle) {
    for (int rad = cur_angle; rad < new_angle; ++rad) {
      servo_motor.write(rad);
      delay(30);
    }
  } else {
    for (int rad = cur_angle; rad > new_angle; --rad) {
      servo_motor.write(rad);
      delay(30);
    }
  }
  cur_angle = new_angle;
}

void Servo_Gripper::turn_relative(double addAngle) {
  double new_angle;
  if (!servo_motor.attached()) return;
  if (invert == 1)
    new_angle = cur_angle - addAngle;
  else
    new_angle = cur_angle + addAngle;
  if (new_angle > 180.0) new_angle = 180.0;
  else if (new_angle < 0.0) new_angle = 0.0;
  if (new_angle > cur_angle) {
    for (int rad = cur_angle; rad < new_angle; ++rad) {
      servo_motor.write(rad);
      delay(30);
    }
  } else {
    for (int rad = cur_angle; rad > new_angle; --rad) {
      servo_motor.write(rad);
      delay(30);
    }
  }
  cur_angle = new_angle;
}

void Servo_Gripper::turn_keys(bool positive) {
  int rad = round(cur_angle);
  if (!servo_motor.attached()) return;

  if (positive) {
    while (PS4.R1()) {
      rad++;
      if (rad > 180) rad = 180;
      servo_motor.write(rad);
      delay(40);
    }
  } else {
    while (PS4.L1()) {
      rad--;
      if (rad < 0) rad = 0;
      servo_motor.write(rad);
      delay(40);
    }
  }
  cur_angle = rad;
}

#endif
