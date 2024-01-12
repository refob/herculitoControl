#ifndef StepperControl_h
#define StepperControl_h

#include <Arduino.h>
#include <stdlib.h>
#include "config.h"

int insideLimitsAB(long positionA, long positionB);

class StepperControl {
public:
  StepperControl(int _enable_pin, int _endstop_pin, int step_pinNumber, int dir_pinNumber,
                 int stepper_direction, int homePosition, int maxPosition,
                 int minEsDist, int parkPosition, int maxSpeed, int accelerationSteps, int stepDelay, float _steps_per_unit);
  void disableOutputs(void);
  void enableOutputs(void);
  void run_one_step(int dir);
  void home(bool skip_homerun);
  int goto_endstop();
  void go_forward(int steps);
  void setCurrentPosition(long currentPosition);
  long currentPosition(void);
  double steps2angle(long position, double fullScale);
  double currentAngle(double fullScale);
  double steps2distance(long position); 
  double currentDistance(void);
  int angle2steps(double ang, double fullScale);
  int distanceSteps(double distance);
  long homePosition(void);
  long parkPosition(void);
  void setTargetPosition(long targetPosition);
  long targetPosition(void);
  int endStop(void);
  int insideLimits(long position);
  long stop(void);  // stop a running motion - returns new move target
  void setSpeedAccelerationPercent(double speed, double acceleration);
  void move(long relative);
  void moveTo(long absolute);
  bool isRunning(void);
  int run(void);
  void setMinPulseWidth(unsigned int minWidth);
  void runSpeedToPosition(void);
  bool power_on;
private:
  int enable_pin;
  byte endstop_pin;
  byte step_pin;
  byte dir_pin;
  int step_delay;                // usec to wait after step is done
  int direction;                 // normal=1, inversion=-1
  int min_es_dist;               // min. distance to end stop for calibration run
  int home_offset;               // steps to move away from endstop position to reach location 0
  int home_position;             // reference position
  int park_position;             //position to park toolhead
  int max_position;              // maximum allowed position in steps
  float max_speed;               // set maximum speed in steps/sec
  float acceleration_steps;      // acceleration in steps/sec^2
  float def_max_speed;           // set default maximum speed in steps/sec
  float def_acceleration_steps;  // set default acceleration in steps/sec^2
  float steps_per_unit;
  long target_position;  // target position in steps
  long start_position;
  bool start_new_move;
  float desired_step_period;   // desired step period in usec
  long deceleration_distance;  // distance in steps
  int direction_scaler;
  float ramp_Initial_step_period;     // initial step period in usec
  float ramp_next_step_period;        // next step period in usec
  unsigned long ramp_last_step_time;  // last step time in usec
  float acceleration;                 // acceleration in steps/usec^2
  float current_step_period;          // current step period in usec
  long current_position;              // current position in steps
  unsigned int min_pulse_width;       // minimum pulse width in usec
};

#endif
