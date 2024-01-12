#include "config.h"
#include "misc.h"
#include "StepperControl.h"

// See also: int StepperControl::insideLimits(long position) below
int insideLimitsAB(long positionA, long positionB) {
  if (positionB < positionA+400) return(1);
  return(0);
}

StepperControl::StepperControl(int _enable_pin, int _endstop_pin, int _step_pin_number, int _dir_pin_number,
                               int _stepper_direction, int _home_position, int _max_postion, int _min_es_dist,
                               int _park_position, int _max_speed, int _acceleration_steps, int _step_delay, float _steps_per_unit) {
  step_pin = 0;
  dir_pin = 0;
  current_position = 0;
  max_speed = 200.0;
  min_pulse_width = 20;
  acceleration_steps = 200.0;
  current_step_period = 0.0;
  step_pin = _step_pin_number;
  dir_pin = _dir_pin_number;
  enable_pin = _enable_pin;
  endstop_pin = _endstop_pin;
  direction = _stepper_direction;
  home_position = _home_position;
  min_es_dist = _min_es_dist;
  park_position = _park_position;
  def_max_speed = max_speed = _max_speed,
  def_acceleration_steps = acceleration_steps = _acceleration_steps;
  step_delay = _step_delay;
  max_position = _max_postion;
  steps_per_unit = _steps_per_unit;
  // setup pins
  pinMode(step_pin, OUTPUT);
  pinMode(dir_pin, OUTPUT);
  if (enable_pin != -1)
    pinMode(enable_pin, OUTPUT);
  //pinMode(endstop_pin, INPUT_PULLUP);
  pinMode(endstop_pin, INPUT);
  // init pin
  digitalWrite(dir_pin, LOW);
  digitalWrite(step_pin, LOW);
  disableOutputs();
}

double StepperControl::steps2angle(long position, double fullScale) {
  return ((double)(position - home_position) * fullScale / steps_per_unit);
}

double StepperControl::currentAngle(double fullScale) {
  return ((double)(current_position - home_position) * fullScale / steps_per_unit);
}

double StepperControl::steps2distance(long position) {
  return ((double)(position - home_position) / steps_per_unit);
}

double StepperControl::currentDistance() {
  return ((double)(current_position - home_position) / steps_per_unit);
}

int StepperControl::angle2steps(double ang, double fullScale) {
  return (round(ang * steps_per_unit / fullScale) + home_position);
}

int StepperControl::distanceSteps(double distance) {
  return (round(distance * steps_per_unit) + home_position);
}

int StepperControl::endStop() {
  return (digitalRead(endstop_pin));
}

int StepperControl::insideLimits(long position) {
  if (position==-1) return(1); // always legal
  if ((position<=0) && (position<=max_position)) return(1);
  return(0); // outside of legal limits
}

void StepperControl::disableOutputs() {
  power_on = false;
  digitalWrite(enable_pin, HIGH);
}

void StepperControl::enableOutputs() {
  power_on = true;
  digitalWrite(enable_pin, LOW);
}

// move back to endstop and report the number of steps taken
int StepperControl::goto_endstop() {
  int cnt = 0;
  if (direction > 0)
    digitalWrite(dir_pin, HIGH);
  else
    digitalWrite(dir_pin, LOW);
  while (digitalRead(endstop_pin) == 0) {
    cnt++;
    digitalWrite(step_pin, HIGH);  // execute the step on the rising edge
    delayMicroseconds(min_pulse_width);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(step_delay);
  }
  return (cnt);
}

void StepperControl::go_forward(int steps) {
  if (direction < 0)
    digitalWrite(dir_pin, HIGH);
  else
    digitalWrite(dir_pin, LOW);
  for (int i = 0; i <= steps; i++) {
    digitalWrite(step_pin, HIGH);  // execute the step on the rising edge
    delayMicroseconds(min_pulse_width);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(step_delay);
  }
}

// move to endstop and then back to the reference point 0
void StepperControl::home(bool skip_homerun) {
  if (StepperControl::goto_endstop() < min_es_dist) {
    StepperControl::go_forward(min_es_dist);
    StepperControl::goto_endstop();
  }
  setCurrentPosition(0);
  if (!skip_homerun) {
    move(home_position + home_offset);
    while (run()!=0)
      ;
    setCurrentPosition(home_position);
  }
}

void StepperControl::runSpeedToPosition() {
  int dir;
  if (current_position == target_position) return;
  if (current_position > target_position) {
    digitalWrite(dir_pin, HIGH);
    dir = 1;
  } else {
    digitalWrite(dir_pin, LOW);
    dir = -1;
  }
  while (current_position != target_position) {
    digitalWrite(step_pin, HIGH);  // execute the step on the rising edge
    delayMicroseconds(min_pulse_width);
    digitalWrite(step_pin, LOW);
    delayMicroseconds(step_delay);
    current_position = current_position + dir;
  }
}

void StepperControl::setCurrentPosition(long currentPosition) {
  current_position = currentPosition;
}

long StepperControl::currentPosition() {
  return (current_position);
}

long StepperControl::homePosition() {
  return (home_position);
}

long StepperControl::parkPosition() {
  return (park_position);
}

void StepperControl::setTargetPosition(long targetPosition) {
  target_position = targetPosition;
}

long StepperControl::targetPosition() {
  return (target_position);
}

// move the target position so that the motor will begin deceleration now
long StepperControl::stop() {
  if (direction_scaler > 0)
    target_position = current_position + deceleration_distance;
  else
    target_position = current_position - deceleration_distance;
  return (target_position);
}

void StepperControl::setSpeedAccelerationPercent(double speed, double acceleration) {
  max_speed = speed * def_max_speed;
  acceleration_steps = acceleration * def_acceleration_steps;
}

void StepperControl::move(long relative) {
  moveTo(current_position + relative);
}

// setup a move, units are in steps, no motion occurs until run() is called
void StepperControl::moveTo(long absolute) {
  long distanceToTravel;
  target_position = absolute;                                             // save the target location
  ramp_Initial_step_period = 1000000.0 / sqrt(2.0 * acceleration_steps);  // determine the period in US of the first step
  desired_step_period = 1000000.0 / max_speed;                            // determine the period in US between steps when going at the desired velocity
  // determine the number of steps needed to go from the desired velocity down to a
  // velocity of 0, Steps = Velocity^2 / (2 * Accelleration)
  deceleration_distance = (long)round((max_speed * max_speed) / (2.0 * acceleration_steps));
  distanceToTravel = target_position - current_position;  // determine the distance and direction to travel
  start_position = current_position;
  if (distanceToTravel < 0) {
    distanceToTravel = -distanceToTravel;
    direction_scaler = -1;
    if (direction > 0)
      digitalWrite(dir_pin, HIGH);
    else
      digitalWrite(dir_pin, LOW);
  } else {
    direction_scaler = 1;
    if (direction > 0)
      digitalWrite(dir_pin, LOW);
    else
      digitalWrite(dir_pin, HIGH);
  }
  // check if travel distance is too short to accelerate up to the desired velocity
  if (distanceToTravel <= (deceleration_distance * 2L))
    deceleration_distance = (distanceToTravel / 2L);
  ramp_next_step_period = ramp_Initial_step_period;  // start the acceleration ramp at the beginning
  acceleration = acceleration_steps / 1E12;
  start_new_move = true;
}

// -----------------------------------------------------------------------------------------------------------
// Move one step, if it time to do so
//  Exit: 0=movement finished, 1=if it was time to move one step, -1=not yet time to do a step
int StepperControl::run(void) {
  unsigned long currentTime;
  unsigned long periodSinceLastStep;
  long distanceToTarget;
  if (current_position == target_position)  // at the target position?
    return (0);
  if (start_new_move) {  // the first call to start this new move
    ramp_last_step_time = micros();
    start_new_move = false;
  }
  currentTime = micros();
  periodSinceLastStep = currentTime - ramp_last_step_time;  // how much time has elapsed since the last step
  if (periodSinceLastStep < (unsigned long)ramp_next_step_period)
    return (-1);  // not yet time for the next step
  // determine the distance from the current position to the target
  distanceToTarget = target_position - current_position;
  if (distanceToTarget < 0)
    distanceToTarget = -distanceToTarget;
  if (distanceToTarget == deceleration_distance) {
    acceleration = -acceleration;                                          // start decelerating
    if (abs(current_position - start_position) < deceleration_distance) {  // within rampup?
      deceleration_distance = abs(current_position - start_position);
      if (current_position > start_position)  // update destination
        target_position = current_position + deceleration_distance;
      else
        target_position = current_position - deceleration_distance;
    }
  }
  digitalWrite(step_pin, HIGH);  // execute the step on the rising edge
  delayMicroseconds(2);
  current_position += direction_scaler;
  current_step_period = ramp_next_step_period;
  // compute the period for the next step
  // StepPeriodInUS = LastStepPeriodInUS * (1 - AccelerationPerUSPerUS * LastStepPeriodInUS^2)
  ramp_next_step_period = ramp_next_step_period * (1.0 - acceleration * ramp_next_step_period * ramp_next_step_period);
  digitalWrite(step_pin, LOW);
  // clip the speed so that it does not accelerate beyond the desired velocity
  if (ramp_next_step_period < desired_step_period)
    ramp_next_step_period = desired_step_period;
  ramp_last_step_time = currentTime;
  if (current_position == target_position) {
    current_step_period = 0.0;  // final target position
    return (0);
  }
  return (1);  // not at final position; still running
}

void StepperControl::run_one_step(int dist) {
  current_position = current_position + dist;
  target_position = current_position;
  if (dist < 0) {
    if (direction > 0)
      digitalWrite(dir_pin, HIGH);
    else
      digitalWrite(dir_pin, LOW);
  } else {
    if (direction > 0)
      digitalWrite(dir_pin, LOW);
    else
      digitalWrite(dir_pin, HIGH);
  }
  delayMicroseconds(20);
  digitalWrite(step_pin, HIGH);  // move one step on the rising edge
  delayMicroseconds(2);
  digitalWrite(step_pin, LOW);
}

bool StepperControl::isRunning() {
  if (current_position == target_position)
    return (false);  // stepper is at the target position
  else
    return (true);  // stepper still running
}

void StepperControl::setMinPulseWidth(unsigned int minWidth) {
  min_pulse_width = minWidth;
}
