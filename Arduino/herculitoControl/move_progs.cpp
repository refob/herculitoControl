#include <Arduino.h>
//#include <stdlib.h> // incl. abs(int)
#include "config.h"
#include "StepperControl.h"
#include "misc.h"
#include "serial.h"
#include "kinematics.h"
#include "GCodeCommands.h"
#include "move_progs.h"
#include "display.h"

extern bool recording;
extern Display display;
extern digOutput led;
extern bool steppers_running;
extern double shank_length;
extern GCodeCommands Parser;
extern StepperControl StepperA;
extern StepperControl StepperB;
extern StepperControl StepperC;
#ifdef STEPPER_D
extern StepperControl StepperD;
#endif
extern void get_next_move_point();

#ifdef DUALSHOCK
#include "dualshock.h"
#endif

#ifdef TH_SERVO
#include "servo_gripper.h"
extern Servo_Gripper servo_gripper;
extern Servo_Gripper servo_gripper2;
#endif

void enable_allOutputs() {
  // Note: CNC Shield V3.0 has only ONE enable pin for all drivers!
  StepperA.enableOutputs();  // A, B, C or D enables power
  delay(1000);               // give some time to stabilize supply voltage
  servo_gripper.enable();
  servo_gripper2.enable();
}

void disable_allOutputs() {
  StepperA.disableOutputs();  // cut off power
  delay(500);
  servo_gripper.disable();
  servo_gripper2.disable();
}

void home_AB_steppers() {
  led.Off();
  if ((StepperA.endStop() == 1) && (StepperB.endStop() == 0)) {
    StepperA.home(true);
    StepperB.home(true);
    StepperA.setSpeedAccelerationPercent(1.0, 1.0);
    StepperB.setSpeedAccelerationPercent(1.0, 1.0);
    runall(STEPPER_A_HOME, -1, -1, -1);
    runall(-1, STEPPER_B_HOME, -1, -1);
    servo_gripper.open_absolute(servo_gripper.homepos);
    servo_gripper2.turn_absolute(0);  // pos 0 is servo_gripper2.homepos
  }
  delay(DEFAULT_DELAY);
  led.On();
}

void home_C_stepper() {
  led.Off();
  StepperC.home(false);
  delay(DEFAULT_DELAY);
  led.On();
}

#ifdef STEPPER_D
void home_D_stepper() {
  led.Off();
  StepperD.home(false);
  delay(DEFAULT_DELAY);
  led.On();
}
#endif

void home_all_actuators() {
  if ((StepperA.endStop() == 1) && (StepperB.endStop() == 0)) {
    display.herculito();
    home_C_stepper();
#ifdef STEPPER_D
    home_D_stepper();
#endif
    home_AB_steppers();
#ifdef TH_SERVO
    servo_gripper.open_absolute(servo_gripper.homepos);
    servo_gripper2.turn_absolute(0);  // pos 0 is servo_gripper2.homepos
#endif
    display.message("Home");
  } else {
    display.error("Off Park");
  }
}

void park_toolhead(bool motorAB, bool motorC, bool motorD) {
  display.herculito();
  led.On();
  if (motorAB) {
#ifdef TH_SERVO
    servo_gripper.open_absolute(servo_gripper.homepos);
    servo_gripper2.turn_absolute(0);  // pos 0 is servo_gripper2.homepos
#endif
      // first put AB to home
    runall(STEPPER_A_HOME, STEPPER_B_HOME, -1, -1);
    StepperA.setSpeedAccelerationPercent(0.1, 0.1);
    StepperB.setSpeedAccelerationPercent(1.0, 0.1);
    runall(-1, 0, -1, -1);
    runall(0, -1, -1, -1);
    StepperB.setSpeedAccelerationPercent(0.1, 0.1);
    runall(STEPPER_A_PARK, STEPPER_B_PARK, -1, -1);
  }
  if (motorD)
    runall(-1, -1, -1, STEPPER_D_PARK);
  if (motorC)
    runall(-1, -1, STEPPER_C_PARK, -1);
  delay(DEFAULT_DELAY);  // wait to hear when power switches off
  disable_allOutputs();
  led.Off();
  if ((StepperA.endStop() == 1) && (StepperB.endStop() == 0)) {
    display.message("Park");
  } else {
    display.error("Off Park");
  }
}

void moveto_next_point(double x, double y, double z, double u, bool linear_move) {
  POINT angs;
  long a_steps, b_steps, a_delta, b_delta;

  if (linear_move) {  // special case for x-only or y-only moves -> G1
    double xdelta, ydelta;
    POINT pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
    xdelta = x - pcur.x;
    ydelta = y - pcur.y;
    if (abs(xdelta) < 0.5) xdelta = 0.0;
    if (abs(ydelta) < 0.5) ydelta = 0.0;
    if ((xdelta == 0.0) && (ydelta != 0)) {
      SPrintln("ydelta ", ydelta);
      y_move(ydelta);
    } else if ((xdelta != 0.0) && (ydelta == 0)) {
      SPrintln("xdelta ", xdelta);
      x_move(xdelta);
    }
    // else:  do nothing and perform only a non-linear move
  }

  angs = point2angles(x + shank_length, y + shank_length);
  a_steps = StepperA.angle2steps(angs.x, TWO_PI);
  b_steps = StepperB.angle2steps(angs.y, TWO_PI);

  a_delta = abs(a_steps - StepperA.currentPosition());
  b_delta = abs(b_steps - StepperB.currentPosition());

  if (a_delta > b_delta) {
    StepperA.setSpeedAccelerationPercent(1.0, 1.0);
    StepperB.setSpeedAccelerationPercent(1.0 * a_delta / a_delta, 1.0 * a_delta / a_delta);
  } else {
    StepperB.setSpeedAccelerationPercent(1.0, 1.0);
    StepperA.setSpeedAccelerationPercent(1.0 * a_delta / b_delta, 1.0 * a_delta / b_delta);
  }
  StepperA.moveTo(a_steps);
  StepperB.moveTo(b_steps);
  StepperC.moveTo(StepperC.angle2steps(u, 360));
#ifdef STEPPER_D
  StepperD.moveTo(StepperD.distanceSteps(z));
#endif
#ifdef DEBUG
  SPrint("moveTo: ", "");
  SPrint3(" ", e.x, e.y);
  SPrint3ln(" ", e.z, e.u);
#endif
  steppers_running = true;
}

#ifdef STEPPER_D
void runall(int destA, int destB, int destC, int destD) {
  bool flagA, flagB, flagC, flagD;
  if (destA != -1) StepperA.moveTo(destA);
  else StepperA.moveTo(StepperA.currentPosition());
  if (destB != -1) StepperB.moveTo(destB);
  else StepperB.moveTo(StepperB.currentPosition());
  if (destC != -1) StepperC.moveTo(destC);
  else StepperC.moveTo(StepperC.currentPosition());
  if (destD != -1) StepperD.moveTo(destD);
  else StepperD.moveTo(StepperD.currentPosition());
  flagA = flagB = flagC = flagD = true;
  while (flagA || flagB || flagC || flagD) {
    flagA = (StepperA.run() != 0);
    flagB = (StepperB.run() != 0);
    flagC = (StepperC.run() != 0);
    flagD = (StepperD.run() != 0);
  };
}
#else
void runall(int destA, int destB, int destC, int destD) {
  bool flagA, flagB, flagC;
  if (destA != -1) StepperA.moveTo(destA);
  else StepperA.moveTo(StepperA.currentPosition());
  if (destB != -1) StepperB.moveTo(destB);
  else StepperB.moveTo(StepperB.currentPosition());
  if (destC != -1) StepperC.moveTo(destC);
  else StepperC.moveTo(StepperC.currentPosition());
#ifdef DEBUG
  SPrint3("Runall3: ", destA, destB);
  SPrintln(" ", destC);
#endif
  flagA = flagB = flagC = true;
  while (flagA || flagB || flagC) {
    flagA = StepperA.run();
    flagB = StepperB.run();
    flagC = StepperC.run();
  };
}
#endif


// wait for active move to finish
bool wait_until_active_move_is_done() {
  bool flagA, flagB, flagC, flagD;
  if (!steppers_running)
    return (true);
    //SPrintln("-> ","waiting for moves");
#ifdef STEPPER_D
  while (steppers_running) {  // loop until current move is done
    flagB = (StepperB.run() != 0);
    flagA = (StepperA.run() != 0);
    flagC = (StepperC.run() != 0);
    flagD = (StepperD.run() != 0);
    steppers_running = flagA || flagB || flagC || flagD;
  }
#else
  while (steppers_running) {  // loop until current move is done
    flagB = (StepperB.run() != 0);
    flagA = (StepperA.run() != 0);
    flagC = (StepperC.run() != 0);
    steppers_running = flagA || flagB || flagC;
  }
#endif
  //SPrintln("-> ","moves done - get next point?");
  get_next_move_point();
  //SPrintln("-> ","wait finished.");
  return (true);
}

double val2speed(double speedup, int val) {
  if (abs(val) >= 80)
    return (speedup * 0.1);
  else if (abs(val) > 40)
    return (speedup * 0.05);
  else
    return (speedup * 0.01);
}

void x_move_joystick() {
  int val, val2;
  RANGE ra;
  POINT pcur, angs;
  long asteps, bsteps;
  bool break_by_joystick = true;
  double yact, speed;
  /*
  double destx, desty;
  POINT angs;
  long asteps, bsteps;
  bool break_by_joystick = true;
  POINT pt, pa, pb, pbr, plim, pcur;
  double speed;
  */

  delay(60);  // delay a bit to get a higher value
  val = PS4.LStickY();
  if (val == -128)  // cover overflow
    val = 127;
  delay(40);
  val2 = PS4.LStickY();
  if (val2 == -128)  // cover overflow
    val = 127;
  val = (val + val2) / 2;     // average result a bit
  if (abs(val) < 10) return;  // ignore
  speed = val2speed(1.0, val);

  pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
  yact = pcur.y;

  ra = xrange(yact);
  if (val > 0)
    angs = point2angles(ra.high + shank_length, yact + shank_length);
  else
    angs = point2angles(ra.low + shank_length, yact + shank_length);
  asteps = StepperA.angle2steps(angs.x, TWO_PI);
  bsteps = StepperB.angle2steps(angs.y, TWO_PI);
  StepperA.setSpeedAccelerationPercent(speed, 1.0);
  StepperA.moveTo(asteps);

  while (StepperA.run() != 0) {
    pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
    if ((pcur.y - yact) < 0)
      StepperB.run_one_step(-1);
    else if ((pcur.y - yact) > 0)
      StepperB.run_one_step(1);
    if (break_by_joystick && PS4.LStickY() == 0) {
      StepperA.stop();
      break_by_joystick = false;
    }
    if (PS4.Down()) break;
    if (PS4.Up()) break;
  }
  StepperA.setSpeedAccelerationPercent(1.0, 1.0);
  StepperB.setSpeedAccelerationPercent(1.0, 1.0);
}

void y_move_joystick() {
  int val, val2;
  RANGE ra;
  POINT pcur, angs;
  double xact, speed;
  long asteps, bsteps;
  bool break_by_joystick = true;
  bool break_by_pos = false;

  delay(60);
  val = PS4.RStickY();
  if (val == -128)  // cover overflow
    val = 127;
  delay(40);
  val2 = PS4.RStickY();
  if (val2 == -128)  // cover overflow
    val2 = 127;
  val = (val + val2) / 2;  // average result a bit
  if (abs(val) < 10)
    return;  // ignore
  speed = val2speed(1.0, val);

  pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
  xact = pcur.x;

  ra = yrange(xact);
  if (val > 0)
    angs = point2angles(xact + shank_length, ra.high + shank_length);
  else
    angs = point2angles(xact + shank_length, ra.low + shank_length);
  asteps = StepperA.angle2steps(angs.x, TWO_PI);
  bsteps = StepperB.angle2steps(angs.y, TWO_PI);

  if (StepperB.currentPosition() < STEPPER_SWITCH_AB) {
    StepperB.setSpeedAccelerationPercent(speed, 1.0);
    StepperB.moveTo(bsteps);

    while (StepperB.run() != 0) {
      pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
      if ((pcur.x - xact) < 0)
        StepperA.run_one_step(1);
      else if ((pcur.x - xact) > 0)
        StepperA.run_one_step(-1);
      if (break_by_joystick && PS4.RStickY() == 0) {
        StepperB.stop();
        break_by_joystick = false;
      }
      if (PS4.Cross()) break;
      if (StepperB.currentPosition() > STEPPER_SWITCH_AB) {
        break_by_pos = true;
        break;
      }
    }
  } else break_by_pos = true;

  if (break_by_pos) {
    StepperA.setSpeedAccelerationPercent(speed, 1.0);
    StepperA.moveTo(asteps);

    while (StepperA.run() != 0) {
      pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
      if ((pcur.x - xact) < 0)
        StepperB.run_one_step(-1);
      else if ((pcur.x - xact) > 0)
        StepperB.run_one_step(1);
      if (break_by_joystick && PS4.RStickY() == 0) {
        StepperA.stop();
        break_by_joystick = false;
      }
      if (PS4.Cross()) break;
    }
  }
  StepperA.setSpeedAccelerationPercent(1.0, 1.0);
  StepperB.setSpeedAccelerationPercent(1.0, 1.0);
}

void z_move_joystick() {
  int val, val2;
  long dsteps;
  double speed;
  bool stop_by_joystick = true;

  delay(60);
  val = PS4.LStickX();
  delay(40);
  val2 = PS4.LStickX();
  val = (val + val2) / 2;  // average result a bit
  if (abs(val) < 10) return;
  speed = val2speed(5.0, val);
  if (val < 0)
    dsteps = STEPPER_D_MAX_STEPS;
  else
    dsteps = 0;
  StepperD.setSpeedAccelerationPercent(speed, 1.0);
  StepperD.moveTo(dsteps);

  while (StepperD.run() != 0) {
    if (stop_by_joystick && PS4.LStickX() == 0) {
      stop_by_joystick = false;
      StepperD.stop();
    }
  }
  StepperD.setSpeedAccelerationPercent(1.0, 1.0);
}

void u_move_joystick(void) {
  int val;
  long csteps;
  double speed;
  bool stop_by_joystick = true;

  delay(60);  // delay a bit to get a higher value
  val = PS4.RStickX();
  delay(40);
  val = (val + PS4.RStickX()) / 2;
  speed = val2speed(4.0, val) / 2.0;
  if (val < 0)
    csteps = StepperC.angle2steps(-360, 360);
  else
    csteps = StepperC.angle2steps(360, 360);

  StepperC.setSpeedAccelerationPercent(speed, 1.0);
  StepperC.moveTo(csteps);

  while (StepperC.run() != 0) {
    if (stop_by_joystick && PS4.RStickX() == 0) {
      stop_by_joystick = false;
      StepperC.stop();
    }
  }
  StepperC.setSpeedAccelerationPercent(1.0, 1.0);
}

void x_move_keys(bool positive) {
  RANGE ra;
  POINT pcur, angs;
  long asteps, bsteps;
  bool break_by_key = true;
  double yact;

  pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
  yact = pcur.y;

  ra = xrange(yact);
  if (positive)
    angs = point2angles(ra.high + shank_length, yact + shank_length);
  else
    angs = point2angles(ra.low + shank_length, yact + shank_length);
  asteps = StepperA.angle2steps(angs.x, TWO_PI);
  bsteps = StepperB.angle2steps(angs.y, TWO_PI);
  StepperA.setSpeedAccelerationPercent(0.03, 1.0);
  StepperA.moveTo(asteps);

  while (StepperA.run() != 0) {
    pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
    if ((pcur.y - yact) < 0)
      StepperB.run_one_step(-1);
    else if ((pcur.y - yact) > 0)
      StepperB.run_one_step(1);
    if (positive) {
      if (break_by_key && PS4.Up() == 0) {
        StepperA.stop();
        break_by_key = false;
      }
      if (PS4.Down()) break;
    } else {
      if (break_by_key && PS4.Down() == 0) {
        StepperA.stop();
        break_by_key = false;
      }
      if (PS4.Up()) break;
    }
  }
  StepperA.setSpeedAccelerationPercent(1.0, 1.0);
}

void y_move_keys(bool positive) {
  RANGE ra;
  POINT pcur, angs;
  double xact;
  long asteps, bsteps;
  bool break_by_key = true;
  bool break_by_pos = false;

  pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
  xact = pcur.x;

  ra = yrange(xact);
  if (positive)
    angs = point2angles(xact + shank_length, ra.high + shank_length);
  else
    angs = point2angles(xact + shank_length, ra.low + shank_length);
  asteps = StepperA.angle2steps(angs.x, TWO_PI);
  bsteps = StepperB.angle2steps(angs.y, TWO_PI);

  if (StepperB.currentPosition() < STEPPER_SWITCH_AB) {
    StepperB.setSpeedAccelerationPercent(0.03, 1.0);
    StepperB.moveTo(bsteps);

    while (StepperB.run() != 0) {
      pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
      if ((pcur.x - xact) < 0)
        StepperA.run_one_step(1);
      else if ((pcur.x - xact) > 0)
        StepperA.run_one_step(-1);
      if (positive) {
        if (break_by_key && PS4.Triangle() == 0) {
          StepperB.stop();
          break_by_key = false;
        }
        if (PS4.Cross()) break;
      } else {
        if (break_by_key && PS4.Cross() == 0) {
          StepperB.stop();
          break_by_key = false;
        }
        if (PS4.Triangle()) break;
      }
      if (StepperB.currentPosition() > STEPPER_SWITCH_AB) {
        break_by_pos = true;
        break;
      }
    }
  } else break_by_pos = true;

  if (break_by_pos) {
    StepperA.setSpeedAccelerationPercent(0.03, 1.0);
    StepperA.moveTo(asteps);

    while (StepperA.run() != 0) {
      pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
      if ((pcur.x - xact) < 0)
        StepperB.run_one_step(-1);
      else if ((pcur.x - xact) > 0)
        StepperB.run_one_step(1);
      if (positive) {
        if (break_by_key && PS4.Triangle() == 0) {
          StepperA.stop();
          break_by_key = false;
        }
        if (PS4.Cross()) break;
      } else {
        if (break_by_key && PS4.Cross() == 0) {
          StepperA.stop();
          break_by_key = false;
        }
        if (PS4.Triangle()) break;
      }
    }
  }
  StepperA.setSpeedAccelerationPercent(1.0, 1.0);
  StepperB.setSpeedAccelerationPercent(1.0, 1.0);
}

void z_move_keys(bool positive) {
  bool break_by_key = true;
  StepperD.setSpeedAccelerationPercent(0.25, 0.25);
  if (positive) {
    StepperD.moveTo(STEPPER_D_MAX_STEPS);
    while (StepperD.run() != 0) {
      if (break_by_key && PS4.Left() == 0) {
        StepperD.stop();
        break_by_key = false;
      }
    }
  } else {
    StepperD.moveTo(0);
    while (StepperD.run() != 0) {
      if (break_by_key && PS4.Right() == 0) {
        StepperD.stop();
        break_by_key = false;
      }
    }
  }
  StepperD.setSpeedAccelerationPercent(1.0, 1.0);
}

void u_move_keys(bool positive) {
  bool break_by_key = true;
  StepperC.setSpeedAccelerationPercent(0.2, 0.2);
  if (positive) {
    StepperC.moveTo(StepperC.angle2steps(360, 360));
    while (StepperC.run() != 0) {
      if (break_by_key && PS4.Circle() == 0) {
        StepperC.stop();
        break_by_key = false;
      }
    }
  } else {
    StepperC.moveTo(StepperC.angle2steps(-360, 360));
    while (StepperC.run() != 0) {
      if (break_by_key && PS4.Square() == 0) {
        StepperC.stop();
        break_by_key = false;
      }
    }
  }
  StepperC.setSpeedAccelerationPercent(1.0, 1.0);
}

void x_move(long xdelta) {
  POINT pcur, angs;
  long asteps, bsteps;
  double yact;

  pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
  yact = pcur.y;

  angs = point2angles(pcur.x + xdelta + shank_length, yact + shank_length);
  asteps = StepperA.angle2steps(angs.x, TWO_PI);
  bsteps = StepperB.angle2steps(angs.y, TWO_PI);
  StepperA.setSpeedAccelerationPercent(0.03, 1.0);
  StepperA.moveTo(asteps);

  while (StepperA.run() != 0) {
    pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
    if ((pcur.y - yact) < 0)
      StepperB.run_one_step(-1);
    else if ((pcur.y - yact) > 0)
      StepperB.run_one_step(1);
  }
  StepperA.setSpeedAccelerationPercent(1.0, 1.0);
}

void y_move(long ydelta) {
  POINT pcur, angs;
  double xact;
  long asteps, bsteps;
  bool break_by_pos = false;

  pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
  xact = pcur.x;

  angs = point2angles(xact + shank_length, pcur.y + ydelta + shank_length);
  asteps = StepperA.angle2steps(angs.x, TWO_PI);
  bsteps = StepperB.angle2steps(angs.y, TWO_PI);

  if (StepperB.currentPosition() < STEPPER_SWITCH_AB) {
    StepperB.setSpeedAccelerationPercent(0.03, 1.0);
    StepperB.moveTo(bsteps);

    while (StepperB.run() != 0) {
      pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
      if ((pcur.x - xact) < 0)
        StepperA.run_one_step(1);
      else if ((pcur.x - xact) > 0)
        StepperA.run_one_step(-1);
      if (StepperB.currentPosition() > STEPPER_SWITCH_AB) {
        break_by_pos = true;
        break;
      }
    }
  } else break_by_pos = true;

  if (break_by_pos) {
    StepperA.setSpeedAccelerationPercent(0.03, 1.0);
    StepperA.moveTo(asteps);

    while (StepperA.run() != 0) {
      pcur = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
      if ((pcur.x - xact) < 0)
        StepperB.run_one_step(-1);
      else if ((pcur.x - xact) > 0)
        StepperB.run_one_step(1);
    }
  }
  StepperA.setSpeedAccelerationPercent(1.0, 1.0);
  StepperB.setSpeedAccelerationPercent(1.0, 1.0);
}

void toggle_recording() {
  if (!recording) {
    recording = true;
    SPrintln("r ", 1);
    display.message("Recording on");
    PS4.setLed(100, 0, 0);
    delay(300);
  } else {
    recording = false;
    SPrintln("r ", 0);
    display.message("Recording off");
    PS4.setLed(0, 0, 100);
    delay(300);
  }
  PS4.sendToController();
  delay(10);  // avoid buffer overflow (?)
}

void insert_point() {
  if (recording) {
    POINT pt = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
    SPrint("s ", pt.x);
    SPrint(" ", pt.y);
    SPrint(" ", StepperD.currentDistance());
    SPrint(" ", StepperC.currentAngle(360));
    SPrint(" ", servo_gripper.get_angle() / 1.8);
    SPrintln(" ", servo_gripper2.get_angle());
    delay(300);
  }
}

void insert_delay() {
  if (recording) {
    SPrintln("d ", 2);
    delay(300);
  }
}
