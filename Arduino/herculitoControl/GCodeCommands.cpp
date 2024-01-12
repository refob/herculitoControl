#include <Arduino.h>
#include <math.h>
#include <RingBufCPP.h>  // RingBufCPP 1.3.0 by D. Aaron Wisner
#include "config.h"
#include "pinout.h"
#include "display.h"
#include "StepperControl.h"
#include "move_progs.h"
#include "misc.h"
#include "GCodeCommands.h"
#include "GCodeParser.h"
#include "serial.h"
#include "kinematics.h"

#ifdef DUALSHOCK
#include <PS4Controller.h>
#include "dualshock.h"
#endif

#ifdef TH_SERVO
#include "servo_gripper.h"
extern Servo_Gripper servo_gripper;
extern Servo_Gripper servo_gripper2;
#endif

extern GCodeCommands Parser;
extern Display display;
extern digOutput led;
extern digOutput led2;
extern digOutput buzzer;

extern unsigned long buzzer_on_time;
extern unsigned long buzzer_off_time;
extern int buzzer_count;
extern int buzzer_state;
extern bool buzzer_active;

extern StepperControl StepperA;
extern StepperControl StepperB;
extern StepperControl StepperC;
#ifdef STEPPER_D
extern StepperControl StepperD;
#endif
extern bool steppers_running;
extern bool enable_move_queue;
extern double shank_length;
extern bool toggle_neopix;

unsigned long move_sequence_start_time;

struct movEvent ev;
RingBufCPP<struct movEvent, MAXMOVBUF> movbuf;

GCodeCommands::GCodeCommands() {
  absoluteMode = true;
  xPosition = 0.0;
  yPosition = 0.0;
#ifdef STEPPER_D
  zPosition = 0.0;
#endif
  uPosition = 0.0;
  P1.a = P2.a = StepperA.homePosition();
  P1.b = P2.b = StepperB.homePosition();
  P1.c = P2.c = StepperC.homePosition();
#ifdef STEPPER_D
  P1.d = P2.d = StepperD.homePosition();
#endif
}

void empty_move_buffer() {
  struct movEvent e;
  while (movbuf.pull(&e)) {}
}

// activate next move point, if available in buffer
bool get_next_move_point() {
  struct movEvent e;
  if (enable_move_queue) {
    display.update_status();
    return (false);
  }
  if (!movbuf.pull(&e))  // pull next destination point
    return (false);
#ifdef STEPPER_D
  moveto_next_point(e.x, e.y, e.z, e.u, e.linear_move);  // started by steppers_running = true;
#else
  moveto_next_point(e.x, e.y, -1, e.u, e.linear_move);
#endif
  display.update_status();
  return (true);
}

// wait until all moves are finished
bool wait_until_all_moves_are_done() {
  while (!wait_until_active_move_is_done()) {}
  while (!movbuf.isEmpty()) {
    while (!wait_until_active_move_is_done()) {}
  }
  return (true);
}

void dump_move_buffer() {
  struct movEvent *e;
  int maxelem = movbuf.numElements();
  SPrintln("dump_move_buffer: ", maxelem);
  for (int i = 0; i < maxelem; i++) {
    e = movbuf.peek(i);
#ifdef STEPPER_D
    SPrint("nXYZU: ", i);
    SPrint3(" ", e->x, e->y);
    SPrint3ln(" ", e->z, e->u);
#else
    SPrint("nXYU: ", i);
    SPrint3(" ", e->x, e->y);
    SPrintln(" ", e->u);
#endif
  }
}

void GCodeCommands::processCommand() {  // Code command loop
  long int tstart = millis();
  long int tend;
  int result = undefined;
  POINT angs;

#ifdef DEBUG
  SPrintln("\nCommand Line: ", Parser.line);
  Parser.RemoveCommentSeparators();
  SPrintln("Comment(s): ", Parser.comments);
#endif
  if (Parser.HasWord('G'))  // ------------------------------------------- G-Codes
  {
    int gCodeNumber = (int)Parser.GetWordValue('G');
    double tempX = xPosition;
    double tempY = yPosition;
#ifdef STEPPER_D
    double tempZ = zPosition;
#endif
    double tempU = uPosition;
    int tempA = -1;
    int tempB = -1;
    int tempC = -1;
    int tempD = -1;

    if (gCodeNumber != 0)
      wait_until_all_moves_are_done();  // empty movbuf before execution of a non G0 command

    switch (gCodeNumber) {  // Read XYZU absolute/relative for G0, G1 and G92
      case 0:
      case 1:
        if (Parser.HasWord('X')) {
          if (absoluteMode)
            tempX = Parser.GetWordValue('X');
          else
            tempX += Parser.GetWordValue('X');
        }
        if (Parser.HasWord('Y')) {
          if (absoluteMode)
            tempY = Parser.GetWordValue('Y');
          else
            tempY += Parser.GetWordValue('Y');
        }
#ifdef STEPPER_D
        if (Parser.HasWord('Z')) {
          if (absoluteMode)
            tempZ = Parser.GetWordValue('Z');
          else
            tempZ += Parser.GetWordValue('Z');
        }
#endif
        if (Parser.HasWord('U')) {
          if (absoluteMode)
            tempU = Parser.GetWordValue('U');
          else
            tempU += Parser.GetWordValue('U');
        }
        break;
    }

    switch (gCodeNumber) {
      case 0:  // G0 – Rapid non-linear Positioning
        display.status();
        if (movbuf.isEmpty())
          move_sequence_start_time = millis();
        ev.x = tempX;  // Just store the new move point in the buffer
        ev.y = tempY;
#ifdef STEPPER_D
        ev.z = tempZ;
#endif
        ev.u = tempU;
        ev.linear_move = false;
        if (!movbuf.add(ev, false))
          result = error;  // should not happend
        xPosition = tempX;
        yPosition = tempY;
#ifdef STEPPER_D
        zPosition = tempZ;
#endif
        uPosition = tempU;
        result = ok;
        break;
      case 1:  // G1 – Linear Positioning
        display.status();
        if (movbuf.isEmpty())
          move_sequence_start_time = millis();
        ev.x = tempX;  // Just store the new move point in the buffer
        ev.y = tempY;
#ifdef STEPPER_D
        ev.z = tempZ;
#endif
        ev.u = tempU;
        ev.linear_move = true;
        if (!movbuf.add(ev, false))
          result = error;  // should not happend
        xPosition = tempX;
        yPosition = tempY;
#ifdef STEPPER_D
        zPosition = tempZ;
#endif
        uPosition = tempU;
        result = ok;
        break;
      case 4:  // G4 -Empty move buffer and then delay
        wait_until_all_moves_are_done();
        if (Parser.HasWord('P'))
          delay(Parser.GetWordValue('P'));  // P milliseconds
        else if (Parser.HasWord('S'))
          delay(1000 * Parser.GetWordValue('S'));                 // S seconds
        tstart = tstart - (move_sequence_start_time - millis());  // add movbuf run time
        result = ok;
        display.update_status();
        break;
      case 6:  // G6 - Direct Stepper Move
        display.status();
        if (absoluteMode) {
          if (Parser.HasWord('A'))
            tempA = Parser.GetWordValue('A');
          if (Parser.HasWord('B'))
            tempB = Parser.GetWordValue('B');
          if (Parser.HasWord('C'))
            tempC = Parser.GetWordValue('C');
#ifdef STEPPER_D
          if (Parser.HasWord('D'))
            tempD = Parser.GetWordValue('D');
#endif
        } else {  // limits are not checked!!!
          if (Parser.HasWord('A'))
            tempA = StepperA.currentPosition() + Parser.GetWordValue('A');
          if (Parser.HasWord('B'))
            tempB = StepperB.currentPosition() + Parser.GetWordValue('B');
          if (Parser.HasWord('C'))
            tempC = StepperC.currentPosition() + Parser.GetWordValue('C');
#ifdef STEPPER_D
          if (Parser.HasWord('D'))
            tempD = StepperD.currentPosition() + Parser.GetWordValue('D');
#endif
        }
        runall(tempA, tempB, tempC, tempD);
        result = ok;
        display.update_status();
        break;
      case 27:  // G27 - Park toolhead
        bool parkAB, parkC, parkD;
        parkAB = parkC = parkD = false;
        if (Parser.HasWord('A'))
          parkAB = true;
        if (Parser.HasWord('B'))
          parkAB = true;
        if (Parser.HasWord('C'))
          parkC = true;
#ifdef STEPPER_D
        if (Parser.HasWord('D'))
          parkD = true;
#endif
        if ((parkAB == false) && (parkC == false) && (parkD == false))
          parkAB = parkC = parkD = true;
        park_toolhead(parkAB, parkC, parkD);
        result = ok;
        break;
      case 28:  // G28 - reset to home position
        bool homeAB, homeC, homeD;
        homeAB = homeC = homeD = false;
        if (Parser.HasWord('A'))
          homeAB = true;
        if (Parser.HasWord('B'))
          homeAB = true;
        if (Parser.HasWord('C'))
          homeC = true;
#ifdef STEPPER_D
        if (Parser.HasWord('D'))
          homeD = true;
#endif
        enable_allOutputs();
#ifdef STEPPER_D
        if (!homeAB && !homeC && !homeD)
          home_all_actuators();
#else
        if (!homeAB && !homeC)
          home_all_actuators();
#endif
        else {
          if (homeAB)
            home_AB_steppers();
          if (homeC)
            home_C_stepper();
#ifdef STEPPER_D
          if (homeD)
            home_D_stepper();
#endif
        }
        absoluteMode = true;
        xPosition = 0.0;
        yPosition = 0.0;
#ifdef STEPPER_D
        zPosition = 0.0;
#endif
        uPosition = 0.0;
        result = ok;
        break;
      case 90:  // G90 - Absolute Positioning.
        absoluteMode = true;
        result = ok;
        break;
      case 91:  // G91 - Incremental Positioning.
        absoluteMode = false;
        result = ok;
        break;
      default:
        result = error;
        break;
    }
  } else if (Parser.HasWord('M'))  // ------------------------------------------- M-Codes
  {
    int mCodeNumber = (int)Parser.GetWordValue('M');
    int gCodeParameter;

    int neoR = 0;
    int neoG = 0;
    int neoB = 0;

    if (mCodeNumber != 0)
      wait_until_all_moves_are_done();  // empty movbuf before execution of an M command
    switch (mCodeNumber) {
      case 0:  // M0 - Stop or Unconditional stop
        empty_move_buffer();
        StepperA.stop();  // stop current move
        StepperB.stop();
        StepperC.stop();
#ifdef STEPPER_D
        StepperD.stop();
#endif
        if (Parser.HasWord('P'))
          delay(Parser.GetWordValue('P'));  // wait P milliseconds
        else if (Parser.HasWord('S'))
          delay(1000 * Parser.GetWordValue('S'));  // wait S seconds
        result = ok;
        break;
      case 3:  // M3 - open toolhead gripper
        display.status();
        result = ok;
        if (Parser.HasWord('P')) {
          servo_gripper.open_absolute(Parser.GetWordValue('P'));
        } else if (Parser.HasWord('S')) {
          servo_gripper.open_relative(Parser.GetWordValue('S'));
        } else result = error;
        display.update_status();
        break;
      case 4:  // M4 - turn toolhead gripper
        display.status();
        result = ok;
        if (Parser.HasWord('P')) {
          servo_gripper2.turn_absolute(Parser.GetWordValue('P'));
        } else if (Parser.HasWord('S')) {
          servo_gripper2.turn_relative(Parser.GetWordValue('S'));
        } else result = error;
        display.update_status();
        break;
      case 5:  // M5 - home toolhead
        servo_gripper.open_absolute(servo_gripper.homepos);
        servo_gripper2.turn_absolute(0);  // pos 0 is
        result = ok;
        break;
      case 17:  // M17 - Enable all stepper motors
        enable_allOutputs();
        result = ok;
        break;
      case 18:  // M18 - Disable all stepper motors
        disable_allOutputs();
        result = ok;
        break;
      case 114:  // M114 - Get Current Position
        if (Parser.HasWord('S')) {
          SPrint("A:", StepperA.currentPosition());
          SPrint(" B:", StepperB.currentPosition());
#ifdef STEPPER_D
          SPrint(" C:", StepperC.currentPosition());
          SPrintln(" D:", StepperD.currentPosition());
#else
          SPrintln(" C:", StepperC.currentPosition());
#endif
        } else if (Parser.HasWord('P')) {
          SPrint("A:", StepperA.currentAngle(360));
          SPrint(" B:", StepperB.currentAngle(360));
#ifdef STEPPER_D
          SPrint(" C:", StepperC.currentAngle(360));
          SPrintln(" D:", StepperD.currentDistance());
#else
          SPrintln(" C:", StepperC.currentAngle(360));
#endif
        } else {
          POINT pt = angles2point(StepperA.currentAngle(TWO_PI), StepperB.currentAngle(TWO_PI));
          SPrint("X:", pt.x);
          SPrint(" Y:", pt.y);
#ifdef STEPPER_D
          SPrint(" Z:", StepperD.currentDistance());
#endif
          SPrintln(" U:", StepperC.currentAngle(360));
        }
        result = ok;
        break;
      case 115:  // M115 - Get Firmware Version and Capabilities
        SPrintsln("=================================");
        SPrintsln((String)PROG_NAME + " - Roboter Arm Control");
        SPrintsln("=================================");
        SPrintln("Version:  ", VERSION);
        if (absoluteMode)
          SPrintsln("Mode:      absolute");
        else
          SPrintsln("Mode:      relative");
#ifdef STEPPER_D
        SPrintsln("Motor(s): 4");
#else
        SPrintsln("Motor(s): 3");
#endif
#ifdef TH_SERVO
        SPrintsln("Gripper1: Servo1");
#endif
#ifdef TH_SERVO2
        SPrintsln("Gripper2: Servo2");
#endif
#ifdef STEPPER_D
        SPrintsln("Endstops: X Y Z U");
#else
        SPrintsln("Endstops: X Y U");
#endif
        SPrint3ln("Shanks:   ", SHANK_LENGTH, "mm");
        SPrintsln("---------------------------------");
        result = ok;
        break;
      case 119:  // M119 - Get Endstop Status
        SPrint("A:", StepperA.endStop());
        SPrint(" B:", StepperB.endStop());
#ifdef STEPPER_D
        SPrint(" C:", StepperC.endStop());
        SPrintln(" D:", StepperD.endStop());
#else
        SPrintln(" C:", StepperC.endStop());
        result = ok;
        break;
#endif
      case 201:  // M201 - Set max. speed and max. acceleration
        if (Parser.HasWord('A')) {
          double val = 100.0 * Parser.GetWordValue('A');
          StepperA.setSpeedAccelerationPercent(val, val);
        }
        if (Parser.HasWord('B')) {
          double val = 100.0 * Parser.GetWordValue('B');
          StepperB.setSpeedAccelerationPercent(val, val);
        }
        if (Parser.HasWord('C')) {
          double val = 100.0 * Parser.GetWordValue('C');
          StepperC.setSpeedAccelerationPercent(val, val);
        }
#ifdef STEPPER_D
        if (Parser.HasWord('D')) {
          double val = 100.0 * Parser.GetWordValue('D');
          StepperD.setSpeedAccelerationPercent(val, val);
        }
#endif
        result = ok;
        break;

      case 870:  // error message
        Parser.RemoveCommentSeparators();
        display.error(Parser.comments);
        result = ok;
        break;
      case 871:  // warning message
        Parser.RemoveCommentSeparators();
        display.warning(Parser.comments);
        result = ok;
        break;
      case 872:  // simple message
        Parser.RemoveCommentSeparators();
        display.message(Parser.comments);
        result = ok;
        break;

      case 880:  // M880 - neopix run light
        neoR = neoG = neoB = 0;
        if (Parser.HasWord('A')) {
          neoR = Parser.GetWordValue('A');
        }
        if (Parser.HasWord('B')) {
          neoG = Parser.GetWordValue('B');
        }
        if (Parser.HasWord('C')) {
          neoB = Parser.GetWordValue('C');
        }
        neopix_set_rgb_color(neoR, neoG, neoB);
        neopix_fill_color();
        result = ok;
        break;

      case 881:  // M880 - neopix run light
        neoR = neoG = neoB = 0;
        if (Parser.HasWord('A')) {
          neoR = Parser.GetWordValue('A');
        }
        if (Parser.HasWord('B')) {
          neoG = Parser.GetWordValue('B');
        }
        if (Parser.HasWord('C')) {
          neoB = Parser.GetWordValue('C');
        }
        neopix_set_rgb_color(neoR, neoG, neoB);
        neopix_next_init();
        toggle_neopix = true;
        result = ok;
        break;

      case 882:  // M880 - toggle neopix off
        toggle_neopix = false;
        neopix_clear();
        result = ok;
        break;
      case 883:
        buzzer_on_time = 900;
        buzzer_off_time = 100;
        buzzer_count = 1;
        buzzer_state = LOW;
        if (Parser.HasWord('A')) {
          buzzer_on_time = Parser.GetWordValue('A');
        }
        if (Parser.HasWord('B')) {
          buzzer_off_time = Parser.GetWordValue('B');
        }
        if (Parser.HasWord('C')) {
          buzzer_count = Parser.GetWordValue('C');
        }
        buzzer_active = true;
        result = ok;
        break;

      case 890:  // M890 - Run User Gcode
        if (Parser.HasWord('S')) {
          gCodeParameter = Parser.GetWordValue('S');
          switch (gCodeParameter) {
            case 1:
              dump_move_buffer();
              break;
            case 2:
              enable_move_queue = true;
              break;
            case 3:
              enable_move_queue = false;
              break;
            case 4:
              break;
            case 5:
              break;
          }
        }
        result = ok;
        break;
      default:
        result = error;
        break;
    }
  } else {
    result = error;
  }

  if (movbuf.numElements() >= MAXMOVBUF - 1) {  // movbuf full?
    if (result != error)
      //SPrint("buffer full - wait", "\n");  // delay until one buffer place is available to execute
      if (!wait_until_active_move_is_done())
        result = error;
  }

  if (result == ok) {
    tend = millis();
    SPrint("ok ", Parser.line);
    SPrintln(" ", tend - tstart);
  } else if (result == error) {
    tend = millis();
    SPrintln("error ", Parser.line);
  }
}
