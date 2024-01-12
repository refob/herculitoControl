#ifndef GCodeCommands_h
#define GCodeCommands_h

#include "GCodeParser.h"

enum resultCodes { undefined,
                   ok,
                   error,
                   busy };

#ifdef STEPPER_D
struct movEvent {
  double x, y, z, u;
  bool linear_move;
};
#else
struct movEvent {
  double x, y, u;
  bool linear_move;
};
#endif
typedef struct movEvent tmovEvent;
typedef struct movEvent* movEventPtr;

#ifdef STEPPER_D
struct stepperLoc {
  unsigned a, b, c, d;
};
#else
struct stepperLoc {
  unsigned a, b, c;
};
#endif
typedef struct stepperLoc tstepperLoc;
typedef struct stepperLoc* stepperLocPtr;

void home_all_steppers();
void park_toolhead(bool motorAB, bool motorC, bool motorD);
void report_coordinates();
void xy_move_relative(double dx, double dy);
void x_move_joystick(void);
void y_move_joystick(void);
void joystick_turn();
void store_P1();
void store_P2();
void restore_P1();
void restore_P2();
void insert_point();
void insert_delay();

class GCodeCommands : public GCodeParser {
public:
  GCodeCommands();
  void processCommand();
  tstepperLoc P1, P2;  // two storepoints
  double xPosition;    // last by gcode set value
  double yPosition;
#ifdef STEPPER_D
  double zPosition;
#endif
  double uPosition;
private:
  bool absoluteMode;  // absolute or relative coordinates?
};

#endif
