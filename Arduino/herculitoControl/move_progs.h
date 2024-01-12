#ifndef MOVE_PROGS_H_
#define MOVE_PROGS_H_

#include <Arduino.h>

void enable_allOutputs();
void disable_allOutputs();
void home_all_actuators();
void home_toolhead();
void home_AB_steppers();
void home_C_stepper();
void home_D_stepper();
void park_toolhead(bool motorAB, bool motorC, bool motorD);
void moveto_next_point(double x, double y, double z, double u, bool linear_move);

// Moves controlled by joysticks
void x_move_joystick(void);
void y_move_joystick(void);
void z_move_joystick(void);
void u_move_joystick(void);
// moves controlled by keys
void x_move_keys(bool positive);
void y_move_keys(bool positive);
void z_move_keys(bool positive);
void u_move_keys(bool positive);
void x_move(long xdelta);
void y_move(long ydelta);

#ifdef STEPPER_D
void runall(int destA, int destB, int destC, int destD);
#else
void runall(int destA, int destB, int destC, int destD);
#endif
bool wait_until_active_move_is_done();
void toggle_recording();
void insert_point();
void insert_delay();

#endif
