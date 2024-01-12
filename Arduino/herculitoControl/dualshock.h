#ifndef DUALSHOCK_H_
#define DUALSHOCK_H

#include <PS4Controller.h>

extern int dualshock_connected;
extern ps4_button_t old_button;

void setup_dualshock();
void remove_bonded_devices();
void onConnect();
void rumble(int left, int right, int tdelay);

#endif
