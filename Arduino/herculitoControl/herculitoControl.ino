/****************************************************************
*                                                               *
*    herculitoControl 1.0 - firmware for herculito robot arm    *
*                                                               *
****************************************************************/

/* uses these libraries:
ESP32Servo    - https://github.com/jkb-git/ESP32Servo
PNGdec        - https://github.com/bitbank2/PNGdec
PS4-esp32     - https://github.com/aed3/PS4-esp32
RingBufCPP    - https://github.com/wizard97/Embedded_RingBuf_CPP (included)
TFT_eSPI      - https://github.com/Bodmer/TFT_eSPI
GCodeParser   - https://github.com/tgolla/GCodeParser (modified+included)
SpeedyStepper - https://github.com/Stan-Reifel/SpeedyStepper (modified+included)
*/

#include <Arduino.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include "config.h"
#include "display.h"
#include "misc.h"
#include "pinout.h"
#include "GCodeCommands.h"
#include "StepperControl.h"
#include "move_progs.h"

Display display = Display(3);  // rotate display 270 degrees
digOutput led(LED_PIN);
digOutput buzzer(BUZZER_PIN);

Adafruit_NeoPixel neopix = Adafruit_NeoPixel(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Higher shank
StepperControl StepperA(A_ENABLE_PIN, A_ENDSTOP_PIN, A_STEP_PIN, A_DIR_PIN, STEPPER_A_DIR,
                        STEPPER_A_HOME, STEPPER_A_MAX_STEPS, STEPPER_A_MIN_ESDIST,
                        STEPPER_A_PARK, STEPPER_A_MAX_SPEED, STEPPER_A_ACCELERATION,
                        STEPPER_A_DELAY, STEPS_PER_REVOLUTION_AB);
// Lower shank
StepperControl StepperB(B_ENABLE_PIN, B_ENDSTOP_PIN, B_STEP_PIN, B_DIR_PIN, STEPPER_B_DIR,
                        STEPPER_B_HOME, STEPPER_B_MAX_STEPS, STEPPER_B_MIN_ESDIST,
                        STEPPER_B_PARK, STEPPER_B_MAX_SPEED, STEPPER_B_ACCELERATION,
                        STEPPER_B_DELAY, STEPS_PER_REVOLUTION_AB);
// Rotation
StepperControl StepperC(C_ENABLE_PIN, C_ENDSTOP_PIN, C_STEP_PIN, C_DIR_PIN, STEPPER_C_DIR,
                        STEPPER_C_HOME, STEPPER_C_MAX_STEPS, STEPPER_C_MIN_ESDIST,
                        STEPPER_C_PARK, STEPPER_C_MAX_SPEED, STEPPER_C_ACCELERATION,
                        STEPPER_C_DELAY, STEPS_PER_REVOLUTION_C);
// Rail option
#ifdef STEPPER_D
StepperControl StepperD(D_ENABLE_PIN, D_ENDSTOP_PIN, D_STEP_PIN, D_DIR_PIN, STEPPER_D_DIR,
                        STEPPER_D_HOME, STEPPER_D_MAX_STEPS, STEPPER_D_MIN_ESDIST,
                        STEPPER_D_PARK, STEPPER_D_MAX_SPEED, STEPPER_D_ACCELERATION,
                        STEPPER_D_DELAY, STEPS_PER_MM_RAIL);
#endif

#ifdef DUALSHOCK
#include "dualshock.h"
#endif

#ifdef TH_SERVO
#include "servo_gripper.h"
Servo_Gripper servo_gripper(SERVO_PIN, SERVO1_CENTER_OFFSET, SERVO1_INVERT, SERVO1_HOME);
Servo_Gripper servo_gripper2(SERVO2_PIN, SERVO2_CENTER_OFFSET, SERVO2_INVERT, SERVO2_HOME);
#endif

GCodeCommands Parser = GCodeCommands();

extern bool get_next_move_point();
bool recording = false;

int led_state = LOW;
unsigned long last_millis = 0;
unsigned long interval = 1000;

unsigned long last_millis2 = 0;
unsigned long interval2 = 60;

unsigned long last_millis3 = 0;
unsigned long interval3 = 20;

unsigned long last_millis4 = 0;
unsigned long interval4 = 0;
unsigned long buzzer_on_time = 0;
unsigned long buzzer_off_time = 0;
int buzzer_count = 1;
bool buzzer_active = false;
int buzzer_state = LOW;

bool steppers_running = false;
bool enable_move_queue = false;

bool toggle_neopix = false;
bool ready_printed = false;

void setup() {
  Serial.begin(BAUDRATE);
  display.warning("Booting");
  neopix_init();
#ifdef DUALSHOCK
  setup_dualshock();
#endif
  //buzzer.On();
  delay(600);
  buzzer.Off();
  Serial.println((String)PROG_NAME + " Roboter Arm V" + (String)VERSION);
}

void park_toolhead_all() {
  park_toolhead(true, true, true);
}

void init_home() {
  enable_allOutputs();
  home_all_actuators();
}

void loop() {
  char ch;
  unsigned long current_millis = millis();
#ifdef STEPPER_D
  bool flagA, flagB, flagC, flagD;
#else
  bool flagA, flagB, flagC;
#endif
  if (Serial.available() > 0) {
    ch = Serial.read();
    if ((ch < 32) && (ch != '\r') && (ch != '\n')) {
      // do nothing
    } else if (Parser.AddCharToLine(ch))  // eol? -> parse
    {
      Parser.ParseLine();
      if (!Parser.NoWords()) Parser.processCommand();  // line not empty?
    }
  }

  // main motor driving function
  if (steppers_running) {
    flagB = (StepperB.run() != 0);
    flagA = (StepperA.run() != 0);
    flagC = (StepperC.run() != 0);
#ifdef STEPPER_D
    flagD = (StepperD.run() != 0);
    steppers_running = flagA || flagB || flagC || flagD;
#else
    steppers_running = flagA || flagB || flagC;
#endif
    if (!steppers_running)
      display.update_status();
  } else get_next_move_point();

#ifdef DUALSHOCK
  if (!steppers_running && PS4.isConnected()) {
    int leftSX = abs(PS4.LStickX());
    int leftSY = abs(PS4.LStickY());
    int rightSX = abs(PS4.RStickX());
    int rightSY = abs(PS4.RStickY());

    if ((leftSY > leftSX) && (leftSY > 32)) {
      display.status();
      x_move_joystick();
      display.update_status();
    } else if ((leftSX > leftSY) && (leftSX > 32)) {
      display.status();
      z_move_joystick();
      display.update_status();
    }

    if ((rightSY > rightSX) && (rightSY > 32)) {
      display.status();
      y_move_joystick();
      display.update_status();
    } else if ((rightSX > rightSY) && (rightSX > 32)) {
      display.status();
      u_move_joystick();
      display.update_status();
    }

    if (PS4.L2Value() > 16) {
      display.status();
      servo_gripper.open_keys(true);
      display.update_status();
    } else if (PS4.R2Value() > 16) {
      display.status();
      servo_gripper.open_keys(false);
      display.update_status();
    }

    if (memcmp(&PS4.data.button, &old_button, sizeof(old_button))) {  // any button pressed or released
      old_button = PS4.data.button;
      if (PS4.Share())
        park_toolhead_all();
      else if (PS4.Options())
        init_home();
      if (PS4.Touchpad())
        toggle_recording();
      if (PS4.L3())
        insert_point();
      if (PS4.R3())
        insert_delay();
    }

    if (current_millis - last_millis2 >= interval2) {  // ~20 times per second
      last_millis2 = current_millis;
      if (PS4.L1()) {
        display.status();
        servo_gripper2.turn_keys(false);
        display.update_status();
      }
      if (PS4.R1()) {
        display.status();
        servo_gripper2.turn_keys(true);
        display.update_status();
      }
      // left direction buttons
      if (PS4.Left()) {
        display.status();
        z_move_keys(true);
        display.update_status();
      }
      if (PS4.Right()) {
        display.status();
        z_move_keys(false);
        display.update_status();
      }

      if (PS4.Up()) {
        display.status();
        x_move_keys(true);
        display.update_status();
      }
      if (PS4.Down()) {
        display.status();
        x_move_keys(false);
        display.update_status();
      }
      // right direction buttons
      if (PS4.Triangle()) {
        display.status();
        //xy_move_relative(0, 1.0);
        y_move_keys(true);
        display.update_status();
      }
      if (PS4.Cross()) {
        display.status();
        //xy_move_relative(0, -1.0);
        y_move_keys(false);
        display.update_status();
      }
      if (PS4.Circle()) {
        display.status();
        u_move_keys(true);
        display.update_status();
      }
      if (PS4.Square()) {
        display.status();
        u_move_keys(false);
        display.update_status();
      }
    }
  }

  if (dualshock_connected && !PS4.isConnected()) {
    Serial.println("m Dualshock Controller got disconnected");
    display.error("Remote OFF");
    dualshock_connected = 0;
    remove_bonded_devices();
  }
#endif

  if (current_millis - last_millis3 >= interval3) {  // ~20 times per second
    last_millis3 = current_millis;                   // restart timer

    if (buzzer_active) {
      if (current_millis - last_millis4 >= interval4) {
        last_millis4 = current_millis;  // restart timer
        if (buzzer_state == LOW) {
          buzzer_state = HIGH;
          buzzer.On();
          interval4 = buzzer_on_time;
        } else {
          buzzer_state = LOW;
          buzzer.Off();
          interval4 = buzzer_off_time;
          buzzer_count--;
        }
      }
      if (buzzer_count < 1)
        buzzer_active = false;
    }

    if (toggle_neopix)
      neopix_next_led();

    if (current_millis - last_millis >= interval) {  // ~1 time per second
      long aa, bb, cc, dd;
      last_millis = current_millis;  // restart timer
      if (led_state == LOW) {
        led_state = HIGH;
        led.On();
        display.life_beat();
        interval = 50;
      } else {
        led_state = LOW;
        led.Off();
        interval = 950;
      }
    }
    if (!ready_printed)
      display.message("Ready");
    ready_printed = true;
  }
}
