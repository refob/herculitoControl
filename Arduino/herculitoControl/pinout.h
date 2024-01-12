#ifndef PINOUT_H_
#define PINOUT_H_

// CNC Shield V3.0
#define A_STEP_PIN        26 // Step Pulse X-Axis
#define A_DIR_PIN         16 // Direction X-Axis
#define A_ENABLE_PIN       0 // HW-hack - was pin 12
#define A_ENDSTOP_PIN     39 // A5
 
#define B_STEP_PIN        25 // Step Pulse Y-Axis
#define B_DIR_PIN         27 // Direction Y-Axis
#define B_ENABLE_PIN       0 // HW-hack - was pin 12
#define B_ENDSTOP_PIN     36 // A4

#define C_STEP_PIN        17 // Step Pulse Z-Axis
#define C_DIR_PIN         14 // Direction Z-Axis
#define C_ENABLE_PIN       0 // HW-hack - was pin 12
#define C_ENDSTOP_PIN     34 // A3+Coolant

#define D_STEP_PIN        19 // SpnEn
#define D_DIR_PIN         18 // SPnDir
#define D_ENABLE_PIN       0 // HW-hack - was pin 12
#define D_ENDSTOP_PIN     35 // A2+Resume

#define SERVO_PIN         13 // Limit X-Axis+
#define SERVO2_PIN         5 // Limit Y-Axis+
#define BUZZER_PIN        23 // Limit Z-Axis+
#define LED_PIN            2 // Abort - connected to blue LED
#define NEO_PIN           32
#define Free_PIN           4

// TFT_* defined in "...\Arduino\libraries\TFT_eSPI-master\User_Setup"
#define TFT_MOSI          21 
#define TFT_SCLK          22
#define TFT_CS            15
#define TFT_DC            33
#define TFT_RST           12 // not wired, default is 4
#define TFT_MISO          12 // not wired, but definition is required
                             // otherwise eTFT lib will use 19 for MISO which kills D_STEP_PIN 
#endif
