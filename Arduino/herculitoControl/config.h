#ifndef CONFIG_H_
#define CONFIG_H_

#define PROG_NAME "HERCULITO"
#define VERSION "1.0"

#define SHANK_LENGTH 150.0 // Code uses the simplification: upper shank == lower shank
#define LEG_LENGTH 285.0 // max. allowed leg extention < 2*SHANK_LENGTH

#define BAUDRATE 115200
#define MAXMOVBUF 50 // size of GCode move point buffer
#define DEFAULT_DELAY 300  // default delay 300ms

#define TH_SERVO        1   // toolhead uses servo
#define DUALSHOCK       1 // Sony Playstation 4 Dualshock bluetooth game controller

#define STEPPER_D       1 // stepper D (=rail drive) exists
#define STEPPER_A_DIR   1 // -1 inverts the stepper direction
#define STEPPER_B_DIR  -1
#define STEPPER_C_DIR  -1
#define STEPPER_D_DIR  -1
#define STEPPER_A_HOME  5400 // steps from 0 to home location
#define STEPPER_B_HOME  2600 // 3600
#define STEPPER_C_HOME  8340
#define STEPPER_D_HOME 15800
#define STEPPER_A_MIN_ESDIST  220 
#define STEPPER_B_MIN_ESDIST    0 // disable
#define STEPPER_C_MIN_ESDIST 2000
#define STEPPER_D_MIN_ESDIST 2000
#define STEPPER_A_PARK  -100 // park position for rail version
#define STEPPER_B_PARK   200
#define STEPPER_C_PARK  3900
#define STEPPER_D_PARK 15000
// maximum number of steps starting from step 0 location
#define STEPPER_A_MAX_STEPS 13700 // 
#define STEPPER_A_MIN_STEPS  3700
#define STEPPER_B_MAX_STEPS  7300  // 7640 
#define STEPPER_C_MAX_STEPS    -1 // no limits due to mechanical construction
#define STEPPER_D_MAX_STEPS 31600 //
#define STEPPER_A_MAX_SPEED 16000
#define STEPPER_B_MAX_SPEED 16000
#define STEPPER_C_MAX_SPEED 12000 // 
#define STEPPER_D_MAX_SPEED 16000
#define STEPPER_A_ACCELERATION  8000 // 
#define STEPPER_B_ACCELERATION  8000 // 
#define STEPPER_C_ACCELERATION  9000
#define STEPPER_D_ACCELERATION 12000
#define STEPPER_A_DELAY 8000 // delay in usec after one step during homing process
#define STEPPER_B_DELAY 8000
#define STEPPER_C_DELAY 1200
#define STEPPER_D_DELAY  600
#define STEPPER_SWITCH_AB 7000 // toggle master engine between A and B

//GEAR RATIO SETTINGS
#define MOTOR_GEAR_TEETH  20 // 
#define MAIN_GEAR_TEETH  140 // ratio = 7.0
#define TURN_GEAR_TEETH  110 // ratio = 5.5

//STEPPER SETTINGS:
#define MICROSTEPS 16 // MICROSTEPPING CONFIGURATION
#define STEPS_PER_REV 200 // NEMA17 STEPS PER REVOLUTION, 1.8° step angle (200 steps/revolution)

// FORMULA: STEPS_PER_REV * MICROSTEPS / MOTOR_GEAR_TEETH / 2
#define STEPS_PER_MM_RAIL 80 // STEPS PER MM FOR RAIL MOTOR
        
// FORMULA: STEPS_PER_DEGREE = MICROSTEPS*STEPS_PER_REV*MAIN_GEAR_TEETH/MOTOR_GEAR_TEETH
#define STEPS_PER_REVOLUTION_AB 22400.0
// FORMULA: STEPS_PER_DEGREE = MICROSTEPS*STEPS_PER_REV*TURN_GEAR_TEETH/MOTOR_GEAR_TEETH
#define STEPS_PER_REVOLUTION_C  17600.0

#ifdef TH_SERVO
#define SERVO1_HOME 0.0
#define SERVO1_CENTER_OFFSET 0.0
#define SERVO1_INVERT 1
#define SERVO2_CENTER_OFFSET 5.0
#define SERVO2_INVERT 0
#define SERVO2_HOME 90.0
#endif

#define NEO_COUNT 30
#define TFT_W 160
#define TFT_H 128

#endif
