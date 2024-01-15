# herculitoControl
 Arduino ESP32 program which drives a
 [herculito robot arm](https://www.thingiverse.com/thing:6422152)
 
 The GCode syntax that herculitoControl understands is documented in 
 [herculitoControl GCode V1.0.pdf](https://github.com/refob/herculitoControl/blob/main/doc/herculitoControl%20GCode%20V1.0.pdf).

 The remote control usage is described in 
 [Sony Dualshock controller help](https://github.com/refob/herculitoControl/blob/main/doc/Sony%20Dualshock%204%20controller%20help.pdf).

 An easy way to drive the herculito robot arm is to connect the USB port via USB serial to a computer running
 the Windows operating system and to install [YAT](https://sourceforge.net/projects/y-a-terminal/).
 YAT provides a command line to allows you to send text to a serial port. You can also send a
 complete text file in one step to the serial link. This is the way how I created the example
 movie.
 
### herculitoContrl uses these external libraries:
* [ESP32Servo](https://github.com/jkb-git/ESP32Servo)
* [PNGdec](https://github.com/bitbank2/PNGdec)
* [PS4-esp32](https://github.com/aed3/PS4-esp32)
* [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI) - Please review the example [User_Setup.h](/TFT_eSPI_User_Setups/User_Setup.h) file

### Code from these libraries is embedded in the herculitoControl sourceforge
* [RingBufCPP](https://github.com/wizard97/Embedded_RingBuf_CPP)
* [GCodeParser](https://github.com/tgolla/GCodeParser)
* [SpeedyStepper](https://github.com/Stan-Reifel/SpeedyStepper)

### References
* [Intersection of two circles](https://paulbourke.net/geometry/circlesphere/)
* [Intersection of a Line and a Sphere (or circle)](https://paulbourke.net/geometry/circlesphere/)
* [G-code - RepRap](https://www.reprap.org/wiki/G-code)
* [Generate stepper-motor speed profiles in real time](https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/)
* [Stan-Reifel Stepper motor control library](https://github.com/Stan-Reifel/SpeedyStepper)
* [AVR446: Linear speed control of stepper motor](https://ww1.microchip.com/downloads/en/Appnotes/doc8017.pdf)
