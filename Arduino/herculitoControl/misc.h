#ifndef MISC_H_
#define MISC_H_

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

void neopix_init();
void neopix_clear();
void neopix_set_rgb_color(int r, int g, int b);
void neopix_fill_color();
void neopix_next_init();
void neopix_next_led();

class digOutput {
public:
  digOutput(int _pin);
  void On();
  void Off();
private:
  int pin;
};

#endif
