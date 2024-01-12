#ifndef DISPLAY_H_
#define DISPLAY_H_

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <PNGdec.h>
#include "tft_colors.h"

#define MAX_IMAGE_WIDTH 160  // 128

class Display {
public:
  Display(int rotation);
  void herculito();
  void message(const char *msg);
  void warning(const char *msg);
  void error(const char *msg);
  void status();
  void update_status();
  void life_beat();
  TFT_eSPI tft;
  PNG png;
  int16_t xpos;
  int16_t ypos;
  long an;
  long bn;
  long cn;
  long dn;
  double gripon;
  double griprn;
  bool gui_active;
private:
  void output_text(const char *msg);
  long aov;
  long bov;
  long cov;
  long dov; 
  double gripoo;
  double gripro;
  int lifebcnt;
};

#endif