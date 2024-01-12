#include "misc.h"
#include "config.h"

extern Adafruit_NeoPixel neopix;

void neopix_init() {
  neopix.begin();
  neopix.show();  // Initialize all pixels to 'off'
  delay(50);
}

void neopix_clear() {
  neopix.clear();
  neopix.show();
  delay(50);
}

uint32_t neopix_color;
uint32_t black_color;

void neopix_set_rgb_color(int r, int g, int b) {
  neopix_color = neopix.Color(r, g, b);
  black_color = neopix.Color(0, 0, 0);
}

void neopix_fill_color() {
  neopix.fill(neopix_color, 0, NEO_COUNT);
  neopix.show();
  delay(50);
}

int neopix_next;
int neopix_act;

void neopix_next_init() {
  neopix_act = 0;
  neopix_next = 1;
}

void neopix_next_led() {
  if (neopix_next > 0) {
    neopix.setPixelColor(neopix_act, black_color);
    neopix_act = neopix_act + neopix_next;
    neopix.setPixelColor(neopix_act, neopix_color);
    neopix.show();
    if (neopix_act == (NEO_COUNT - 1)) neopix_next = -1;
  } else if (neopix_next < 0) {
    neopix.setPixelColor(neopix_act, black_color);
    neopix_act = neopix_act + neopix_next;
    neopix.setPixelColor(neopix_act, neopix_color);
    neopix.show();
    if (neopix_act == 0) neopix_next = 1;
  }
}

digOutput::digOutput(int _pin) {
  pin = _pin;
  if (pin == -1) return;
  pinMode(pin, OUTPUT);
}

void digOutput::On() {
  if (pin == -1) return;
  digitalWrite(pin, HIGH);
}

void digOutput::Off() {
  if (pin == -1) return;
  digitalWrite(pin, LOW);
}
