#ifndef SERIAL_H_
#define SERIAL_H_

#include <Arduino.h>

#define SPrint(s, v) \
  Serial.print(s); \
  Serial.print(v)
#define SPrint3(a, b, c) \
  Serial.print(a); \
  Serial.print(b); \
  Serial.print(" "); \
  Serial.print(c)
#define SPrint3ln(a, b, c) \
  Serial.print(a); \
  Serial.print(b); \
  Serial.print(" "); \
  Serial.println(c)
#define SPrintln(s, v) \
  Serial.print(s); \
  Serial.println(v)
#define SPrintsln(s) \
  Serial.println(s)

void serial_double_print(String msg, double n1, double n2, int len = 7, int digits = 2);
void serial_double_println(String msg, double n1, double n2, int len = 7, int digits = 2);

#endif
