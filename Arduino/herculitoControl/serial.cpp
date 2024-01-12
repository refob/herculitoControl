#include "serial.h"

void serial_double_print(String msg, double n1, double n2, int len, int digits) {
  char buffer[20];
  Serial.print(msg);
  dtostrf(n1, len, digits, buffer);
  Serial.print(buffer);
  Serial.print(" ");
  dtostrf(n2, len, digits, buffer);
  Serial.print(buffer);
}

void serial_double_println(String msg, double n1, double n2, int len, int digits) {
  char buffer[20];
  Serial.print(msg);
  dtostrf(n1, len, digits, buffer);
  Serial.print(buffer);
  Serial.print(" ");
  dtostrf(n2, len, digits, buffer);
  Serial.println(buffer);
}
