#include <Arduino.h>
#include "config.h"
#include "display.h"

#ifdef DUALSHOCK
#include <PS4Controller.h>
#include <string.h>
#ifndef ESP32
#error "DUALSHOCK only supported only for ESP32"
#endif
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

extern Display display;

int dualshock_connected = 0;
ps4_button_t old_button;

char *bda2str(const uint8_t *bda, char *str, size_t size) {
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

void remove_bonded_devices() {
  uint8_t pairedDeviceBtAddr[20][6];
  char bda_str[18];
  // Get the numbers of bonded/paired devices in the BT module
  int count = esp_bt_gap_get_bond_device_num();
  if (!count) return;  // do nothing
  esp_err_t tError = esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  if (ESP_OK == tError) {
    for (int i = 0; i < count; i++) {
      Serial.print("m Found bonded device # ");
      Serial.print(i);
      Serial.print(" -> ");
      Serial.print(bda2str(pairedDeviceBtAddr[i], bda_str, 18));
      esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
      if (ESP_OK == tError) {
        Serial.println(" - removed.");
      } else {
        Serial.println(" - failed to remove bonded device!");
      }
    }
  }
}

void rumble(int left, int right, int tdelay) {
  if (tdelay > 1000) tdelay = 1000;
  if (left > 255) left = 255;
  if (right > 255) right = 255;
  PS4.setRumble(right, left);  // weak/ strong rumble
  PS4.sendToController();
  delay(tdelay);
  PS4.setRumble(0, 0);
  PS4.sendToController();
}

void onConnect() {
  Serial.println("m Dualshock controller connected");
  display.message("Remote On");
  if (PS4.isConnected()) {
    PS4.setFlashRate(0, 0);
    PS4.setLed(0, 0, 100);
    PS4.sendToController();
    delay(10);  // avoid buffer overflow (?)
    dualshock_connected = 1;
    if (PS4.Charging()) Serial.println("m The controller is charging");
    if (PS4.Audio()) Serial.println("m The controller has headphones attached");
    if (PS4.Mic()) Serial.println("m The controller has a mic attached");
    /* returns always 0 
    Serial.print("Battery Level: ");
    Serial.println(PS4.Battery());
    */
    old_button = PS4.data.button;
  }
}

void setup_dualshock() {
  char bda_str[18];

  PS4.begin();
  const uint8_t *bda = esp_bt_dev_get_address();  // side effect: set bt addr
  Serial.print("ESP32 bluetooth address: ");
  Serial.println(bda2str(bda, bda_str, 18));
  remove_bonded_devices();
  PS4.attachOnConnect(onConnect);
}

#endif
