// #include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include "../lib/ps4_control/ps4_control.h"
#include <PS4Controller.h>
// #include "../lib/esp_wifi/init_esp_wifi.h"

void setup(){
  // Start serial with platform baudrate/monitor speed
  Serial.begin(115200);
  // pinMode(13, OUTPUT);
  // digitalWrite(13, HIGH);
  init_ps4();
  // init_esp();
  PS4.begin();
  Serial.println("Ready.");
}

void loop() {
  checkInput();
}

