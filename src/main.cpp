#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>

#include "../lib/Gyro/Gyro_calib.h"
#include "../lib/PID/PID_controller.h"
#include "../lib/Printing/Result_printing.h"
#include "../lib/Setup/System_setup.h"
#include "../lib/Throttle_Control/Throttle_Speed.h"
#include "../lib/Wifi_receiver/Slave_esp_wifi.h"

void setup()
{
  Serial.begin(115200);
  // Set up EPS-NOW for slave ESP3 
  // system_setup();
  // calibration_measurement();
  init_ESC();
  init_espnow_receiver();
}

void loop()
{
  // gyro_calib_signal();
  // corrected_values();
  // PID_calculate();
  // gyro_printing();
  // angle_printing();
  ReceiveThrottleInput();
  // Serial.println(button);
  Serial.println(Throttle);
}