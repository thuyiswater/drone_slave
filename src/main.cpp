#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <ESP32Servo.h>
#include "../lib/wifi_receiver/slave_esp_wifi.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

#include "../lib/PID/PID_controller.h"
#include "../lib/Gyro/Gyro_calib.h"
#include "../lib/Setup/System_setup.h"
#include "../lib/Printing/Result_printing.h"

// Servo ESC;

// int EscPin = 5;

// void setup()
// {
//   // Set up EPS-NOW for slave ESP32
//   init_espnow();
//   // Check if the data has received yet to initiate the ESP-NOW protocol
//   // ESC.attach(EscPin,1000,2000);
// }

// void loop()
// {
//   // UART2ESC();
// }



/*                    */
/*                    */
/*                    */



void setup()
{
  // Set up EPS-NOW for slave ESP32
  init_espnow_receiver();
  system_setup();
  calibration_measurement();
}

void loop()
{
  gyro_calib_signal();
  corrected_values();
  PID_calculate();
  gyro_printing();
  angle_printing();
}
