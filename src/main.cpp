#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>

// #include "../lib/Gyro/Gyro_calib.h"
#include "../lib/PID/PID_controller.h"
// #include "../lib/Printing/Result_printing.h"
#include "../lib/Setup/System_setup.h"
#include "../lib/Wifi_receiver/Slave_esp_wifi.h"
void setup()
{
  Serial.begin(115200);
  // Set up EPS-NOW for slave ESP3
  init_ESC();
  system_setup();
  calibration_measurement();
  init_espnow_receiver();

}

void loop()
{
  gyro_calib_signal();
  pid_calculate();
  pid_calc_roll();
  pid_calc_pitch();
  pid_calc_yaw();
  control_throttle();
  
  
  // Serial.printf("%0.1f\n", InputThrottle);

  Serial.printf("Motor 1: ");
  Serial.printf("%0.1f\t", MotorInput1);

  Serial.printf("Motor 2: ");
  Serial.printf("%0.1f\t", MotorInput2);

  Serial.printf("Motor 3: ");
  Serial.printf("%0.1f\t", MotorInput3);

  Serial.printf("Motor 4: ");
  Serial.printf("%0.1f\n", MotorInput4);

  //   Serial.printf("Roll: ");
  // Serial.printf("%0.1f\t", InputRoll);

  // Serial.printf("Pitch: ");
  // Serial.printf("%0.1f\t", InputPitch);

  // Serial.printf("Yaw: ");
  // Serial.printf("%0.1f\t\n", InputYaw);


  time_reset();
  
}