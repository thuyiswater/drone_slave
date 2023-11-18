#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <PID_controller.h>
#include <Slave_esp_wifi.h>

Adafruit_MPU6050 mpu;
  
void setup()
{
  Serial.begin(115200);
  while (!Serial);
  // Set up EPS-NOW for slave ESP3
  init_ESPNOW_Slave();
  init_ESC();
  system_setup();
  calibration_measurement(); //mpu
  // calibrate(); //Turn this on when calibrate
}


void loop()
{
corrected_values();
kalman_1d_roll();
kalman_1d_pitch();
value_update();
pid_equation_angleroll();
pid_equation_anglepitch();
pid_equation_rateroll();
pid_equation_ratepitch();
pid_equation_rateyaw();
control_throttle();
// checkInputController();//Turn this on when calibrate

// // Print_PS4_Value ();
SerialDataWrite();

}