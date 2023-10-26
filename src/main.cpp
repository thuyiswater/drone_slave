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
  // Set up EPS-NOW for slave ESP3
  init_ESPNOW_Slave();
  system_setup();
  calibration_measurement();
  init_ESC();
  LoopTimer=micros();
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
reset_timer();


Serial.print("Motor 1: ");
Serial.print(MotorInput1);

Serial.print("\tMotor 2: ");
Serial.print(MotorInput2);

Serial.print("\tMotor 3: ");
Serial.print(MotorInput3);

Serial.print("\tMotor 4: ");
Serial.print(MotorInput4);
}