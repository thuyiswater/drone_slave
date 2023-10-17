#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Arduino.h>

#include "Gyro_calib.h"
#include "PID_controller.h"


void gyro_printing()
{
    Serial.print("\nRoll rate [degree/s] = ");
    Serial.print(RateRoll);
    Serial.print("\tPitch Rate [degree/s] = ");
    Serial.print(RatePitch);
    Serial.print("\tYaw Rate [degree/s] = ");
    Serial.println(RateYaw);
    Serial.println(); //newline
    delay(600);
}

void angle_printing()
{
    Serial.print ("Roll angle [deg] = ");
    Serial.print (AngleRoll);
    
    Serial.print ("\t\tPID roll angle [deg] = ");
    Serial.print (PID_output_roll);
}