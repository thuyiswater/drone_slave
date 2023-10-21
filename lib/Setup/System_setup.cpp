#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Arduino.h>
#include <ESP32Servo.h>

void system_setup()
{
    // pinMode(13, OUTPUT);
    // digitalWrite(13, HIGH);
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}
  
