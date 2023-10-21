#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <Arduino.h>

float AccX_calib = -0.05, AccY_calib = 0.05, AccZ_calib = 0.08;
float AngleRoll, AnglePitch;
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

void gyro_calib_signal()
{
    ///////////////////////////////////////////////
    //Initiate I2C for MPU
    ///////////////////////////////////////////////
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    ///////////////////////////////////////////////
    //Set accelerometer full scale range
    ///////////////////////////////////////////////
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    ///////////////////////////////////////////////
    //Locate accelerometer data (first data at 0x3B)
    ///////////////////////////////////////////////
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();

    ///////////////////////////////////////////////
    //Request 6 registers of acceleration
    ///////////////////////////////////////////////
    Wire.requestFrom(0x68,6);
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    ///////////////////////////////////////////////
    //Set gyroscope full scale range
    ///////////////////////////////////////////////
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();
    
    ///////////////////////////////////////////////
    //Locate gyroscope data (first data at 0x43)
    ///////////////////////////////////////////////
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();

    ///////////////////////////////////////////////
    //Request 6 registers of Gyroscope
    ///////////////////////////////////////////////
    Wire.requestFrom(0x68,6);
    //Gyro data is divided in halves of 8bits and combined here
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();
    
    ///////////////////////////////////////////////
    //Divide gyro data by LSB sensitivity
    ///////////////////////////////////////////////
    RateRoll = (float) GyroX/65.5;
    RatePitch = (float) GyroY/65.6;
    RateYaw = (float) GyroZ/65.5;
        
    ///////////////////////////////////////////////
    //Divide acceleration data by LSB sensitivity and add calibrations
    ///////////////////////////////////////////////
    AccX=(float)AccXLSB/4096 + AccX_calib;
    AccY=(float)AccYLSB/4096 + AccY_calib;
    AccZ=(float)AccZLSB/4096 + AccZ_calib;

    ///////////////////////////////////////////////
    //Calculate angle and convert to degrees
    ///////////////////////////////////////////////
    AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);   
    AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

///////////////////////////////////////////////
//Declare variables
///////////////////////////////////////////////
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;


void calibration_measurement()
{
    for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber ++)
    {
        gyro_calib_signal();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
}

void corrected_values()
{
    RateRoll-= RateCalibrationRoll;
    RatePitch-= RateCalibrationPitch;
    RateYaw-= RateCalibrationYaw;
}


