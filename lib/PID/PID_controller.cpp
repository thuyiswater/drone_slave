#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include "../Wifi_receiver/Slave_esp_wifi.h"


//Gyro setup
float AccX_calib = -0.05, AccY_calib = 0.05, AccZ_calib = 0.08;
float AngleRoll, AnglePitch;
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;

//Motor setup
#define EscPin_LeftFront 4
#define EscPin_LeftBack 23
#define EscPin_RightFront 5
#define EscPin_RightBack 18
Servo ESC1, ESC2, ESC3, ESC4;
float difference_Dist = (float) 180 / (float) 127;


// PID setup
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
int RateCalibrationNumber;

uint32_t LoopTimer;

float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
float InputRoll, InputThrottle, InputPitch, InputYaw;
float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;

float PIDReturn[]={0, 0, 0};


// float PRateRoll=1.5; float PRatePitch=PRateRoll; float PRateYaw=0;

// float IRateRoll=2.03 ; float IRatePitch=IRateRoll; float IRateYaw=0;

// float DRateRoll=0.01 ; float DRatePitch=DRateRoll; float DRateYaw=0;


float PRateRoll=0.64; float PRatePitch=PRateRoll; float PRateYaw=0;

float IRateRoll=1.03 ; float IRatePitch=IRateRoll; float IRateYaw=0;

float DRateRoll=0.01 ; float DRatePitch=DRateRoll; float DRateYaw=0;

float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void init_ESC(){
ESC1.attach(EscPin_LeftFront,1000,2000); 
ESC2.attach(EscPin_RightFront,1000,2000);
ESC3.attach(EscPin_LeftBack,1000,2000);
ESC4.attach(EscPin_RightBack,1000,2000);

ESC1.write(0);
ESC2.write(0);
ESC3.write(0);
ESC4.write(0);
}

float ReceiveThrottleInput(){
      //Left JoyStick Control - Throttle
      int MatchingThrottleInput = 0;
      if (LY_joystick_receivedValue <= 10) {
        MatchingThrottleInput = 0;
      } else if (LY_joystick_receivedValue > 10) {
        MatchingThrottleInput = LY_joystick_receivedValue *  difference_Dist;
      }
      return MatchingThrottleInput;
}
float ReceivePitchInput(){
      //Right JoyStick Control (RY) - Pitch
      int MatchingPitchInput = 0;
        if (RY_joystick_receivedValue <= 10 || RY_joystick_receivedValue >= -2) { 
        MatchingPitchInput = 127;
      } else MatchingPitchInput = 127 + RY_joystick_receivedValue;
      return MatchingPitchInput;
}
float ReceiveRollInput(){
      //Right JoyStick Control (RX) - Roll
      int MatchingRollInput = 0;
          if (RX_joystick_receivedValue <= 10 || RX_joystick_receivedValue >= -2) {
        MatchingRollInput = 127;
      } else MatchingRollInput = 127 + RX_joystick_receivedValue;
      return MatchingRollInput;
}
float ReceiveYawInput(){
      //Right JoyStick Control (RX) - Roll
      int MatchingYawInput = 0;
          if (LX_joystick_receivedValue <= 10 || LX_joystick_receivedValue >= -2) {
        MatchingYawInput = 127;
      } else MatchingYawInput = 127 + LX_joystick_receivedValue;
      return MatchingYawInput;
}

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

void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm) {
  float Pterm=P*Error;
  float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2;

  if (Iterm > 40000) Iterm = 40000; //micro sec
  else if (Iterm <-40000) Iterm =- 40000;

  float Dterm=D*(Error-PrevError)/0.004;
  float PIDOutput= Pterm+Iterm+Dterm;

  if (PIDOutput>40000) PIDOutput=40000;
  else if (PIDOutput <-40000) PIDOutput=-40000;

  PIDReturn[0]=PIDOutput;
  PIDReturn[1]=Error;
  PIDReturn[2]=Iterm;
}

void reset_pid(void) {
  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
}

void reset_timer(){
    LoopTimer=micros();
}

void pid_calculate(){
    RateRoll-=RateCalibrationRoll;
    RatePitch-=RateCalibrationPitch;
    RateYaw-=RateCalibrationYaw;


    // DesiredRateRoll=0.15*(ReceiveRollInput() - 127);
    DesiredRateRoll=0.15*(ReceiveRollInput() - 127);
    DesiredRatePitch=0.15*(ReceivePitchInput() - 127);
    DesiredRateYaw=0.15*(ReceiveYawInput() - 127);
    InputThrottle = ReceiveThrottleInput();
   

    ErrorRateRoll=DesiredRateRoll-RateRoll;
    ErrorRatePitch=DesiredRatePitch-RatePitch;
    ErrorRateYaw=DesiredRateYaw-RateYaw;
 }
float pid_calc_roll(){//loop
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll=PIDReturn[0];
    PrevErrorRateRoll=PIDReturn[1]; 
    PrevItermRateRoll=PIDReturn[2];
    return InputRoll;
}
float pid_calc_pitch(){//loop
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch=PIDReturn[0]; 
    PrevErrorRatePitch=PIDReturn[1]; 
    PrevItermRatePitch=PIDReturn[2];
    return InputPitch;
}
float pid_calc_yaw(){ //loop
   pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw,PrevItermRateYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorRateYaw=PIDReturn[1]; 
       PrevItermRateYaw=PIDReturn[2];
    return InputYaw;
} 

void control_throttle(){
   
    if (InputThrottle > 160) //80% of total power
     {
        InputThrottle = 160;
    }
    MotorInput1= 1.024*(InputThrottle-InputRoll-InputPitch-InputYaw); //8 bits value
    MotorInput2= 1.024*(InputThrottle-InputRoll+InputPitch+InputYaw);
    MotorInput3= 1.024*(InputThrottle+InputRoll+InputPitch-InputYaw);
    MotorInput4= 1.024*(InputThrottle+InputRoll-InputPitch+InputYaw);

    if (MotorInput1 > 180){
        MotorInput1 = 180;
        }
    if (MotorInput2 > 180){
        MotorInput2 = 180;
        } 
    if (MotorInput3 > 180){
        MotorInput3 = 180;
        }
    if (MotorInput4 > 180){
        MotorInput4 = 180;
        }
        
    int ThrottleIdle=80;
    if (MotorInput1 < ThrottleIdle){
        MotorInput1 =  ThrottleIdle;
    }
    
    if (MotorInput2 < ThrottleIdle) {
        MotorInput2 =  ThrottleIdle;
    }
    if (MotorInput3 < ThrottleIdle) {
        MotorInput3 =  ThrottleIdle;
    }
    if (MotorInput4 < ThrottleIdle) {
        MotorInput4 =  ThrottleIdle;
    }

    int ThrottleCutOff=20;
    if ( InputThrottle < 20) {
        MotorInput1=ThrottleCutOff; 
        MotorInput2=ThrottleCutOff;
        MotorInput3=ThrottleCutOff; 
        MotorInput4=ThrottleCutOff;
        reset_pid();
  }
    ESC1.write(MotorInput4);
    ESC2.write(MotorInput1);
    ESC3.write(MotorInput3);
    ESC4.write(MotorInput2);
}

void time_reset(){
    while (micros() - LoopTimer < 4000){}
        LoopTimer=micros(); }