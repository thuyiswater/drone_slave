    #include <Wire.h>
    #include <Arduino.h>
    #include <Adafruit_MPU6050.h>  
    #include <ESP32Servo.h>
    #include <BasicLinearAlgebra.h>
    #include "../Wifi_receiver/Slave_esp_wifi.h"

    float RateRoll, RatePitch, RateYaw;
    float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
    int RateCalibrationNumber;
    uint32_t LoopTimer;
    float DesiredRateRoll, DesiredRatePitch,DesiredRateYaw;
    float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
    float InputRoll, InputThrottle, InputPitch, InputYaw;
    float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
    float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
    float PIDReturn[]={0, 0, 0};
    float difference_Dist = (float) 180 / (float) 127;

    float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
    float AccX, AccY, AccZ;
    float AngleRoll, AnglePitch;
    float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
    float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
    float Kalman1DOutput[]={0,0};

    float DesiredAngleRoll, DesiredAnglePitch;
    float ErrorAngleRoll, ErrorAnglePitch;
    float PrevErrorAngleRoll, PrevErrorAnglePitch;
    float PrevItermAngleRoll, PrevItermAnglePitch;

    //PID gains for velocicty (rate)
    float PRateRoll = 0.6; float PRatePitch=PRateRoll; float PRateYaw = 2;
    float IRateRoll = 3.5; float IRatePitch=IRateRoll; float IRateYaw = 12;
    float DRateRoll = 0.03; float DRatePitch=DRateRoll; float DRateYaw = 0;

    //PID gains for position (angle)
    float PAngleRoll = 3.78533; float PAnglePitch = PAngleRoll;
    float IAngleRoll = 0.24066; float IAnglePitch = IAngleRoll;
    float DAngleRoll = 0.38672; float DAnglePitch = DAngleRoll;

//Motor setup
    #define EscPin_LeftFront 4
    #define EscPin_LeftBack 23
    #define EscPin_RightFront 5
    #define EscPin_RightBack 18
    Servo ESC1, ESC2, ESC3, ESC4;

void system_setup(){
    Wire.setClock(400000);
    Wire.begin();
    // delay(250);
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
    KalmanState=KalmanState+0.004*KalmanInput;
    KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
    KalmanState=KalmanState+KalmanGain * (
    KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0]=KalmanState; 
    Kalman1DOutput[1]=KalmanUncertainty;
}

void gyro_signals(void)
{
    Wire.beginTransmission(0x68);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(); 
    Wire.requestFrom(0x68,6);

    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(0x68);
    Wire.write(0x1B); 
    Wire.write(0x8);
    Wire.endTransmission();                                                   
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68,6);

    int16_t GyroX=Wire.read()<<8 | Wire.read();
    int16_t GyroY=Wire.read()<<8 | Wire.read();
    int16_t GyroZ=Wire.read()<<8 | Wire.read();

    RateRoll=(float)GyroX/65.5;
    RatePitch=(float)GyroY/65.5;
    RateYaw=(float)GyroZ/65.5;
    AccX=(float)AccXLSB/4096;
    AccY=(float)AccYLSB/4096;
    AccZ=(float)AccZLSB/4096;

    AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
    AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}
void calibration_measurement()
{
    for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
        gyro_signals();
        RateCalibrationRoll+=RateRoll;
        RateCalibrationPitch+=RatePitch;
        RateCalibrationYaw+=RateYaw;
        // delay(1);
    }
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
    
}

void init_ESC(){
ESC1.attach(EscPin_LeftFront,1000,2000); // motor 4
ESC2.attach(EscPin_RightFront,1000,2000); //motor 1
ESC3.attach(EscPin_LeftBack,1000,2000); //motor 3
ESC4.attach(EscPin_RightBack,1000,2000); //motor 2

ESC1.write(0);
ESC2.write(0);
ESC3.write(0);
ESC4.write(0);
}
float ReceiveThrottleInput(){
    //Left JoyStick Control - Throttle
    int MatchingThrottleInput = 0;
    if (LY_joystick_receivedValue <= 10 ) {
        MatchingThrottleInput = 0;
    } else{
        MatchingThrottleInput = LY_joystick_receivedValue* difference_Dist;
    }
    return MatchingThrottleInput;
}
float ReceivePitchInput(){
    //Right JoyStick Control (RY) - Pitch
    int MatchingPitchInput = 0;
    if (RY_joystick_receivedValue <= 10 && RY_joystick_receivedValue >= -2) { 
        MatchingPitchInput = 127;
    } else MatchingPitchInput = 127 + RY_joystick_receivedValue;
    return MatchingPitchInput;
}
float ReceiveRollInput(){
    //Right JoyStick Control (RX) - Roll
    int MatchingRollInput = 0;
    if (RX_joystick_receivedValue <= 10 && RX_joystick_receivedValue >= -2) {
        MatchingRollInput = 127;
    } else MatchingRollInput = 127 + RX_joystick_receivedValue;
    return MatchingRollInput;
}
float ReceiveYawInput(){
    //Right JoyStick Control (RX) - Roll
    int MatchingYawInput = 0;
    if (LX_joystick_receivedValue <= 10 && LX_joystick_receivedValue >= -2) {
        MatchingYawInput = 127;
    } else MatchingYawInput = 127 + LX_joystick_receivedValue;
    return MatchingYawInput;
}
//PID equation for position (angle) and velocity (rate)
void pid_equation(float Error, float P , float I, float D, float PrevError, float PrevIterm)
{
    float Pterm=P*Error; //P controller
    float Iterm=PrevIterm+I*(Error+PrevError)*0.004/2; //I controller

    //Set the limit for I integral controller
    if (Iterm > 400) Iterm=400;
    else if (Iterm <-400) Iterm=-400;

    float Dterm=D*(Error-PrevError)/0.004; //D controller
    float PIDOutput= Pterm+Iterm+Dterm; //PID output is the sum of controllers

    //Set the limit for the PID output
    if (PIDOutput>400) PIDOutput=400;
    else if (PIDOutput <-400) PIDOutput=-400;

    PIDReturn[0]=PIDOutput;
    PIDReturn[1]=Error;
    PIDReturn[2]=Iterm;
}

void reset_pid(void) {
    //Setpoints for velocity
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;

    //Setpoints for position
    PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
    PrevItermAngleRoll=0; PrevItermAnglePitch=0;

}

//////////////////////////////////Setup/////////////////////////////////////


void corrected_values()
{
    gyro_signals();
    RateRoll -= RateCalibrationRoll;
    RatePitch -= RateCalibrationPitch;
    RateYaw -= RateCalibrationYaw;
}

void kalman_1d_roll(){
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll=Kalman1DOutput[0]; KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
}

void kalman_1d_pitch(){
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch=Kalman1DOutput[0]; KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

}

void value_update(){
    
    DesiredAngleRoll=0.10*(ReceiveRollInput()-127);
    DesiredAnglePitch=0.10*(ReceivePitchInput()-127);
    DesiredRateYaw=0.15*(ReceiveYawInput()-127);
    InputThrottle=ReceiveThrottleInput();
    ErrorAngleRoll=DesiredAngleRoll-KalmanAngleRoll;
    ErrorAnglePitch=DesiredAnglePitch-KalmanAnglePitch;
 }

void pid_equation_angleroll(){
    pid_equation(ErrorAngleRoll, PAngleRoll, IAngleRoll, DAngleRoll, PrevErrorAngleRoll,PrevItermAngleRoll);     
    DesiredRateRoll=PIDReturn[0]; 
    PrevErrorAngleRoll=PIDReturn[1];
    PrevItermAngleRoll=PIDReturn[2];
    }

void pid_equation_anglepitch(){
    pid_equation(ErrorAnglePitch, PAnglePitch, IAnglePitch, DAnglePitch, PrevErrorAnglePitch, PrevItermAnglePitch);
    DesiredRatePitch=PIDReturn[0]; 
    PrevErrorAnglePitch=PIDReturn[1];
    PrevItermAnglePitch=PIDReturn[2];
    ErrorRateRoll=DesiredRateRoll-RateRoll;
    ErrorRatePitch=DesiredRatePitch-RatePitch;
    ErrorRateYaw=DesiredRateYaw-RateYaw; 
    }

void pid_equation_rateroll(){
    pid_equation(ErrorRateRoll, PRateRoll, IRateRoll, DRateRoll, PrevErrorRateRoll, PrevItermRateRoll);
    InputRoll=PIDReturn[0];
    PrevErrorRateRoll=PIDReturn[1]; 
    PrevItermRateRoll=PIDReturn[2];
}

void pid_equation_ratepitch(){
    pid_equation(ErrorRatePitch, PRatePitch, IRatePitch, DRatePitch, PrevErrorRatePitch, PrevItermRatePitch);
    InputPitch=PIDReturn[0]; 
    PrevErrorRatePitch=PIDReturn[1]; 
    PrevItermRatePitch=PIDReturn[2];

}

void pid_equation_rateyaw(){
    pid_equation(ErrorRateYaw, PRateYaw,IRateYaw, DRateYaw, PrevErrorRateYaw,PrevItermRateYaw);
    InputYaw=PIDReturn[0]; 
    PrevErrorRateYaw=PIDReturn[1]; 
    PrevItermRateYaw=PIDReturn[2];

}

void control_throttle(){
   
    if (InputThrottle > 160) //80% of total power
    {
        InputThrottle = 160;
    }
    MotorInput1 = 1.0 * (InputThrottle - InputRoll - InputPitch - InputYaw); //8 bits value
    MotorInput2 = 1.0 * (InputThrottle - InputRoll + InputPitch + InputYaw);
    MotorInput3 = 1.0 * (InputThrottle + InputRoll + InputPitch - InputYaw);
    MotorInput4 = 1.0 * (InputThrottle + InputRoll - InputPitch + InputYaw);

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
        
    int ThrottleIdle = 80;
    if (MotorInput1 < ThrottleIdle){
        MotorInput1 = ThrottleIdle;
    }
    
    if (MotorInput2 < ThrottleIdle) {
        MotorInput2 = ThrottleIdle;
    }
    if (MotorInput3 < ThrottleIdle) {
        MotorInput3 = ThrottleIdle;
    }
    if (MotorInput4 < ThrottleIdle) {
        MotorInput4 = ThrottleIdle;
    }

    int ThrottleCutOff = 20;
    if ( InputThrottle < 20) {
        MotorInput1 = ThrottleCutOff; 
        MotorInput2 = ThrottleCutOff;
        MotorInput3 = ThrottleCutOff; 
        MotorInput4 = ThrottleCutOff;
        reset_pid();
  }
    ESC1.write(MotorInput4);
    ESC2.write(MotorInput1);
    ESC3.write(MotorInput3);
    ESC4.write(MotorInput2);
}

void reset_timer(){
    LoopTimer = micros();
}