#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include "../Gyro/Gyro_calib.h"

///////////////////////////////////////////////
//PID variables and its gain
///////////////////////////////////////////////
float P_controller_roll = 5.0; //P gain
float I_controller_roll = 0.0; //I gain
float D_controller_roll = 0.0; //D gain
int PID_max_roll = 400;

///////////////////////////////////////////////
//Declare global variables
///////////////////////////////////////////////
float P_controller_pitch = P_controller_roll;
float I_controller_pitch = I_controller_roll;
float D_controller_pitch = D_controller_roll;
int PID_max_pitch = PID_max_roll;

///////////////////////////////////////////////
//Error variables for calculating
///////////////////////////////////////////////
float PID_error_temp;
float I_mem_roll, PID_roll_setpoint = 0, PID_output_roll, PID_last_roll_d_error; //For rolling and its error
float I_mem_pitch, PID_pitch_setpoint = 0, PID_output_pitch, PID_last_pitch_d_error; //for pitching and its error

void PID_calculate()
{
    ///////////////////////////////////////////////
    //Roll calculations
    ///////////////////////////////////////////////

    //Angular error
    PID_error_temp = AngleRoll - PID_roll_setpoint;
    I_mem_roll += I_controller_roll * PID_error_temp; //Sum of errors is Integral
    
    //Set the limit for I controller
    if (I_mem_roll > PID_max_roll) I_mem_roll = PID_max_roll;
    else if (I_mem_roll < PID_max_roll * -1) I_mem_roll = PID_max_roll * -1;

    //PID output as sum of controllers
    PID_output_roll = P_controller_roll * PID_error_temp + I_mem_roll + D_controller_roll * (PID_error_temp - PID_last_roll_d_error);

    //Set the limit for output
    if (PID_output_roll > PID_max_roll) PID_output_roll = PID_max_roll;
    else if (PID_output_roll < PID_max_roll * -1) PID_output_roll = PID_max_roll * -1;

    //Save the last error for D controller
    PID_last_roll_d_error = PID_error_temp;


    ///////////////////////////////////////////////
    //Pitch calculations
    ///////////////////////////////////////////////
    //Angular error
    PID_error_temp = AnglePitch - PID_pitch_setpoint;
    I_mem_pitch += I_controller_pitch * PID_error_temp; //Sum of errors is Integral
    
    //Set the limit for I controller
    if (I_mem_pitch > PID_max_pitch) I_mem_pitch = PID_max_pitch;
    else if (I_mem_pitch < PID_max_pitch * -1) I_mem_pitch = PID_max_pitch * -1;

    //PID output as sum of controllers
    PID_output_pitch = P_controller_pitch * PID_error_temp + I_mem_pitch + D_controller_pitch * (PID_error_temp - PID_last_pitch_d_error);

    //Set the limit for output
    if (PID_output_pitch > PID_max_pitch) PID_output_pitch = PID_max_pitch;
    else if (PID_output_pitch < PID_max_pitch * -1) PID_output_pitch = PID_max_pitch * -1;

    //Save the last error for D controller
    PID_last_pitch_d_error = PID_error_temp;
}