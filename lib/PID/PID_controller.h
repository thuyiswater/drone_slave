#pragma once

extern float PID_output_roll;
extern float PID_output_pitch;
extern float InputThrottle;
extern float MotorInput4, MotorInput1, MotorInput2, MotorInput3;
extern float InputRoll, InputThrottle, InputPitch, InputYaw;

void init_ESC();
void gyro_calib_signal();
void calibration_measurement();
void corrected_values();
float ReceiveThrottleInput();
float ReceivePitchInput();
float ReceiveRollInput();
void pid_calculate();
float pid_calc_roll();
float pid_calc_pitch();
float pid_calc_yaw();
void control_throttle();
void time_reset();