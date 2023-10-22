#pragma once

extern float PID_output_roll;
extern float PID_output_pitch;

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