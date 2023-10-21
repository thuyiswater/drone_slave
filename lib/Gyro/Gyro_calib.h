#pragma once

// Declare global variables

extern float AccX_calib , AccY_calib, AccZ_calib;
extern float AngleRoll, AnglePitch;
extern float RateRoll, RatePitch, RateYaw;
extern float AccX, AccY, AccZ;

void gyro_calib_signal();
void calibration_measurement();
void corrected_values();