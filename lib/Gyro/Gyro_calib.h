#pragma once

// Declare global variables
extern float RateRoll;
extern float RatePitch;
extern float RateYaw;

extern float AngleRoll;
extern float AnglePitch;

void gyro_calib_signal();
void calibration_measurement();
void corrected_values();