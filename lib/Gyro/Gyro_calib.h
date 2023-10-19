#pragma once

// Declare global variables
float RateRoll;
float RatePitch;
float RateYaw;

float AngleRoll;
float AnglePitch;

void gyro_calib_signal();
void calibration_measurement();
void corrected_values();