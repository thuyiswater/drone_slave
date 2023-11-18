#pragma once

extern int8_t LY_joystick_receivedValue;
extern int8_t RX_joystick_receivedValue;
extern int8_t RY_joystick_receivedValue;
extern int8_t L1_button_receivedValue;
extern int8_t R1_button_receivedValue;

void init_ESPNOW_Slave();
void Print_PS4_Value ();