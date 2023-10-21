#include <Arduino.h>
#include <ESP32Servo.h>

#include "../Wifi_receiver/Slave_esp_wifi.h"

#define EscPin_LeftFront 4
#define EscPin_LeftBack 21
#define EscPin_RightFront 5
#define EscPin_RightBack 18

Servo ESC1, ESC2, ESC3, ESC4;
float Throttle = 0;
float difference_Dist = (float) 180 / (float) 127;

void init_ESC(){
ESC1.attach(EscPin_LeftBack,1000,2000); 
ESC2.attach(EscPin_LeftFront,1000,2000);
ESC3.attach(EscPin_RightFront,1000,2000);
ESC4.attach(EscPin_RightBack,1000,2000);

ESC1.write(0);
ESC2.write(0);
ESC3.write(0);
ESC4.write(0);
}

void ReceiveThrottleInput(){
      //Left JoyStick Control
      delay (20);
      if (LJSY <= 10) {
        Throttle = 0;
      } else if (LJSY > 10) {
        Throttle = LJSY *  difference_Dist;
      }

      //Right JoyStick Control (P)


      ESC1.write(Throttle);
      ESC2.write(Throttle);
      ESC3.write(Throttle);
      ESC4.write(Throttle);

      Serial.printf("%.2f \n", Throttle);
}