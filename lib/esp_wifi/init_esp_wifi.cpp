// #include <Arduino.h>
// #include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xB0,0xA7,0x32,0x16,0x1E,0x24};

//Structure example to receive data
//Must match the sender structure
typedef struct struct_message_rcv {
    float roll_angle;
    float pid;
    float pwm;
} struct_message_rcv;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message_snd {
    float motor_FL;
    float motor_FR;
    float motor_BL;
    float motor_BR;
} struct_message_snd;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message_snd motorControls;

// Create a struct_message to hold incoming sensor readings
struct_message_rcv incomingReadings;

esp_now_peer_info_t peerInfo;

// // Callback when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.print("\r\nLast Packet Send Status:\t");
//   Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\t" : "Delivery Fail\t");
// }

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
}

// void updateDisplay(){
//   // Display Readings in Serial Monitor
//   Serial.print("INCOMING READINGS -  ");
//   Serial.print(" Roll Angle: ");
//   Serial.print(incomingReadings.roll_angle);
//   Serial.print("º");
//   Serial.print(" -  PID: ");
//   Serial.print(incomingReadings.pid);
//   Serial.print(" ");
//   Serial.print(" -  PWM: ");
//   Serial.print(incomingReadings.pwm);
//   Serial.print(" ");
//   Serial.println();
// }

void init_esp() {
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  // if (esp_now_init() != ESP_OK) {
  //   Serial.println("Error initializing ESP-NOW");
  //   return;
  // }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  // if (esp_now_add_peer(&peerInfo) != ESP_OK){
  //   Serial.println("Failed to add peer");
  //   return;
  // }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void update_esp(){
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &motorControls, sizeof(motorControls));
  // updateDisplay();
}

