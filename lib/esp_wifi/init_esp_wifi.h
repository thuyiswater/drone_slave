#ifndef DRONE_FUNCTIONS_H
#define DRONE_FUNCTIONS_H

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
// void updateDisplay();
void init_esp();
void update_esp();

#endif 