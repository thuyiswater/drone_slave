#include <esp_now.h>
#include <WiFi.h>

// MAC address of the sender - Middle ESP32
uint8_t middleMacAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0xDD, 0xD4};

// PMK and LMK keys
static const char* PMK_KEY_STR = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STR = "_SON_DINH_VU_ED3";


// UART Data received from middle ESP32 through ESP_NOW
int8_t LY_joystick_receivedValue;
int8_t RX_joystick_receivedValue;
int8_t RY_joystick_receivedValue;


// Define a testing message structure
typedef struct {

  int8_t LJSY;
  int8_t RJSX;
  int8_t RJSY;
} UARTmessage;

 
// Create a structured object
wifiMessage wifiData;

// Create a middle peer object
esp_now_peer_info_t middlePeer;

// Function to print sender's MAC address on Serial Monitor
void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  Serial.print("Packet received from: ");
  printMAC(middleMacAddress);                /// For debug only ///


  memcpy(&UARTData, incomingData, sizeof(UARTData));
  Serial.print("Left Y: ");
  Serial.println(UARTData.LJSY);
  Serial.print("Right X: ");
  Serial.println(UARTData.RJSX);
  Serial.print("Right Y: ");
  Serial.println(UARTData.RJSY);

  LY_joystick_receivedValue = UARTData.LJSY;


}

void init_espnow_receiver()
{
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Set the PMK key
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

  // Register peer
  memcpy(middlePeer.peer_addr, middleMacAddress, 6);
  middlePeer.channel = 0;
    ///*** Set the middle device's LMK ***///
    for (uint8_t i = 0; i < 16; i++)
    {
      middlePeer.lmk[i] = LMK_KEY_STR[i];
    }
  middlePeer.encrypt = true; // Only middle peer is accessible

  // Add middle peer   
  if (esp_now_add_peer(&middlePeer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}
