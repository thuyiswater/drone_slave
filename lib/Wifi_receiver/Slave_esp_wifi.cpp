#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// MAC address of the sender - Middle ESP32
uint8_t middleAddress[] = {0x48, 0xE7, 0x29, 0x9F, 0xDD, 0xD4};

// New Slave MAC
uint8_t New_MAC_Address[] = {0xB0, 0xA7, 0x32, 0x16, 0x1E, 0x24};

// PMK and LMK keys
static const char* PMK_KEY_STR = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STR = "_SON_DINH_VU_ED3";

// UART Data received from middle ESP32 through ESP_NOW
int8_t LX_joystick_receivedValue;
int8_t LY_joystick_receivedValue;
int8_t RX_joystick_receivedValue;
int8_t RY_joystick_receivedValue;

// Define a wifi joystick message structure
typedef struct {
  int8_t LJSX;
  int8_t LJSY;
  int8_t RJSX;
  int8_t RJSY;
} joystickMessage;

// Define a wifi GPS message structure
// typedef struct {
//   int8_t Long;
//   int8_t Lat;
// } gpsMessage;
 
// Create a structured object for joystick incoming data
joystickMessage joystickData;

// Create a structured object for GPS sent data
// gpsMessage gpsData;

// Create a middle peer object
esp_now_peer_info_t middlePeer;

// Function to print sender's MAC address on Serial Monitor  /// debug!!! ///
// void printMAC(const uint8_t * mac_addr){
//   char macStr[18];
//   snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
//            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
//   Serial.println(macStr);
// }

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  // Serial.print("Packet received from: ");
  // printMAC(middleMacAddress);                /// For debug only ///

  memcpy(&joystickData, incomingData, sizeof(joystickData));

  LX_joystick_receivedValue = joystickData.LJSX;
  LY_joystick_receivedValue = joystickData.LJSY;
  RX_joystick_receivedValue = joystickData.RJSX;
  RY_joystick_receivedValue = joystickData.RJSY;

//   Serial.print("\n\nLeft X      Left Y      Right X      Right Y: \n");
//   Serial.printf(" %d           %d           %d           %d\n\n\n", LX_joystick_receivedValue, LY_joystick_receivedValue, RX_joystick_receivedValue, RY_joystick_receivedValue);
}

// Callback function executed when data is sent
// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
// {
//   Serial.print("\n\nLast Packet Send Status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery SUCCESS!" : "Delivery FAIL...");
// }

void init_ESPNOW_Slave()
{
  // Set Slave ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Change MAC
  esp_wifi_set_mac(WIFI_IF_STA, New_MAC_Address);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Set the PMK key for Slave ESP32
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

  // Register Middle ESP32 peer
  memcpy(middlePeer.peer_addr, middleAddress, 6);
  middlePeer.channel = 0;

    ///*** Set the Middle ESP32's LMK ***///
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

  // Register callback function for received data
  esp_now_register_recv_cb(OnDataRecv);

  // Register callback function for sent data
  // esp_now_register_send_cb(OnDataSent);
}

// void sendingGPS_throughESPNOW()
// {
//   // Assign structured GPS data
//   gpsData.Long = Longtitude;
//   gpsData.Lat = Latitude;

//   // Send message via ESP-NOW
//   esp_err_t result = esp_now_send(middleAddress, (int8_t *) &gpsData, sizeof(gpsData));
// }