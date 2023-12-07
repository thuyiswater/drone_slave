#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// MAC address of the sender - Middle ESP32
uint8_t middleAddress[] = {0x48, 0xE7, 0x29, 0x9E, 0x94, 0xF8};

// New Slave MAC
uint8_t New_MAC_Address[] = {0x48, 0xE7, 0x29, 0x96, 0x77, 0x44};

// PMK and LMK keys
static const char* PMK_KEY_STR = "_A_H_L_T_T_T_ED3";
static const char* LMK_KEY_STR = "_SON_DINH_VU_ED3";

// UART Data received from middle ESP32 through ESP_NOW
int PWM;
byte X_value;
byte Y_value;
byte leftB;
byte rightB;

// Define a wifi joystick message structure
typedef struct {
  int P;
  byte XJS;
  byte YJS;
  byte LB;
  byte RB;
} Wifi_receivedMessage;
 
// Create a structured object for joystick incoming data
Wifi_receivedMessage wifiData;

// Create a middle peer object
esp_now_peer_info_t masterPeer;

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&wifiData, incomingData, sizeof(wifiData));

  PWM = wifiData.P;
  X_value = wifiData.XJS;
  Y_value = wifiData.YJS;
  leftB = wifiData.LB;
  rightB = wifiData.RB;
}

void init_ESPNOW_Slave()
{
  // Set Slave ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  esp_wifi_set_ps(WIFI_PS_NONE);

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
  memcpy(masterPeer.peer_addr, middleAddress, 6);
  masterPeer.channel = 0;

    ///*** Set the Middle ESP32's LMK ***///
    for (uint8_t i = 0; i < 16; i++)
    {
      masterPeer.lmk[i] = LMK_KEY_STR[i];
    }

  masterPeer.encrypt = true; // Only middle peer is accessible

  // Add middle peer   
  if (esp_now_add_peer(&masterPeer) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register callback function for received data
  esp_now_register_recv_cb(OnDataRecv);

  // Register callback function for sent data
  // esp_now_register_send_cb(OnDataSent);
}

void Print_PS4_Value (){
  Serial.print(" [");
  Serial.printf("%4d", PWM);
  Serial.print ("] ");
  Serial.print("[");
  Serial.printf("%4d", X_value);
  Serial.print ("  ");
  Serial.printf("%4d", Y_value);
  Serial.print("] ");

  Serial.print("[");
  Serial.print(leftB);
  Serial.print ("  ");
  Serial.print(rightB);
  Serial.print ("]\n");
}