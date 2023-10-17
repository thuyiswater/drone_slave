#include <esp_now.h>
#include <WiFi.h>
// #include <ESP32Servo.h>

// Servo ESC;

int8_t data1 = 0;
int8_t data2 = 0;
// int EscPin = 5;
// int Throttle = 0;
 
// Define a data structure
typedef struct {
  uint8_t a, b;
} UARTmessage;
 
// Create a structured object
UARTmessage UARTData;
 
 
// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&UARTData, incomingData, sizeof(UARTData));
  Serial.print("Data received: ");
  Serial.println(len);
  Serial.print("Received Joystick UART Value: ");
  Serial.println(UARTData.a);
  Serial.print("Received Button UART Value: ");
  Serial.println(UARTData.b);
  Serial.println();
}
 
void init_espnow_receiver()
{
  // Set up Serial Monitor
  Serial.begin(115200);
  
  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
 
  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}















// void UART2ESC()
// {
//   // // This will be moved to slave (doi function do Slave nhan WiFi)
//   // if(SerialPort1.available()) // Check whether data is sent from the Master through UART1
//   // {
//   //   data1 = SerialPort1.read(); // Read the data sent
//   //   Serial.printf("%d\n", data1); // Print out data
//   // }

//   // if (SerialPort2.available()) // Check whether data is sent from the Master through UART2
//   // {
//   //   data2 = SerialPort2.read(); // Read the data sent
//   //   // Serial.printf("%d\n",data);
//   //   if (data2 == 1) // Set condition to increase the Throttle
//   //   {
//   //     Throttle++; // Increase Throttle

//   //     if (Throttle >= 255) // Check the Throttle value if it is more than enough
//   //     {
//   //       Throttle = 255; // Remain the stable value
//   //     }
//   //   };

//   //   if (data2 == 2) // Set condition to decrease the Throttle
//   //   {
//   //     Throttle--; // Decrease Throttle

//   //     if (Throttle <=  0) // Check the Throttle value if it is less than enough
//   //     {
//   //       Throttle = 0; // Remain the stable value
//   //     }
//   //   };
//   // } 
    
//   // Serial.printf("%d\n", Throttle); // Print the Throttle values
//   // int ControlPWM = map(Throttle, 0, 1023, 0 ,255); // Map...
//   // ESC.write(ControlPWM); // ...
// }