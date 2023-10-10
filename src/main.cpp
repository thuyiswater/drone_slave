#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Arduino.h>
#include <ESP32Servo.h>

Servo ESC;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;
float AccX_cal = -0.05, AccY_cal = 0.04, AccZ_cal = 0.08;
int EscPin = 5;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 5;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.0;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 0.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint = 0, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint = 0, pid_output_pitch, pid_last_pitch_d_error;

#include <esp_now.h>
#include <WiFi.h>

// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x64,0xB7,0x08,0x60,0x0B,0xB0};

// // Define variables to store BME280 readings to be sent
// float temperature;
// float humidity;
// float pressure;

// Define variables to store incoming readings
float incomingTemp;
float incomingHum;
float incomingPres;
int Controller_PWM;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    float temp;
    float hum;
    float pres;
} struct_message;

// Create a struct_message called BME280Readings to hold sensor readings
struct_message BME280Readings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Controller_PWM, incomingData, sizeof(Controller_PWM));
  // Serial.print("Controller PWM: ");
  // Serial.println(len);
  // incomingTemp = incomingReadings.temp;
  // incomingHum = incomingReadings.hum;
  // incomingPres = incomingReadings.pres;
}
 
void updateDisplay(){
  // Display Readings in Serial Monitor
  Serial.println("INCOMING READINGS");
  Serial.print("Roll Angle: ");
  Serial.print(incomingReadings.temp);
  Serial.println(" º");
  Serial.print("PID: ");
  Serial.print(incomingReadings.hum);
  Serial.println(" ");
  Serial.print("PWM: ");
  Serial.print(incomingReadings.pres);
  Serial.println(" ");
  Serial.println();
}

void gyro_signals(void) {
  // Initiate I2C for MPU
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  // Set acclerometer full scale range
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  // Locate accelerometer data (first data at 0x3B)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  // Request 6 registers of acceleration
  Wire.requestFrom(0x68,6);
  // Acceleration data is divided in halves and combined here
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  // Set gyroscope full scale range
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();
  // Locate gyroscope data (first data at 0x43)                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  // Gyro data is divided in halves of 8bits and combined here
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  // Divide gyro data by LSB sensitivity
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  // Divide acceleration data by LSB sensitivity and add calibrations
  AccX=(float)AccXLSB/4096 + AccX_cal;
  AccY=(float)AccYLSB/4096 + AccY_cal;
  AccZ=(float)AccZLSB/4096 + AccZ_cal;
  // Calculate angle and convert to degrees
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void calculate_pid(){
  //Roll calculations
  // Anglular error
  pid_error_temp = AngleRoll - pid_roll_setpoint;
  // Sum of errors -> Integral
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  // Set limit for i-controller
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  // PID output as sum of controllers
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  // Set limit for output
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  // Save last error for d-controller
  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = AnglePitch - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;
}

void setup() {
  // Start serial with platform baudrate/monitor speed
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  // Set clock to 400kHz (MPU6050 I2C)
  Wire.setClock(400000);
  // MPU startup
  Wire.begin();
  delay(250);
  // Start Gyro in powermode (full 0bits)
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  ESC.attach(EscPin,1000,2000);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  // esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  gyro_signals();
  // Serial.print("Acceleration X [g]= ");
  // Serial.print(AccX);
  // Serial.print(" Acceleration Y [g]= ");
  // Serial.print(AccY);
  // Serial.print(" Acceleration Z [g]= ");
  // Serial.println(AccZ);
  // Serial.print("Acceleration X [g]= ");
  // Serial.print(AccX);
  Serial.print(" Roll Angle [deg]= ");
  Serial.print(AngleRoll);
  // Serial.print(" Pitch angle [deg]= ");
  // Serial.print(AnglePitch);
  calculate_pid();
  Serial.print(" PID Roll = ");
  Serial.print(pid_output_roll);
  // Serial.print(" PID Pitch = ");
  // Serial.print(pid_output_pitch);
  int ControlPWM = 0;
  if (Controller_PWM != 0){
    ControlPWM = Controller_PWM;
  }
  else {
    // Map pid range to motor range
    // ControlPWM = map(pid_output_roll, -400, 400, 0 ,100);
  }
  ESC.write(ControlPWM);
  Serial.print(" PWM = ");
  Serial.println(ControlPWM);

  // // Set values to send
  BME280Readings.temp = AngleRoll;
  BME280Readings.hum = pid_output_roll;
  BME280Readings.pres = ControlPWM;

  // // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BME280Readings, sizeof(BME280Readings));
  delay(50);
}