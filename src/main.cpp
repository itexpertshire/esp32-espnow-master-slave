/*
  ESPNow master - slave example to exchange image file
*/

/*
https://github.com/trylaarsdam/pio-tflite-lib

*/

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_camera.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>


// Select camera model
#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_LILY_T_SIM_CAM_V1_2
//#define CAMERA_MODEL_IR
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

#define CHUNK_LENGTH 245

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

std::vector<uint8_t> byte_vector;
uint32_t fpsLastTime = 0;
int nbFrames = 0;
String size="";

// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0xE4, 0x65, 0xB8, 0x20, 0x78, 0x0C}; //esp32 dev board MAC address for receiving E4:65:B8:20:78:0C
String broadcastDevice = "E4:65:B8:20:78:0C";

//esp32 wrover
uint8_t senderAddress[] = {0xD4, 0xD4, 0xDA, 0x5B, 0x04, 0x34}; //esp32 wrover transmitter D4:D4:DA:5B:04:34
String senderDevice = "D4:D4:DA:5B:04:34"; //

//T-SIMCAM
//uint8_t senderAddress[] = {0x34, 0x85, 0x18, 0x8B, 0xD9, 0xE4};
//String senderDevice = "34:85:18:8B:D9:E4"; //T-SIMCAM

// Status to check if receiver received the packet and ready to receive next packet

boolean readyToReceive = true;
#define MAX_RETRY_CNT 10
#define MAX_TIMEOUT_MS 3000
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
uint8_t *incomingReadings;

esp_now_peer_info_t peerInfo;

void updateDisplay();
void getSendPictures();
void sendPacketData(const char* buf, uint16_t len, uint16_t chunkLength);
void sendPacketWithRetry(uint8_t *buf, uint16_t len);
boolean waitForAck();

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail-"+String(status));
  if (status ==0){
    success = "Delivery Success :)";
    readyToReceive = true;
  }
  else{
    success = "Delivery Fail :(";
    readyToReceive = false;
  }
}

void espcamera_init(){
  // Configure the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 5000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.fb_count = 2;
  config.fb_location    = CAMERA_FB_IN_DRAM; //*!< The location where the frame buffer will be allocated */
  config.grab_mode      = CAMERA_GRAB_LATEST;// CAMERA_GRAB_WHEN_EMPTY;  //*!< When buffers should be filled */
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 60;

  #if defined(CAMERA_MODEL_WROVER_KIT) || defined(CAMERA_MODEL_AI_THINKER)
    //config.frame_size = FRAMESIZE_UXGA;
    config.fb_location    = CAMERA_FB_IN_PSRAM; //*!< The location where the frame buffer will be allocated */
    config.grab_mode      = CAMERA_GRAB_LATEST;// CAMERA_GRAB_WHEN_EMPTY;  //*!< When buffers should be filled */
    config.frame_size = FRAMESIZE_240X240;
    config.jpeg_quality = 50;
  #endif
 
  // Camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1200);
    ESP.restart();
  } 

  delay(1200);
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  //memcpy(&incomingReadings, incomingData, len);
  Serial.print("Bytes received: ");
  //Serial.println(len);
  Serial.printf("packet Length: %d\n", len);

  std::vector<uint8_t> temp_vector(&incomingData[0],&incomingData[len]);

  Serial.printf("incomingData[0]: %u \n",incomingData[0]);
  Serial.printf("temp_vector Length: %u\n", temp_vector.size());

      if (len == CHUNK_LENGTH && 
      incomingData[0] == 255 && 
      incomingData[1] == 216 &&
      incomingData[2] == 255) { // FF D8 FF
        byte_vector.clear();
      }

  byte_vector.insert(byte_vector.end(), temp_vector.begin(), temp_vector.end());
  Serial.print("byte_vector Length: ");
  Serial.println(byte_vector.size());

      if (len != CHUNK_LENGTH && 
      incomingData[len - 2] == 255 && 
      incomingData[len - 1] == 217) { // FF D9
        uint8_t* jpgData = byte_vector.data();
        size = byte_vector.size();
        Serial.print("Image size: ");
        Serial.println(size);
        
      
        nbFrames++;
        if (millis() - fpsLastTime >= 1000) {
          //drawingFPSText(nbFrames);
          Serial.println("Frame Rate  "+ String(nbFrames) );
          nbFrames = 0;
          fpsLastTime += 1000;
        }
      }
  
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  if (broadcastDevice == WiFi.macAddress().c_str()){
    // Init OLED display
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
      Serial.println(F("SSD1306 allocation failed"));
      for(;;);
    }
  }

  
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_MODE_STA);
  WiFi.enableLongRange(true);
  Serial.println(WiFi.macAddress());
  
  WiFi.disconnect(); 

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } 
  

 if (senderDevice == WiFi.macAddress().c_str()){
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  espcamera_init();
 }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  if (broadcastDevice == WiFi.macAddress().c_str()){
  // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
  }



  
}
 
void loop() {
  
 
if (senderDevice == WiFi.macAddress().c_str()){
  getSendPictures();
  delay(20000);
}

if (broadcastDevice == WiFi.macAddress().c_str()){

 
 
  updateDisplay();
  delay(1000);
  
}
  



}



void getSendPictures(){
    camera_fb_t* fb = NULL;
    esp_err_t res = ESP_OK;
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      esp_camera_fb_return(fb);
      return;
    }

    if (fb->format != PIXFORMAT_JPEG) {
      Serial.println("PIXFORMAT_JPEG not implemented");
      esp_camera_fb_return(fb);
      return;
    }
    sendPacketData((const char*)fb->buf, fb->len, CHUNK_LENGTH);
    esp_camera_fb_return(fb);
}

void updateDisplay(){
  // Display Readings on OLED Display
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  //display.setTextSize(2);
  display.print("Size: ");
  display.print(size);
  display.setCursor(0, 35);
  display.print("Frame: ");
  display.print(nbFrames);
  display.setCursor(0, 55);
  display.print(success);
  display.display();
  
  
}

boolean waitForAck(){
uint32_t start_time = millis(); 

    while (!readyToReceive) {

      delay(100);
      uint32_t  time_duration = millis() - start_time;
      if (time_duration > MAX_TIMEOUT_MS) break;
    }
     uint32_t  time_duration = millis() - start_time;
     Serial.print(time_duration); Serial.println(" ms for ACK");

  return readyToReceive;

}

void sendPacketWithRetry(uint8_t *buf, uint16_t len){
  int retryCnt = 0;

  while (true){
       if (readyToReceive){
           Serial.printf("Retry Cnt %u\n",retryCnt);
           esp_err_t result = esp_now_send(broadcastAddress, buf, len);

          if (result == ESP_OK) {
              Serial.printf("Sent with success size %u\n",len);
              readyToReceive = false;
              if (waitForAck()) break;  //Wait for success ack from receiver or else retransmit
            }
            else {
              Serial.printf("Error sending the data %u\n",len);
            }

       } 
       readyToReceive=true;
       //Wait for retransmit        
        delay(600);
        retryCnt++;

        if (retryCnt > MAX_RETRY_CNT) break; //Exit loop if max rety exceeded

  } 

}


void sendPacketData(const char* buf, uint16_t len, uint16_t chunkLength) {
  uint8_t *buffer[chunkLength];
  size_t blen = chunkLength;
  size_t rest = len % blen;

  Serial.printf("blen %u\n",blen);
  Serial.printf("len %u\n",len);
  Serial.printf("rest %u\n",rest);

  for (uint8_t i = 0; i < len / blen; ++i) {
    memcpy(&buffer, buf + (i * blen), blen);
    // Send message via ESP-NOW
    sendPacketWithRetry( (uint8_t *) &buffer, chunkLength);
    //break;
  }
 
 
  if (rest) {
    memcpy(&buffer, buf + (len - rest), rest);
        // Send message via ESP-NOW
        sendPacketWithRetry( (uint8_t *) &buffer, rest);
  }
  
}