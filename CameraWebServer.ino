#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Wire.h>
/*
2021-12-23  jh  angelegt aus Beispiele CamWebserver ind dann aus Link kopiert
                https://forum.arduino.cc/t/problem-with-esp32-cam-pir-motion-connected-to-telegram/879854/25

Resolution einbauen
*/


#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

const char* ssid = "IBH";
const char* password = "owq5rvsygbpe42f4do195m3ht";

String chatId = "1502609650";

// Initialize Telegram BOT
String BOTtoken = "5019330328:AAFMkbtY7ExOxetlNmdIjbbsdTjsgBs9GvA";

bool sendPhoto = false;

WiFiClientSecure clientTCP;

  camera_config_t config;

UniversalTelegramBot bot(BOTtoken, clientTCP);

#define LED 4
#define PIR 2
bool flashState = LOW;

// Motion Sensor
bool motionDetected = false;
bool PirControler = 0;

int botRequestDelay = 1000;   // mean time between scan messages
long lastTimeBotRan;     // last time messages' scan has been done

long sleeptime = 5000;
void startCameraServer();

void handleNewMessages(int numNewMessages);
String sendPhotoTelegram();

// Indicates when motion is detected
static void IRAM_ATTR detectsMovement(void * arg){
  //Serial.println("MOTION DETECTED!!!");
  motionDetected = true;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

//  camera_config_t config;
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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
    delay(1000);
    ESP.restart();
  }
  
  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
//  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_framesize(s, FRAMESIZE_XGA);


  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *) 13);  
  if (err != ESP_OK){
    Serial.printf("handler add failed with error 0x%x \r\n", err); 
  }
  err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
  if (err != ESP_OK){
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  } 
  //initCAM(FRAMESIZE_XGA);

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, flashState);
}

void initCAM (framesize_t x){
  // camera init
    config.frame_size = x;
    config.jpeg_quality = 12;
    config.fb_count = 1;

  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
    delay(1000);
    ESP.restart();
  }

    sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
//  s->set_framesize(s, FRAMESIZE_QVGA);
  s->set_framesize(s, x);
  err = gpio_isr_handler_add(GPIO_NUM_13, &detectsMovement, (void *) 13);  
  if (err != ESP_OK){
    Serial.printf("handler add failed with error 0x%x \r\n", err); 
  }
  err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
  if (err != ESP_OK){
    Serial.printf("set intr type failed with error 0x%x \r\n", err);
  } 

}

void loop() {
  if (sendPhoto){
    Serial.println("Preparing photo");
    sendPhotoTelegram(); 
    sendPhoto = false; 
  }
 
  if(motionDetected){
    bot.sendMessage(chatId, "Motion detected!!", "");
    Serial.println("Motion Detected");
    sendPhotoTelegram();
    motionDetected = false;
  }  
  
  if (millis() > lastTimeBotRan + botRequestDelay){
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages){
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }

  if (PirControler){
    delay(1000);
    if(digitalRead(PIR) == 1){
      digitalWrite(LED, HIGH);
      delay (1000);
      Serial.print("Motion Detected,  Value = ");
      Serial.println(digitalRead(PIR));
      String motion = "Motion detected !\n";
      motion += "You will receive a photo now.\n";
      bot.sendMessage(chatId, motion, "");
      sendPhotoTelegram();
     
    }
  }
  
  if (PirControler){
    if(digitalRead(PIR) == 0) {
      digitalWrite(LED, LOW);
      delay (2000);
    }
  }
  Serial.print("sleep [s] ");
  Serial.println(sleeptime);
  delay (sleeptime);

}

String sendPhotoTelegram(){
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  
  fb = esp_camera_fb_get();             // initiate picture from CAM
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }  
  
  Serial.println("Connect to " + String(myDomain));

  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");
    
    String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + chatId + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--RandomNerdTutorials--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;    
    uint32_t t1 = millis();
  
    clientTCP.println("POST /bot"+BOTtoken+"/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
    clientTCP.println();
    clientTCP.print(head);
  
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0;n<fbLen;n=n+1024) {
      if (n+1024<fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        clientTCP.write(fbBuf, remainder);
      }
    }  
    
    clientTCP.print(tail);
    // calculate expired time
    Serial.printf("Total upload time %lu ms\n", millis() - t1 );    
 
    esp_camera_fb_return(fb);       // release reserved buffer from CAM

      if (PirControler){
    if(digitalRead(PIR) == 0) {
      digitalWrite(LED, LOW);
      delay (2000);
    }
  }

    
    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;
    // receive command from bot
    while ((startTimer + waitTime) > millis()){
      Serial.print(".");
      delay(100);      
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state==true) getBody += String(c);        
        if (c == '\n') {
          if (getAll.length()==0) state=true; 
          getAll = "";
        } 
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length()>0) break;
    }
    clientTCP.stop();
    Serial.println("message: " + getBody);
  }
  else {
    getBody="Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

void handleNewMessages(int numNewMessages){
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++){
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != chatId){
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }
    
    // Print the received message
    String text = bot.messages[i].text;
    Serial.println("Rcvd: " + text);

    String fromName = bot.messages[i].from_name;

    if (text == "/FlashOn") {
      flashState = HIGH;
      digitalWrite(LED, flashState);
    }

    else if (text == "/FlashOff") {
      flashState = LOW;
      digitalWrite(LED, flashState);
    }
    
    else if (text == "/Photo") {
      sendPhoto = true;
      Serial.println("New photo request");
    }
    
    else if (text == "/PIROn"){
      PirControler = 1;
      bot.sendMessage(chatId, "PIR Sensor is ON, you will get a notification if a motion is detected.", "");
    }

    else if (text == "/PIROff"){
      PirControler = 0;
      bot.sendMessage(chatId, "PIR sensor is OFF, you will no longer receive any notification.", "");
    }
    
    else if (text == "/RESXGA"){
      initCAM(FRAMESIZE_XGA);
      bot.sendMessage(chatId, "Resolution set to XVGA.", "");
    }
    else if (text == "/RESQVGA"){
      initCAM(FRAMESIZE_QVGA);
      bot.sendMessage(chatId, "Resolution set to QGA.", "");
    }
    else if (text == "/RESXGA"){
      initCAM(FRAMESIZE_XGA);
      bot.sendMessage(chatId, "Resolution set to QGA.", "");
    }
    else if (text == "/SLEEP30"){
      sleeptime = 30000;
      bot.sendMessage(chatId, "sleeptime set to 30s.", "");
    }
    else if (text == "/SLEEP20"){
      sleeptime = 20000;
      bot.sendMessage(chatId, "sleeptime set to 20s.", "");
    }
    else if (text == "/SLEEP10"){
      sleeptime = 10000;
      bot.sendMessage(chatId, "sleeptime set to 10s.", "");
    }
    else if (text == "/SLEEP05"){
      sleeptime = 5000;
      bot.sendMessage(chatId, "sleeptime set to 5s.", "");
    }

    else if (text == "/start"){
      String welcome = "Hi sir, here are the commands I can execute for you :\n";
      welcome += "/Photo : Take a new photo.\n";
      welcome += "/FlashOn : Turns flash LED ON.\n";
      welcome += "/FlashOff : Turns flash LED Off.\n";
      welcome += "/PIROn : Activate your PIR sensor.\n";
      welcome += "/PIROff : Shut your PIR sensor down.\n";
      // welcome += "/readings : request sensor readings\n\n";
      welcome += "You'll receive a photo whenever motion is detected.\n";
      bot.sendMessage(chatId, welcome, "Markdown");
    }
    else {
      bot.sendMessage(chatId, "I don't understand, sorry. Refer to the commands I showed you above.", "");      
    }
  }
}
