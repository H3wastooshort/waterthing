#include <EEPROM.h>
#include <LoRa.h>
#include <Wire.h>
#include "fonts.h"
#include <SSD1306Wire.h> //https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <WiFi.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <NTPClient.h> //https://github.com/arduino-libraries
#include <WebServer.h>
#include <SPIFFS.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

#define LORA_FREQ  869525000 //869.525mhz is allowed to be used at 100mW 10% duty cycle (360 sec an hour) in germany
#define LORA_TX_PWR 20 //20dbm/100mW is max
#define LORA_RETRANSMIT_TIME 5000 //time between retransmit attempts in ms
#define LORA_RETRANSMIT_TRIES 5
#define LORA_MAGIC 42
#define LORA_TX_INTERVAL 300000 //time between beacon broadcasts in ms

#define LORA_CS_PIN 18
#define LORA_RST_PIN 14
#define LORA_DIO0_PIN 26

#define OLED_FPS 30
#define OLED_ADDRESS 0x3c
#define OLED_SDA_PIN 4
#define OLED_SCL_PIN 15
#define OLED_RST_PIN 16

#define EEPROM_SIZE 512

char host_name[18] = "WaterthingGW-XXXX"; // last 4 chars will be chip-id

SSD1306Wire oled(OLED_ADDRESS, OLED_SDA_PIN, OLED_SCL_PIN, GEOMETRY_128_64);
WiFiManager wm;
WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, "europe.pool.ntp.org", 0, 60000);
WebServer server(80);

//conf
struct settings_s {
  char conf_ssid[16] = "WaterthingGW\0\0\0";
  char conf_pass[16] = "524901457882057";
  uint8_t lora_security_key[16];
} settings;

//lora
uint8_t lora_outgoing_packet_id = 0; //increments every packet
byte lora_outgoing_queue[4][48] = {0}; //holds up to 4 messages that are to be sent with max 48 bits. first byte is message id, 2nd is message type, rest is data. if all 0, no message in slot. set to 0 after up to 0 retransmits
uint8_t lora_outgoing_queue_idx = 0; //idx where to write
uint32_t lora_outgoing_queue_last_tx[4] = {0};
uint8_t lora_outgoing_queue_tx_attempts[4] = {0};
bool lora_tx_ready = true;

int16_t last_lora_rssi = -0;
byte lora_incoming_queue[4][48] = {0}; //holds up to 4 messages that are to be sent with max 48 bits.
uint8_t lora_incoming_queue_idx = 0; //idx where to write
uint16_t lora_incoming_queue_len[4] = {0}; //lengths of recieved packages

void handle_lora_packet(int packet_size) {
  for (uint8_t b = 0; b <= packet_size; b++) {
    lora_incoming_queue[lora_incoming_queue_idx][b] = LoRa.read();
  }

  lora_incoming_queue_len[lora_incoming_queue_idx] = packet_size;
  lora_incoming_queue_idx++;
  if (lora_incoming_queue_idx++ >= 4) lora_incoming_queue_idx = 0;
}

void handle_lora_tx_done() {
  lora_tx_ready = true;
}

//display
void draw_display_boilerplate() {
  oled.clear();
  oled.setColor(WHITE);
  oled.setFont(Lato_Thin_8);
  oled.drawRect(0, 11, 127, 53);

  //last lora rssi left
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(0, 0, (String)last_lora_rssi);

  //name
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  oled.drawString(63, 0, host_name);

  //time on the right
  oled.setTextAlignment(TEXT_ALIGN_RIGHT);
  char time_buf[5];
  sprintf(time_buf, "%02d:%02d", ntp.getHours(), ntp.getMinutes());
  oled.drawString(127, 0, (String)time_buf);
  
  oled.display();
}

void config_ap_callback() {
  //put conf AP credentials on screen
}

void rest_status() {
  
}

void rest_admin_get() {
  
}

void rest_admin_set() {
  
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, LOW);
  delay(50);
  digitalWrite(OLED_RST_PIN, HIGH);
  delay(50);
  Serial.begin(115200);

  //oled
  Serial.println(F("OLED Setup..."));
  oled.init();
  oled.displayOn();
  oled.clear();
  oled.setColor(WHITE);
  oled.setFont(Lato_Thin_12);
  oled.setContrast(255);
  oled.flipScreenVertically();

  //eeprom setup
  Serial.println(F("EEPROM Setup..."));
  oled.clear();
  oled.drawString(0, 0, "EEPROM...");
  oled.display();
  if (EEPROM.begin(EEPROM_SIZE)) {

  }
  else {

  }

  //lora setup
  Serial.println(F("LoRa Setup..."));
  oled.clear();
  oled.drawString(0, 0, "LoRa...");
  oled.display();
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  LoRa.setSPIFrequency(1E6); //1mhz is way fast
  if (LoRa.begin(LORA_FREQ)) {
    LoRa.idle();
    LoRa.setSyncWord(0x12);
    LoRa.setTxPower(LORA_TX_PWR);
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(8); //sf,bw,cr make a data rate of 366 bits per sec or 45,75 bytes per sec
    LoRa.enableCrc();
    LoRa.onTxDone(handle_lora_tx_done);
    LoRa.onReceive(handle_lora_packet);
    LoRa.receive();
    oled.drawString(0, 12, F("OK"));
    oled.display();
  }
  else {
    oled.drawString(0, 12 , F("ERROR"));
    oled.display();
    Serial.println(F("LoRa not detected!"));
    while (true) delay(10);
  }
  delay(100);

  //wifi connect
  Serial.println(F("Connecting WiFi..."));
  oled.clear();
  oled.drawString(0, 0, "WiFi...");
  oled.display();
  WiFi.mode(WIFI_STA);
  WiFi.hostname(host_name);
  //wm.setAPCallback(config_ap_callback);
  if (!wm.autoConnect(settings.conf_ssid, settings.conf_pass)) {
    ESP.restart();
  }
  MDNS.begin(host_name);
  MDNS.addService("http", "tcp", 80);
  delay(100);

  //ota
  ArduinoOTA.setHostname(host_name);
  ArduinoOTA
  .onStart([]() {
    String ota_type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      ota_type = "sketch";
    else // U_SPIFFS
      ota_type = "filesystem";


    oled.clear();
    oled.setFont(Lato_Thin_24);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    oled.drawString(63,0, "OTA:"); 
    oled.drawString(63,31, ota_type);
    oled.display();
    Serial.println("Start updating " + ota_type);
    delay(1000);
  })
  .onEnd([]() {
    oled.clear();
    oled.setFont(Lato_Thin_24);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    oled.drawString(63,31,"OTA OK!");
    oled.display();
    Serial.println("\nEnd");
    delay(1000);
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    oled.clear();
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    String progress1_str;
    progress1_str += progress;
    progress1_str += '/';
    progress1_str += total;
    String progress2_str;
    uint8_t progress_percent = round(((float)progress / (float)total * 100));
    progress2_str += progress_percent;
    progress2_str += '%';
    oled.setFont(Lato_Thin_12);
    oled.drawString(63, 0, progress1_str);
    oled.setFont(Lato_Thin_24);
    oled.drawString(63, 16, progress2_str);
    oled.drawProgressBar(0, 63 - 16, 127, 16, progress_percent);
    oled.display();

    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    oled.clear();
    oled.setFont(Lato_Thin_24);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    String ota_err;
    oled.drawString(63,0, "Error:"); 
    if (error == OTA_AUTH_ERROR) ota_err = F("Auth");
    else if (error == OTA_BEGIN_ERROR) ota_err = F("Begin");
    else if (error == OTA_CONNECT_ERROR) ota_err = F("Connect");
    else if (error == OTA_RECEIVE_ERROR) ota_err = F("Recieve");
    else if (error == OTA_END_ERROR) ota_err = F("End");
    oled.drawString(63,31, ota_err);

    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");

    oled.display();

    delay(5000);
  });
  ArduinoOTA.begin();

  //ntp
  Serial.println(F("NTP Setup..."));
  oled.clear();
  oled.drawString(0, 0, "NTP...");
  oled.display();
  ntp.begin();
  ntp.update();
  delay(100);


  //webserver
  Serial.println(F("WebServer & Mail Setup..."));
  oled.clear();
  oled.drawString(0, 0, "WebServer&Mail...");
  oled.display();
  SPIFFS.begin();
  server.serveStatic("/", SPIFFS, "/www/");
  server.on("/rest", HTTP_GET, rest_status);
  server.on("/admin/rest", HTTP_POST, rest_admin_set);
  server.on("/admin/rest", HTTP_GET, rest_admin_get);
  server.begin();
  delay(100);

  oled.clear();
  oled.display();
}

void update_display() {
  static uint32_t last_disp_update = 0;
  if (millis() - last_disp_update > /*(1000F / OLED_FPS)*/ 100) {
    draw_display_boilerplate();
    last_disp_update = millis();
  }
}

void loop() {
  ArduinoOTA.handle();
  ntp.update();
  server.handleClient();

  update_display();

  delay(5);
}
