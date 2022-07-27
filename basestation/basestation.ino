#include <EEPROM.h>
#include <LoRa.h>
#include <Wire.h>
#include "fonts.h"
#include <SSD1306Wire.h> //https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <WiFi.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <NTPClient.h> //https://github.com/arduino-libraries
#include <ESPMail.h> //minimally fixed version: https://github.com/HACKER-3000/ESPMail32
#include <WebServer.h>
#include <SPIFFS.h>
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <map>

#undef LED_BUILTIN
#define LED_BUILTIN 25

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
ESPMail email;

//conf
struct settings_s {
  char conf_ssid[16] = "WaterthingGW\0\0\0";
  char conf_pass[16] = "524901457882057";
  uint8_t lora_security_key[16] = {0};
  char alert_email[32] = {0};
  char smtp_server[32] = {0};
  char smtp_user[32] = {0};
  char smtp_pass[32] = {0};
} settings;

//lora
uint8_t lora_outgoing_packet_id = 0; //increments every packet
byte lora_outgoing_queue[4][48] = {0}; //holds up to 4 messages that are to be sent with max 48 bits. first byte is message id, 2nd is message type, rest is data. if all 0, no message in slot. set to 0 after up to 0 retransmits
uint8_t lora_outgoing_queue_idx = 0; //idx where to write
uint32_t lora_outgoing_queue_last_tx[4] = {0};
uint8_t lora_outgoing_queue_tx_attempts[4] = {0};
bool lora_tx_ready = true;

byte lora_incoming_queue[4][48] = {0}; //holds up to 4 messages that are to be sent with max 48 bits.
uint8_t lora_incoming_queue_idx = 0; //idx where to write
uint16_t lora_incoming_queue_len[4] = {0}; //lengths of recieved packages

//recieved stuff
byte last_wt_status = 0xFF; //left bytes main status, right 4 bytes extra status
uint16_t last_liters_left = 0xFFFF; //0xFFFF means not known
uint16_t last_liters_called = 0xFFFF; //0xFFFF means not known, 0x0000 means done
int16_t last_lora_rssi = -999;
uint64_t last_recieved_packet_time = 0; //time in unix timestamp

std::map<byte, String> status_to_text {
  // SSSSEEEE
  {0b11111111, "Unknown"},
  {0b00000000, "Idle"},
  {0b00000001, "Off"},
  {0b00000010, "Done Today"},
  {0b00000011, "Too Rainy"},
  {0b00010000, "Pumping"},
  {0b00100000, "Emptying"},
  {0b00110000, "Afterdrain"},
  {0b01000000, "NO WATER"},
  {0b01010000, "LOW BAT."},
  {0b01100000, "NO TIME"},
  {0b01110000, "GEN. FAIL"},
  {0b01111000, "TS FAIL"},
  {0b01110110, "RTC FAIL"},
  {0b01110100, "RTC FAIL"},
  {0b01110010, "RTC UNSET"},
  {0b01111100, "TS+RTC FAIL"},
  {0b01111110, "TS+RTC FAIL"},
  {0b01111010, "TS+RTC FAIL"}
};


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

  //oled.display();
}

void config_ap_callback() {
  //put conf AP credentials on screen
}

void rest_status() {
  DynamicJsonDocument stuff(512);
  stuff["status"]["state"] = last_wt_status >> 4;
  stuff["status"]["extra"] = last_wt_status & 0b00001111;
  stuff["status"]["as_text"] = status_to_text[last_wt_status];
  stuff["irrigation"]["left"] = last_liters_left;
  stuff["irrigation"]["called"] = last_liters_called;
  stuff["lora_rx"]["last_rssi"] = last_lora_rssi;
  stuff["lora_rx"]["last_packet_time"] = last_recieved_packet_time;
  stuff["lora_tx"]["send_queue_entries"] = -1;
  stuff["lora_tx"]["send_queue_attempts"] = lora_outgoing_queue_tx_attempts;
  char json_stuff[512];
  serializeJsonPretty(stuff, json_stuff);
  server.send(200, "application/json", json_stuff);

}

void rest_admin_get() {

}

void rest_admin_set() {

}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
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
    EEPROM.get(0, settings);
    oled.drawString(0, 12, F("OK"));
  }
  else {
    EEPROM.put(0, settings);
    oled.drawString(0, 12, F("Initialized"));
  }
  delay(100);

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
    oled.drawString(63, 0, "OTA:");
    oled.drawString(63, 31, ota_type);
    oled.display();
    Serial.println("Start updating " + ota_type);
    delay(1000);
  })
  .onEnd([]() {
    digitalWrite(LED_BUILTIN, HIGH);
    oled.clear();
    oled.setFont(Lato_Thin_24);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    oled.drawString(63, 31, "OTA OK!");
    oled.display();
    Serial.println("\nEnd");
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    static bool led_flip = false;
    digitalWrite(LED_BUILTIN, led_flip);
    led_flip = !led_flip;
    
    /*static uint8_t skipper = 0;
    if (skipper == 0) skipper = 16;
    else {
      skipper--;
      return;
    }*/
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
    digitalWrite(LED_BUILTIN, LOW);
    oled.clear();
    oled.setFont(Lato_Thin_24);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    String ota_err;
    oled.drawString(63, 0, "Error:");
    if (error == OTA_AUTH_ERROR) ota_err = F("Auth");
    else if (error == OTA_BEGIN_ERROR) ota_err = F("Begin");
    else if (error == OTA_CONNECT_ERROR) ota_err = F("Connect");
    else if (error == OTA_RECEIVE_ERROR) ota_err = F("Recieve");
    else if (error == OTA_END_ERROR) ota_err = F("End");
    oled.drawString(63, 31, ota_err);

    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");

    oled.display();

    delay(5000);
    digitalWrite(LED_BUILTIN, HIGH);
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
  Serial.println(F("WebServer Setup..."));
  oled.clear();
  oled.drawString(0, 0, "WebServer...");
  oled.display();
  email.begin();
  SPIFFS.begin();
  server.on("/rest", HTTP_GET, rest_status);
  server.on("/admin/rest", HTTP_POST, rest_admin_set);
  server.on("/admin/rest", HTTP_GET, rest_admin_get);
  server.on("/", []() { //redirect to index
    server.sendHeader("Location", "/index.html");
    server.send(300, "text/html", "<a href=\"/index.html\">click here</a>");
  });
  server.serveStatic("/", SPIFFS, "/www/");
  server.begin();
  delay(100);

  oled.clear();
  oled.display();

  digitalWrite(LED_BUILTIN, HIGH);
}

void update_display() {
  static uint32_t last_disp_update = 0;
  if (millis() - last_disp_update > 1000 / OLED_FPS) {
    draw_display_boilerplate();

    oled.setFont(Lato_Thin_12);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    oled.drawString(63, 12, "Last Status:");
    oled.setFont(Lato_Thin_24);
    oled.drawString(63, 24, status_to_text[last_wt_status]);
    oled.display();

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
