#include <LoRa.h>
#include <Wire.h>
#include "fonts.h"
#include <SSD1306Wire.h> //https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <WiFi.h>
#include <WiFiManager.h>
#include <NTPClient.h>
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

#define OLED_RST_PIN 16

const char *conf_ssid = "WaterthingGW";
const char *conf_pass = "5249014578";

char host_name[18] = "WaterthingGW-XXXX"; // last 4 chars will be chip-id

SSD1306Wire oled(0x3c, 4, 15, GEOMETRY_128_64);  // ADDRESS, SDA, SCL
WiFiManager wm;
WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, "europe.pool.ntp.org", 3600, 60000);
WebServer server(80);


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

  //name
  
  //time on the right

  oled.display();
}

void config_ap_callback() {
  //put conf AP credentials on screen
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(OLED_RST_PIN,OUTPUT); 
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
  if (!wm.autoConnect(conf_ssid, conf_pass)) {
    ESP.restart();
  }
  MDNS.begin(host_name);
  MDNS.addService("http", "tcp", 80);
  delay(100);

  //ota
  ArduinoOTA.setHostname(host_name);
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
  server.begin();
  delay(100);

  oled.clear();
  oled.display();
}

void update_display() {
  draw_display_boilerplate();

  
}

void loop() {
  ArduinoOTA.handle();
  ntp.update();
  server.handleClient();
  
  update_display();
  
  delay(10);
}
