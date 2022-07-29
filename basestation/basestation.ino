#include <EEPROM.h>
#include <LoRa.h>
#include <Wire.h>
#include "fonts.h"
#include <OLEDDisplay.h>
#include <SSD1306Wire.h> //https://github.com/ThingPulse/esp8266-oled-ssd1306
#include <WiFi.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <NTPClient.h> //https://github.com/arduino-libraries/NTPClient
#include <WebServer.h>
#include <EMailSender.h> //https://github.com/xreef/EMailSender
#include <SPIFFS.h>
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <map>

#undef LED_BUILTIN
#define LED_BUILTIN 25

#define RED_LED_PIN 12
#define GREEN_LED_PIN 13
#define BLUE_LED_PIN 21

#define LORA_FREQ  869525000 //869.525mhz is allowed to be used at 100mW 10% duty cycle (360 sec an hour) in germany (NO WARRANTY ON THAT!)
#define LORA_TX_PWR 20 // 20dbm/100mW is max
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

SSD1306Wire oled(OLED_ADDRESS, OLED_SDA_PIN, OLED_SCL_PIN, GEOMETRY_128_64/*, I2C_ONE, 200000*/);//sometimes I2C_ONE is not decalred, sometimes it is, idk why
WiFiManager wm;
WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, "europe.pool.ntp.org", 0, 60000);
WebServer server(80);
EMailSender email("", "", "", "", 0);

//conf
struct settings_s {
  char conf_ssid[16] = "WaterthingGW\0\0\0";
  char conf_pass[16] = "524901457882057";
  uint8_t lora_security_key[16] = {0};
  char alert_email[32] = {0};
  char smtp_server[32] = {0};
  char smtp_user[32] = {0};
  char smtp_pass[32] = {0};
  uint16_t smtp_port = 587;
  char web_user[32] = "waterthing";
  char web_pass[32] = "thisisnotsecure";
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
byte lora_last_incoming_message_IDs[16] = {0};
uint8_t lora_last_incoming_message_IDs_idx = 0;

enum lora_packet_types_ws_to_gw { //water system to gateway
  PACKET_TYPE_STATUS = 0,
  PACKET_TYPE_WATER = 1,
  PACKET_TYPE_AUTH_C_REQ = 250,
  PACKET_TYPE_CMD_DISABLED = 253,
  PACKET_TYPE_CMD_AUTH_FAIL = 254,
  PACKET_TYPE_CMD_OK = 255,
};

enum lora_packet_types_gw_to_ws { //gateway to water system
  PACKET_TYPE_ACK = 255
};

//recieved stuff
byte last_wt_status = 0xFF; //left bytes main status, right 4 bytes extra status
uint64_t last_wt_status_millis = 0xFFFFFFFFFFFFFFFF;
uint16_t last_liters_left = 0xFFFF; //0xFFFF means not known
uint16_t last_liters_called = 0xFFFF; //0xFFFF means not known, 0x0000 means done
int16_t last_lora_rssi = -999;
int16_t last_lora_snr = -999;
uint64_t last_recieved_packet_time = 0; //time in unix timestamp

//web
char web_login_cookies[255][32];
uint8_t web_login_cookies_idx = 0;


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
  for (uint8_t b = 1; b <= packet_size; b++) {
    lora_incoming_queue[lora_incoming_queue_idx][b] = LoRa.read();
  }

  last_lora_rssi = LoRa.packetRssi();
  last_lora_snr = LoRa.packetSnr();
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
  oled.clear();
  oled.setColor(WHITE);
  oled.setFont(Lato_Thin_8);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);


  oled.drawString(0, 0, settings.conf_ssid);
  oled.drawString(0, 16, settings.conf_pass);
}

size_t get_charstring_len(char * charstring) {
  for (uint16_t b = 0; b > sizeof(charstring); b++) if (charstring[b] == 0) return b - 1;
}

bool check_auth() {
  String cookie_header = server.header("Cookie");
  uint16_t lc_start = cookie_header.indexOf("login_cookie=");
  uint16_t lc_end = cookie_header.substring(lc_start).indexOf("; ");

  char login_cookie[32]; cookie_header.substring(lc_start, lc_end).toCharArray(login_cookie, 32);

  bool authed = false;
  for (uint16_t c; c <= 255; c++) { //check if login cookie valid
    bool matching = true;
    for (uint8_t b = 0; b < 32; b++) { //compare each byte
      if (web_login_cookies[c][b] != login_cookie[b]) {
        matching = false;
        break;
      }
      if (matching) {
        authed = true;
        break;
      }
    }
    if (authed) break;
  }

  return authed;
}

void rest_status() {
  DynamicJsonDocument stuff(512);
  stuff["status"]["state"] = last_wt_status >> 4;
  stuff["status"]["extra"] = last_wt_status & 0b00001111;
  stuff["status"]["as_text"] = status_to_text[last_wt_status];
  stuff["irrigation"]["left"] = last_liters_left;
  stuff["irrigation"]["called"] = last_liters_called;
  stuff["lora_rx"]["last_rssi"] = last_lora_rssi;
  stuff["lora_rx"]["last_snr"] = last_lora_snr;
  stuff["lora_rx"]["last_packet_time"] = last_recieved_packet_time;
  stuff["lora_tx"]["send_queue_entries"] = -1;
  stuff["lora_tx"]["send_queue_attempts"] = lora_outgoing_queue_tx_attempts;
  char json_stuff[512];
  serializeJsonPretty(stuff, json_stuff);
  server.send(200, "application/json", json_stuff);

}

void rest_login() {
  if (check_auth()) {
    server.send(200, "application/json", "{\"success\":\"Already Authenticated.\"}");
    return;
  }
  bool matching = true;


  DynamicJsonDocument login(128);  
  deserializeJson(login, server.arg("plain"));
  const char * user = server.hasArg("plain") ? login["user"] : server.arg("user").c_str();
  const char * pass = server.hasArg("plain") ? login["pass"] : server.arg("pass").c_str();

  
  uint8_t user_len = get_charstring_len(settings.web_user);
  uint8_t pass_len = get_charstring_len(settings.web_pass);
  if (user_len == strlen(user) and pass_len == strlen(pass)) {
    for (uint8_t b = 0; b < user_len; b++) {
      if (user[b] != settings.web_user[b]) {
        matching = false;
        break;
      }
    }
    for (uint8_t b = 0; b < pass_len; b++) {
      if (pass[b] != settings.web_pass[b]) {
        matching = false;
        break;
      }
    }
  }
  else matching = false;

  if (matching) {
    for (uint8_t b = 0; b < 32; b++) web_login_cookies[web_login_cookies_idx][b] = LoRa.random();
    String cookiestring = "login_cookie=";
    cookiestring += web_login_cookies[web_login_cookies_idx];
    cookiestring += "; Path=/admin/; SameSite=Strict; Max-Age=86400"; //cookie kept for a day

    server.sendHeader("Set-Cookie", cookiestring);
    server.send(200, "application/json", "{\"success\":\"Authenticated\"}");
  }
  else {
    server.send(403, "application/json", "{\"error\":\"Credentials invalid.\"}");
  }
}

void rest_admin_get() {
  if (!check_auth()) {
    server.send(403, "application/json", "{\"error\":403}");
    return;
  }
  uint16_t status_code = 200;
  DynamicJsonDocument resp(512);

  char buf[512];
  serializeJson(resp, buf);
  server.send(status_code, "application/json", buf);
}

void rest_admin_set() {
  if (!check_auth()) {
    server.send(403, "application/json", "{\"error\":403}");
    return;
  }


}

void rest_debug() {
  if (!check_auth()) {
    server.send(403, "application/json", "{\"error\":403}");
    return;
  }
  uint16_t status_code = 200;
  DynamicJsonDocument resp(512);

  char buf[512];
  serializeJson(resp, buf);
  server.send(status_code, "application/json", buf);
}

/*enum mail_alert_enum {
  MAIL_ALERT_WATER,
  MAIL_ALERT_BAT,
  MAIL_ALERT_GENERAL
  };

  void send_email_alert(mail_alert_enum alert_type) {

  }*/

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
  email.setSMTPServer(settings.smtp_server);
  email.setNameFrom(host_name);
  email.setEMailFrom(settings.smtp_user);
  email.setEMailLogin(settings.smtp_user);
  email.setEMailPassword(settings.smtp_pass);
  email.setSMTPPort(settings.smtp_port);
  email.setIsSecure(true);
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
    //LoRa.receive(); //why do i get tons of random garbage?
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
  oled.drawString(0, 12, F("OK"));
  oled.display();
  delay(100);

  //ota
  Serial.println(F("OTA Setup..."));
  oled.clear();
  oled.drawString(0, 0, "OTA...");
  oled.display();
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
  oled.drawString(0, 12, F("OK"));
  oled.display();
  ArduinoOTA.begin();
  delay(100);

  //ntp
  Serial.println(F("NTP Setup..."));
  oled.clear();
  oled.drawString(0, 0, "NTP...");
  oled.display();
  ntp.begin();
  ntp.update();
  oled.drawString(0, 12, F("OK"));
  oled.display();
  delay(100);

  //webserver
  Serial.println(F("WebServer Setup..."));
  oled.clear();
  oled.drawString(0, 0, "WebServer...");
  oled.display();
  SPIFFS.begin();
  server.on("/rest", HTTP_GET, rest_status);
  server.on("/admin/login", HTTP_POST, rest_login);
  server.on("/admin/settings_rest", HTTP_POST, rest_admin_set);
  server.on("/admin/settings_rest", HTTP_GET, rest_admin_get);
  server.on("/admin/debug_rest", HTTP_GET, rest_debug);
  server.on("/", []() { //redirect to index
    server.sendHeader("Location", "/index.html");
    server.send(300, "text/html", "<a href=\"/index.html\">click here</a>");
  });
  server.serveStatic("/", SPIFFS, "/www/");
  for (uint16_t c; c <= 255; c++) for (uint8_t b = 0; b < 32; b++) web_login_cookies[c][b] = LoRa.random(); //reset login cookies
  server.begin();
  oled.drawString(0, 12, F("OK"));
  oled.display();
  delay(100);

  oled.clear();
  oled.display();

  Serial.println(F("Booted"));
  digitalWrite(LED_BUILTIN, HIGH);
}

void update_display() {
  static uint32_t last_disp_update = 0;
  if (millis() - last_disp_update > 1000 / OLED_FPS) {
    draw_display_boilerplate();

    oled.setFont(Lato_Thin_12);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    String status_time = "Last Status ( ";
    status_time += ((millis() - last_wt_status_millis) / 1000) / 60 > 999 ? ">999" : (String)(int)round(((millis() - last_wt_status_millis) / 1000) / 60);
    status_time += "m ago):";
    oled.drawString(63, 12, status_time);
    oled.setFont(Lato_Thin_24);
    oled.drawString(63, 24, status_to_text[last_wt_status]);
    oled.display();

    last_disp_update = millis();
  }
}

void send_ack(byte packet_id) {
  lora_outgoing_queue[lora_outgoing_queue_idx][0] = lora_outgoing_packet_id;
  lora_outgoing_queue[lora_outgoing_queue_idx][1] = PACKET_TYPE_ACK;
  lora_outgoing_queue[lora_outgoing_queue_idx][2] = packet_id;

  lora_outgoing_queue_idx++;
  lora_outgoing_queue_last_tx[lora_outgoing_queue_idx] = millis();
  lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] = 0;
  lora_outgoing_packet_id++;
}

void handle_lora() {
  uint32_t last_lora_tx = 0;

  //recieve
  for (uint8_t p_idx = 0; p_idx < 4; p_idx++) {
    bool is_empty = true;
    for (uint8_t i = 0; i < 48; i++) if (lora_incoming_queue[p_idx][i] != 0) {
        is_empty = false;  //check for data in packet
        break;
      }
    if (!is_empty) {
      Serial.print(F("Recieved LoRa Packet: "));
      for (uint8_t b = 0; b < lora_incoming_queue_len[p_idx]; b++) {
        Serial.print(lora_incoming_queue[p_idx][b], HEX);
        Serial.write(' ');
      }

      if (lora_incoming_queue[p_idx][0] == 42) { //if magic correct
        bool already_recieved = false;
        for (uint8_t i = 0; i < 16; i++) if (lora_incoming_queue[p_idx][0] = lora_last_incoming_message_IDs[i]) already_recieved = true;

        if (!already_recieved) {
          Serial.print(F("Magic Correct.\nPacket type: "));
          switch (lora_incoming_queue[p_idx][2]) {
            case PACKET_TYPE_STATUS:
              Serial.println(F("Status"));
              last_wt_status = lora_incoming_queue[p_idx][3];
              last_wt_status_millis = millis();
              break;

            default:
              break;
          }
          lora_last_incoming_message_IDs[lora_last_incoming_message_IDs_idx] = lora_incoming_queue[p_idx][0];
          if (lora_last_incoming_message_IDs_idx >= 16) lora_last_incoming_message_IDs_idx = 0;
          lora_last_incoming_message_IDs_idx++;
          last_recieved_packet_time = ntp.getEpochTime();
        }
        send_ack(lora_incoming_queue[p_idx][1]); //respond so retransmits wont occur
      }

      for (uint8_t i = 0; i < 48; i++) lora_incoming_queue[p_idx][i] = 0;
    }
  }

  //queue handle
  if (lora_tx_ready) {
    for (uint8_t p_idx = 0; p_idx < 4; p_idx++) {
      bool is_empty = true;
      for (uint8_t i = 0; i < 48; i++) if (lora_outgoing_queue[p_idx][i] != 0) {
          is_empty = false;  //check for data in packet
          break;
        }
      if (!is_empty and millis() - lora_outgoing_queue_last_tx[p_idx] > LORA_RETRANSMIT_TIME and lora_tx_ready) {
        Serial.print(F("Sending LoRa Packet: "));

        LoRa.beginPacket();
        LoRa.write(LORA_MAGIC);
        uint8_t lora_bytes = 0;
        switch (lora_outgoing_queue[p_idx][1]) { // check 2nd byte (packet type)
            /*case :
              lora_bytes = 2;
              break;*/
        }
        for (uint8_t b = 0; b < lora_bytes; b++) {
          LoRa.write(lora_outgoing_queue[p_idx][b]);
          Serial.print(lora_outgoing_queue[p_idx][b], HEX);
          Serial.write(' ');
        }
        LoRa.endPacket(true); //tx in async mode
        lora_tx_ready = false;

        lora_outgoing_queue_last_tx[p_idx] = millis();
        lora_outgoing_queue_tx_attempts[p_idx]++;
        if (lora_outgoing_queue_tx_attempts[p_idx] >= LORA_RETRANSMIT_TRIES) {
          for (uint8_t i = 0; i < 48; i++) lora_outgoing_queue[p_idx][i] = 0; //clear packet if unsuccessful
          lora_outgoing_queue_last_tx[p_idx] = millis();
        }
      }
    }
  }
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  ntp.update();

  update_display();
  handle_lora();

  delay(5);
}
