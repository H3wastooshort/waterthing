//Gateway

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
#include "mbedtls/md.h"

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

SSD1306Wire oled(OLED_ADDRESS, -1, -1, GEOMETRY_128_64/*, I2C_ONE, 200000*/);//sometimes I2C_ONE is not decalred, sometimes it is, idk why
WiFiManager wm;
WiFiUDP ntpUDP;
NTPClient ntp(ntpUDP, "europe.pool.ntp.org", 0, 60000);
WebServer server(80);
EMailSender email("", "", "", "", 0);

//conf
struct settings_s {
  char conf_ssid[16] = "WaterthingGW";
  char conf_pass[16] = "CHANGE_ME_42";
  uint8_t lora_security_key[16] = {0};
  char alert_email[32] = "max.mustermann@example.com";
  char smtp_server[32] = {0};
  char smtp_user[32] = {0};
  char smtp_pass[32] = {0};
  uint16_t smtp_port = 587;
  char web_user[32] = "waterthing";
  char web_pass[32] = "thisisnotsecure";
  uint8_t display_brightness = 255;
} settings;

//lora
uint8_t lora_outgoing_packet_id = 1; //increments every packet
byte lora_outgoing_queue[4][48] = {0}; //holds up to 4 messages that are to be sent with max 48 bits. first byte is magic, 2nd message id, 3nd is message type, rest is data. if all 0, no message in slot. set to 0 after up to 0 retransmits
uint8_t lora_outgoing_queue_idx = 0; //idx where to write
uint64_t lora_outgoing_queue_last_tx = 0;
uint8_t lora_outgoing_queue_tx_attempts[4] = {0};
bool lora_tx_ready = true;

byte lora_incoming_queue[4][48] = {0}; //holds up to 4 messages that are to be sent with max 48 bits.
uint8_t lora_incoming_queue_idx = 0; //idx where to write
uint16_t lora_incoming_queue_len[4] = {0}; //lengths of recieved packages
byte lora_last_incoming_message_IDs[16] = {0};
uint8_t lora_last_incoming_message_IDs_idx = 0;

//shared stuff start
enum lora_packet_types_ws_to_gw { //water system to gateway
  PACKET_TYPE_STATUS = 0,
  PACKET_TYPE_WATER = 1,
  PACKET_TYPE_TEST = 69,
  PACKET_TYPE_REBOOT = 240,
  PACKET_TYPE_WS_ACK = 249,
  PACKET_TYPE_AUTH_CHALLANGE = 250,
  PACKET_TYPE_NO_CHALLANGE = 251,
  PACKET_TYPE_CMD_DISABLED = 253,
  PACKET_TYPE_CMD_AUTH_FAIL = 254,
  PACKET_TYPE_CMD_OK = 255,
};

enum lora_packet_types_gw_to_ws { //gateway to water system
  PACKET_TYPE_CURRENT_TIME = 0,
  PACKET_TYPE_ADD_WATER = 1,
  PACKET_TYPE_CANCEL_WATER = 2,
  PACKET_TYPE_GW_REBOOT = 241,
  PACKET_TYPE_REQUST_CHALLANGE = 250,
  PACKET_TYPE_ACK = 255
};

//length is only packet data. add 3 for real packet size
uint8_t ws_to_gw_packet_type_to_length(uint8_t pt) {
  switch (pt) {
    case PACKET_TYPE_STATUS: return 3; break;
    case PACKET_TYPE_WATER: return 5; break;
    case PACKET_TYPE_TEST: return 5; break;
    case PACKET_TYPE_AUTH_CHALLANGE: return 16; break;
    case PACKET_TYPE_CMD_DISABLED: return 0; break;
    case PACKET_TYPE_CMD_AUTH_FAIL: return 1; break;
    case PACKET_TYPE_CMD_OK: return 1; break;
    case PACKET_TYPE_REBOOT: return 0; break;
    case PACKET_TYPE_WS_ACK: return 1; break;
    default: return 47; break;
  }
}

uint8_t gw_to_ws_packet_type_to_length(uint8_t pt) {
  switch (pt) {
    case PACKET_TYPE_ACK: return 1; break;
    case PACKET_TYPE_REQUST_CHALLANGE: return 0; break;
    case PACKET_TYPE_CURRENT_TIME: return 0; break;
    case PACKET_TYPE_ADD_WATER: return 35; break;
    case PACKET_TYPE_CANCEL_WATER: return 33; break;
    case PACKET_TYPE_GW_REBOOT: return 0; break;
  }
}
//shared stuff end

//recieved stuff
int16_t last_lora_rssi = 0xFFFF;
int16_t last_lora_snr = 0xFFFF;
int32_t last_lora_freq_error = 0xFFFFFFFF;
uint64_t last_recieved_packet_time = 0; //time in unix timestamp

byte last_wt_status = 0xFF; //left bytes main status, right 4 bytes extra status
uint64_t last_wt_status_timestamp = 0;
uint16_t last_liters_left = 0xFFFF; //0xFFFF means not known, 0x0000 means done
uint16_t last_liters_called = 0xFFFF; //0xFFFF means not known, 0x0000 means done
uint64_t last_wt_liters_timestamp = 0;
float last_wt_battery_voltage = -1;
uint64_t last_wt_battery_voltage_timestamp = 0;
uint64_t last_wt_reboot_timestamp = 0;

//authed lora cmds
enum auth_state_e {
  AUTH_STEP_IDLE = 0, //wait for entry in lora_auth_cmd_queue
  AUTH_STEP_TX_CHALLANGE_REQUEST = 1,
  AUTH_STEP_WAIT_CHALLANGE = 2,
  AUTH_STEP_TX_ANSWER = 3,
  AUTH_STEP_WAIT_CMD_SUCCESS = 4,
} auth_state;
byte last_wt_challange[16] = {0}; //if all 0, not valid/non recieved
byte lora_auth_cmd_queue[4][16] = {0}; //the data part of all authed commands to be sent.
uint8_t lora_auth_cmd_queue_idx = 0;
uint8_t lora_auth_packet_processing = 255; //255 means invalid
byte last_auth_cmd_response = 0; //0 means invalid
byte last_auth_challange_packet_id = 0xFF;
uint64_t last_auth_packet_millis = 0;

//web
char web_login_cookies[256][32];
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

//display
enum gw_page_e {PAGE_STATUS = 0, PAGE_WT = 1, PAGE_LORA = 2, PAGE_WIFI = 3};
uint8_t disp_page = 0;
std::map<uint8_t, String> gw_page_to_text{
  {PAGE_STATUS, "Stat"},
  {PAGE_WT, "WT"},
  {PAGE_LORA, "LoRa"},
  {PAGE_WIFI, "WiFi"}
};
uint8_t rx_indicator_blink = 0;
//uint8_t tx_indicator_blink = 0;
uint8_t web_indicator_blink = 0;
uint8_t tx_indicator_pos = 0;

uint64_t last_disp_button_down = 0;
void IRAM_ATTR disp_button_down() {
  if (millis() - last_disp_button_down > 250) disp_page++;
  if (disp_page > 3) disp_page = 0;
  last_disp_button_down = millis();
}


char randomASCII() { //give random lowercase ascii
  switch (random(0, 3)) {
    default:
    case 0:
      return max(min(uint8_t(random(0, 24) + 'a'), uint8_t('z')), uint8_t('a'));
      break;

    case 1:
      return max(min(uint8_t(random(0, 24) + 'A'), uint8_t('Z')), uint8_t('A'));
      break;

    case 2:
      return max(min(uint8_t(random(0, 10) + '0'), uint8_t('9')), uint8_t('0'));
      break;
  }
}

void handle_lora_packet(int packet_size) { //TODO: maybe move magic checking here
  if (packet_size <= 48) { //3840 is an erronious recieve
    for (uint8_t b = 0; b < 48; b++) lora_incoming_queue[lora_incoming_queue_idx][b] = 0; //firstly clear

    for (uint8_t b = 0; (b < min(packet_size, 48)) and LoRa.available(); b++) {
      lora_incoming_queue[lora_incoming_queue_idx][b] = LoRa.read();
    }

    last_lora_rssi = LoRa.packetRssi();
    last_lora_snr = LoRa.packetSnr();
    last_lora_freq_error = LoRa.packetFrequencyError();
    lora_incoming_queue_len[lora_incoming_queue_idx] = min(packet_size, 48);
    lora_incoming_queue_idx++;
    if (lora_incoming_queue_idx >= 4) lora_incoming_queue_idx = 0;
  }
  LoRa.flush(); //clear packet if for some reason the is anything left
}

void handle_lora_tx_done() {
  LoRa.receive();
  lora_tx_ready = true;
}

//display
uint8_t alive_line_pos = 125;
bool alive_line_dir = false; //false towards left, true towards right
void draw_display_boilerplate() {
  oled.clear();
  oled.setColor(WHITE);
  oled.setFont(Lato_Thin_8);
  oled.drawRect(0, 11, 127, 42); //content outline

  //page number
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.drawString(0, 0, gw_page_to_text[disp_page]);

  //name
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  oled.drawString(63, 0, host_name);

  //time on the right
  oled.setTextAlignment(TEXT_ALIGN_RIGHT);
  char time_buf[5];
  sprintf(time_buf, "%02d:%02d", ntp.getHours(), ntp.getMinutes());
  oled.drawString(127, 0, (String)time_buf);

  //rx/tx indicators
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  uint8_t bottom_pos = 0;
  oled.drawString(bottom_pos , 55, "RX");
  bottom_pos += oled.getStringWidth("RX") + 2;
  if (rx_indicator_blink > 0) {
    oled.fillRect(bottom_pos, 55, 8, 8);
    rx_indicator_blink--;
  }
  else  oled.drawRect(bottom_pos, 55, 8, 8);
  bottom_pos += 8 + 4;
  oled.drawString(bottom_pos , 55, "TX");
  bottom_pos += oled.getStringWidth("TX") + 2;
  tx_indicator_pos = bottom_pos;
  //if (tx_indicator_blink > 0) {
  if (!lora_tx_ready) {
    oled.fillRect(bottom_pos, 55, 8, 8);
    //tx_indicator_blink--;
  }
  else  oled.drawRect(bottom_pos, 55, 8, 8);
  bottom_pos += 8 + 4;
  oled.drawString(bottom_pos , 55, "WEB");
  bottom_pos += oled.getStringWidth("WEB") + 2;
  if (web_indicator_blink > 0) {
    oled.fillRect(bottom_pos, 55, 8, 8);
    web_indicator_blink--;
  }
  else  oled.drawRect(bottom_pos, 55, 8, 8);
  bottom_pos += 8 + 4;

  //moving line to show its alive
  alive_line_pos += alive_line_dir ? 1 : -1;
  if (alive_line_pos >= 125) alive_line_dir = false;
  if (alive_line_pos <= bottom_pos) alive_line_dir = true;
  oled.drawLine(alive_line_pos , 55, alive_line_pos, 63);
}

void config_ap_callback(WiFiManager *myWiFiManager) {
  //put conf AP credentials on screen
  oled.clear();
  oled.setColor(WHITE);
  oled.setFont(Lato_Thin_8);
  oled.setTextAlignment(TEXT_ALIGN_LEFT);

  oled.drawString(0, 0, "Connect AP and visit");
  oled.drawString(0, 10, "http://192.168.4.1/");
  oled.drawString(0, 20, "SSID:");
  oled.drawString(0, 30, settings.conf_ssid);
  oled.drawString(0, 40, "Password:");
  oled.drawString(0, 50, settings.conf_pass);
  oled.display();
}


bool check_auth() {
  Serial.println(F("Authed Access Attempt"));
  bool authed = false;

  if (server.hasHeader("Cookie")) {
    String cookie_header = server.header("Cookie");
    uint16_t lc_start = cookie_header.indexOf("login_cookie=");
    uint16_t lc_end = cookie_header.substring(lc_start).indexOf("; ");
    if (lc_end == 0) lc_end = cookie_header.length() - lc_start;
    const char* login_cookie = cookie_header.substring(lc_start + 13, lc_end).c_str();

    //Serial.print(F(" * Header: "));
    //Serial.println(cookie_header);
    //Serial.print(F(" * Cookie: "));
    //Serial.println(login_cookie);
    //Serial.print(F(" * Cookie Length: "));
    //Serial.println(login_cookie.length());

    if (strlen(login_cookie) == 31) {
      for (uint16_t c = 0; c < 256; c++) { //check if login cookie valid
        //Serial.print(F(" * Cookie Candidate: "));
        //Serial.println(correct_l_cookie);

        if (strcmp(login_cookie, (const char*)web_login_cookies[c]) == 0) {
          authed = true;
          break;
        }
      }
    }
  }
  else Serial.println(F(" * Cookie Header Missing"));

  Serial.print(F(" * "));
  Serial.println(authed ? F("SUCCESS") : F("FAIL"));
  Serial.println();
  return authed;
}

void rest_status() {
  web_indicator_blink += 1;
  DynamicJsonDocument stuff(1024);
  stuff["status"]["state"] = last_wt_status >> 4;
  stuff["status"]["extra"] = last_wt_status & 0b00001111;
  stuff["status"]["as_text"] = status_to_text[last_wt_status];
  stuff["status"]["timestamp"] = last_wt_status_timestamp;
  stuff["irrigation"]["left"] = last_liters_left;
  stuff["irrigation"]["called"] = last_liters_called;
  stuff["irrigation"]["timestamp"] = last_wt_liters_timestamp;
  stuff["battery"]["voltage"] = last_wt_battery_voltage;
  stuff["battery"]["timestamp"] = last_wt_battery_voltage_timestamp;
  stuff["other"]["last_reboot"] = last_wt_reboot_timestamp;
  stuff["lora_rx"]["last_rssi"] = last_lora_rssi;
  stuff["lora_rx"]["last_snr"] = last_lora_snr;
  stuff["lora_rx"]["last_freq_error"] = last_lora_freq_error;
  stuff["lora_rx"]["last_packet_time"] = last_recieved_packet_time;
  stuff["gateway"]["wifi"]["ssid"] = WiFi.SSID();
  stuff["gateway"]["wifi"]["rssi"] = WiFi.RSSI();
  char json_stuff[2048];
  serializeJsonPretty(stuff, json_stuff);
  server.sendHeader("Refresh", "10");
  server.send(200, "application/json", json_stuff);
}

void rest_login_get() {
  DynamicJsonDocument resp(128);
  server.send(200, "application/json", check_auth() ? "true" : "false");
}

void rest_login() {
  web_indicator_blink += 1;
  if (check_auth()) {
    server.send(200, "application/json", "{\"success\":\"Already Authenticated.\"}");
    Serial.println(F("Login Attempt: Already Authed"));
    return;
  }

  Serial.println(F("Login Attempt"));

  DynamicJsonDocument login(128);
  if (server.hasArg("plain")) deserializeJson(login, server.arg("plain"));
  String user = server.hasArg("plain") ? login["user"] : server.arg("user").c_str();
  String pass = server.hasArg("plain") ? login["pass"] : server.arg("pass").c_str();
  String correct_user = settings.web_user;
  String correct_pass = settings.web_pass;

  Serial.print(F(" * User: "));
  Serial.println(user);
  Serial.print(F(" * Pass: "));
  Serial.println(pass);
  Serial.print(F(" * Correct User: "));
  Serial.println(correct_user);
  Serial.print(F(" * Correct Pass: "));
  Serial.println(correct_pass);

  if (user.equals(correct_user) and pass.equals(correct_pass)) {
    for (uint8_t b = 0; b < 31; b++) web_login_cookies[web_login_cookies_idx][b] = randomASCII();
    web_login_cookies[web_login_cookies_idx][31] = 0;
    String cookiestring = "login_cookie=";
    cookiestring += web_login_cookies[web_login_cookies_idx];
    cookiestring += "; Path=/admin/; SameSite=Strict; Max-Age=86400"; //cookie kept for a day

    server.sendHeader("Set-Cookie", cookiestring);
    if (!server.hasArg("plain")) server.sendHeader("Refresh", "3;url=/admin/control.html");
    server.send(200, "application/json", "{\"success\":\"Authenticated\",\"login_cookie\":\"" + cookiestring + "\"}");
    Serial.println(F(" * SUCCESS"));
  }
  else {
    if (!server.hasArg("plain")) server.sendHeader("Refresh", "5;url=/admin/login.html");
    server.send(403, "application/json", "{\"error\":\"Credentials invalid.\"}");
    Serial.println(F(" * FAIL"));
  }

  Serial.println();
}

void rest_control() {
  web_indicator_blink += 1;
  if (!check_auth()) {
    server.send(403, "application/json", "{\"error\":403}");
    return;
  }
  uint16_t status_code = 200;
  DynamicJsonDocument resp(512);

  DynamicJsonDocument req(256);
  if (server.hasArg("plain")) deserializeJson(req, server.arg("plain"));

  if (req["call_for_water"].is<uint16_t>() or server.hasArg("call_for_water")) {
    for (uint8_t b = 0; b < 16; b++) lora_auth_cmd_queue[lora_auth_cmd_queue_idx][b] = 0;
    union {
      uint16_t water_call = 0;
      byte water_call_b[2];
    };
    water_call = server.hasArg("plain") ? req["call_for_water"] : server.arg("call_for_water").toInt();

    lora_auth_cmd_queue[lora_auth_cmd_queue_idx][0] = PACKET_TYPE_ADD_WATER;
    lora_auth_cmd_queue[lora_auth_cmd_queue_idx][1] = water_call_b[0];
    lora_auth_cmd_queue[lora_auth_cmd_queue_idx][2] = water_call_b[1];

    lora_auth_cmd_queue_idx++;
    if (lora_auth_cmd_queue_idx >= 16) lora_auth_cmd_queue_idx = 0;

    resp["success"] = "queued water call command";
  }

  if (req["cancel_water"] == true or server.hasArg("cancel_water")) {
    for (uint8_t b = 0; b < 16; b++) lora_auth_cmd_queue[lora_auth_cmd_queue_idx][b] = 0;

    lora_auth_cmd_queue[lora_auth_cmd_queue_idx][0] = PACKET_TYPE_CANCEL_WATER;

    lora_auth_cmd_queue_idx++;
    if (lora_auth_cmd_queue_idx >= 16) lora_auth_cmd_queue_idx = 0;

    resp["success"] = "queued cancel command";
  }

  if (!server.hasArg("plain")) server.sendHeader("Refresh", "3;url=/admin/control.html");
  char buf[512];
  serializeJson(resp, buf);
  server.send(status_code, "application/json", buf);
}

void rest_control_status() {
  web_indicator_blink += 1;
  uint16_t status_code = 200;
  DynamicJsonDocument resp(128);

  uint8_t left_q = 0;
  for (uint8_t p = 0; p < 4; p++) {
    bool is_empty = true;
    for (uint8_t b = 0; b < 16; b++) if (lora_auth_cmd_queue[p][b] != 0) is_empty = false;
    if (!is_empty) left_q++;
  }

  resp["auth_state"] = auth_state;
  resp["left_in_queue"] = left_q;
  resp["last_auth_cmd_response"] = last_auth_cmd_response;
  resp["last_auth_packet_millis"] = last_auth_packet_millis;

  char buf[256];
  serializeJson(resp, buf);
  server.send(status_code, "application/json", buf);
}

void rest_admin_get() {
  web_indicator_blink += 1;
  if (!check_auth()) {
    server.send(403, "application/json", "{\"error\":403}");
    return;
  }
  uint16_t status_code = 200;
  DynamicJsonDocument resp(1024);
  //settings
  resp["wifi"]["conf_ssid"] = settings.conf_ssid;
  resp["wifi"]["conf_pass"] = settings.conf_pass;
  for (uint8_t b = 0; b < 16; b++) resp["lora"]["security_key"][b] = settings.lora_security_key[b];
  resp["mail"]["alert_email"] = settings.alert_email;
  resp["mail"]["smtp_server"] = settings.smtp_server;
  resp["mail"]["smtp_user"] = settings.smtp_user;
  //resp["mail"]["smtp_pass"] = settings.smtp_pass;
  resp["mail"]["smtp_port"] = settings.smtp_port;
  resp["mail"]["web_user"] = settings.web_user;
  //resp["mail"]["web_pass"] = settings.web_pass;

  char buf[2048];
  serializeJson(resp, buf);
  server.send(status_code, "application/json", buf);
}

byte hexs_to_byte(const String& s) { //hex string to byte. take 1 byte of hex in string form, returns byte
  char hex_val[3]; //3rd is \0
  s.toCharArray(hex_val, 3);
  byte b_val = 0;
  for (uint8_t p = 0; p < 2 /*why does it run 3 times when there is a 2 and one time when there is a 1?!*/; p++) { //this could also be used for longer nums, maybe i will do that
    if (hex_val[p] >= '0' and hex_val[p] <= '9') b_val += (hex_val[p] - '0') * (16 ^ (1 - p));
    else if (hex_val[p] >= 'A' and hex_val[p] <= 'F') b_val += (hex_val[p] - 'A') * (16 ^ (1 - p));
    else if (hex_val[p] >= 'a' and hex_val[p] <= 'f') b_val += (hex_val[p] - 'a') * (16 ^ (1 - p));
    else {
      Serial.println(F("Not a HEX value"));
      return 0x00;
    }
  }
  return b_val;
}

void rest_admin_set() {
  web_indicator_blink += 1;
  if (!check_auth()) {
    server.sendHeader("Refresh", "3;url=/admin/conf.html");
    server.send(403, "application/json", "{\"error\":403}");
    return;
  }

  if (server.hasArg("submit_lora")) {
    for (uint8_t b = 0; b < 16; b++) {
      settings.lora_security_key[b] = hexs_to_byte(server.arg("lora_key_" + String(b + 1)));
    }
  }

  if (server.hasArg("mail_address")) {
    server.arg("mail_address").toCharArray(settings.alert_email, 32);
  }
  if (server.hasArg("smtp_server")) {
    server.arg("smtp_server").toCharArray(settings.smtp_server, 32);
  }
  if (server.hasArg("smtp_user")) {
    server.arg("smtp_user").toCharArray(settings.smtp_user, 32);
  }
  if (server.hasArg("smtp_pass")) {
    server.arg("smtp_pass").toCharArray(settings.smtp_pass, 32);
  }
  if (server.hasArg("smtp_port")) {
    settings.smtp_port = server.arg("smtp_port").toInt();
  }
  if (server.hasArg("web_user")) {
    server.arg("web_user").toCharArray(settings.web_user, 32);
  }
  if (server.hasArg("conf_ssid")) {
    server.arg("conf_ssid").toCharArray(settings.conf_ssid, 16);
  }
  if (server.hasArg("conf_pass")) {
    server.arg("conf_pass").toCharArray(settings.conf_pass, 16);
  }

  EEPROM.put(0, settings);
  server.sendHeader("Refresh", "3;url=/admin/conf.html");
  server.send(200, "application/json", "{\"success\":\"ok\"");
}

void rest_debug() {
  Serial.println(F("DEBUG REST CALLED")); delay(10);
  //web_indicator_blink += 1;
  if (!check_auth()) {
    server.send(403, "application/json", "{\"error\":403}");
    return;
  }

  DynamicJsonDocument stuff(4096);

  //ram
  stuff["memory"]["heap_free"] = ESP.getFreeHeap();

  //settings
  stuff["settings"]["conf_ssid"] = settings.conf_ssid;
  stuff["settings"]["conf_pass"] = settings.conf_pass;
  stuff["settings"]["lora_security_key"] = settings.lora_security_key;
  stuff["settings"]["alert_email"] = settings.alert_email;
  stuff["settings"]["smtp_server"] = settings.smtp_server;
  stuff["settings"]["smtp_user"] = settings.smtp_user;
  //stuff["settings"]["smtp_pass"] = settings.smtp_pass;
  stuff["settings"]["smtp_port"] = settings.smtp_port;
  //stuff["settings"]["web_user"] = settings.web_user;
  //stuff["settings"]["web_pass"] = settings.web_pass;
  stuff["settings"]["display_brightness"] = settings.display_brightness;

  //lora queues
  stuff["lora_tx"]["next_packet_id"] = lora_outgoing_packet_id;
  for (uint8_t i = 0; i < 4; i++) for (uint8_t b = 0; b < 48; b++) stuff["lora_tx"]["send_queue"]["entries"][i][b] = lora_outgoing_queue[i][b];
  for (uint8_t i = 0; i < 4; i++) stuff["lora_tx"]["send_queue"]["entry_attempts"][i] = lora_outgoing_queue_tx_attempts[i];

  for (uint8_t i = 0; i < 4; i++) for (uint8_t b = 0; b < 16; b++) stuff["lora_tx"]["auth_send_queue"]["entries"][i][b] = lora_auth_cmd_queue[i][b];
  stuff["lora_tx"]["auth_state"] = auth_state;

  for (uint8_t i = 0; i < 16; i++) stuff["lora_rx"]["last_packet_IDs"][i] = lora_last_incoming_message_IDs[i];
  for (uint8_t i = 0; i < 4; i++) for (uint8_t b = 0; b < 48; b++) stuff["lora_rx"]["recive_queue"]["entries"][i][b] = lora_incoming_queue[i][b];
  for (uint8_t i = 0; i < 4; i++) stuff["lora_rx"]["recive_queue"]["entry_len"][i] = lora_incoming_queue_len[i];

  char buf[4096];
  serializeJson(stuff, buf);
  server.send(200, "application/json", buf);
}


enum mail_alert_enum {
  MAIL_ALERT_WATER = 0,
  MAIL_ALERT_BATTERY = 1,
  MAIL_ALERT_RADIO_SILENCE = 2,
  MAIL_ALERT_GENERAL = 3,
  MAIL_ALERT_NONE = 255
}; //function wont take enum as arg, gonna use uint8_t

uint8_t last_mail_alert = MAIL_ALERT_NONE;
void send_email_alert(uint8_t alert_type) { //fuck this enum shit
  if (alert_type == last_mail_alert) return;
  last_mail_alert = alert_type;

  Serial.println(F("Sending mail alert:"));
  EMailSender::EMailMessage msg;
  File msg_body_file;
  msg.mime = "text/html";

  switch (alert_type) { //read in first part of mail and add values
    case MAIL_ALERT_WATER: {
        msg.subject = "[WT] ACHTUNG: Wassertank Leer!";
        msg_body_file = SPIFFS.open("/mail/de_alert_water.html", "r");
        while (msg_body_file.available()) msg.message += msg_body_file.read();
        msg_body_file.close();
      } break;
    case MAIL_ALERT_BATTERY: {
        msg.subject = "[WT] ACHTUNG: Batterie Leer!";
        msg_body_file = SPIFFS.open("/mail/de_alert_battery.html", "r");
        while (msg_body_file.available()) msg.message += msg_body_file.read();
        msg_body_file.close();
        msg.message += last_wt_battery_voltage;
        msg.message += 'V';
      } break;
    case MAIL_ALERT_GENERAL: {
        msg.subject = "[WT] ACHTUNG: Systemfehler!";
        msg_body_file = SPIFFS.open("/mail/de_alert_gen_fail.html", "r");
        while (msg_body_file.available()) msg.message += msg_body_file.read();
        msg_body_file.close();
        //                     XXXXTMU?
        if (last_wt_status & 0b00001000) msg.message += " * Tanksensoren Werte Unsinnig";
        if (last_wt_status & 0b00000100) msg.message += " * RTC fehlt.";
        if (last_wt_status & 0b00000010) msg.message += " * RTC nicht eingestellt.";
      } break;
  }

  msg.message += "\n<br><br>\nUNIX Zeist.: ";
  msg.message += ntp.getEpochTime();
  msg.message += "\n</p>\n</body>\n</html>";

  EMailSender::Response r = email.send(settings.alert_email, msg);

  Serial.print(F(" * Status: "));
  Serial.println(r.status);
  Serial.print(F(" * Code: "));
  Serial.println(r.code);
  Serial.print(F(" * Description: "));
  Serial.println(r.desc);
  Serial.println();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(0, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, LOW);
  delay(50);
  digitalWrite(OLED_RST_PIN, HIGH);
  delay(50);
  Serial.begin(115200);

  sprintf(host_name, "WaterthingGW-%04X", (uint16_t)ESP.getEfuseMac()); //last part of MAC

  //oled
  Serial.println(F("OLED Setup..."));
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(200000);
  oled.init();
  oled.displayOn();
  oled.clear();
  oled.setFont(Lato_Thin_12);
  oled.setContrast(settings.display_brightness);
  oled.flipScreenVertically();
  oled.setColor(WHITE);
  oled.fillRect(0, 0, 128, 64); //able to see dead pixels
  oled.display();
  delay(100);
  //====dumb cool effect, you can comment this out====
  oled.setColor(BLACK);
  oled.setTextAlignment(TEXT_ALIGN_CENTER);
  String boot_msg = "Waterthing Gateway";
  uint8_t boot_msg_pos = 64;
  for (uint16_t i = 0; i < 2048; i++) { //dumb cool effect, you can comment this out
    if (!digitalRead(0)) { //skippable with PRG button
      while (!digitalRead(0));;
      delay(50);//bounce
      break; //exit
    }

    //lines
    oled.setColor(BLACK);
    oled.drawLine(random(-200, 200), random(-200, 200), random(-200, 200), random(-200, 200));

    //text
    if (i > 1024) { //after 1024 draw slowed
      if (i % 8 and boot_msg_pos > 24) {
        oled.setColor(BLACK);
        oled.drawString(63, boot_msg_pos, boot_msg);
        boot_msg_pos--;
      }
      oled.setColor(WHITE);
      oled.drawString(63, boot_msg_pos, boot_msg);
    }
    oled.display();
  }
  //====effect end====
  oled.setTextAlignment(TEXT_ALIGN_LEFT);
  oled.setColor(WHITE);

  //eeprom setup
  Serial.println(F("Loading Config..."));
  oled.clear();
  oled.drawString(0, 0, "Config...");
  oled.drawString(0, 40, "Hold PRG button NOW");
  oled.drawString(0, 50, "to reset to factory.");
  oled.display();
  delay(1000);
  oled.setColor(BLACK);
  oled.fillRect(0, 32, 128, 32);
  oled.setColor(WHITE);
  oled.display();

  if (EEPROM.begin(EEPROM_SIZE) and digitalRead(0)) { //hold GPIO0 low to reset conf at boot
    EEPROM.get(0, settings);
    oled.drawString(0, 12, F("OK"));
    oled.display();
  }
  else {
    EEPROM.put(0, settings);
    EEPROM.commit();
    oled.drawString(0, 12, F("Initialized"));
    oled.display();
    delay(900);
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
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8); //sf,bw,cr make a data rate of 366 bits per sec or 45,75 bytes per sec
    LoRa.enableCrc();
    //LoRa.onTxDone(handle_lora_tx_done); //uncomment when async fixed
    //LoRa.onReceive(handle_lora_packet);
    LoRa.receive();

    randomSeed(LoRa.random());

    //boot packet
    lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
    lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
    lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_GW_REBOOT;

    lora_outgoing_queue_last_tx = millis() - LORA_RETRANSMIT_TIME + 10000;
    lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] =  0;
    lora_outgoing_packet_id++;
    if (lora_outgoing_packet_id < 1) lora_outgoing_packet_id == 1; //never let it go to 0, that causes bugs
    lora_outgoing_queue_idx++;
    if (lora_outgoing_queue_idx >= 4) lora_outgoing_queue_idx = 0;


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
  Serial.print(F("Connecting to "));
  Serial.print(wm.getWiFiSSID());
  Serial.println(F("..."));
  oled.clear();
  oled.drawString(0, 0, "WiFi...");
  String wifi_ssid_s = wm.getWiFiSSID(true);
  oled.drawString(0, 16, wifi_ssid_s);
  oled.drawString(0, 40, "Hold PRG button NOW");
  oled.drawString(0, 50, "to start config AP.");
  oled.display();
  delay(1000);
  oled.setColor(BLACK);
  oled.fillRect(0, 32, 128, 32);
  oled.setColor(WHITE);
  oled.display();

  WiFi.mode(WIFI_STA);
  WiFi.hostname(host_name);
  wm.setAPCallback(config_ap_callback);
  if (!digitalRead(0)) {
    wm.startConfigPortal(settings.conf_ssid, settings.conf_pass);
    ESP.restart();
  }
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

  //email
  Serial.println(F("E-Mail Setup..."));
  email.setSMTPServer(settings.smtp_server);
  email.setNameFrom(host_name);
  email.setEMailFrom(settings.smtp_user);
  email.setEMailLogin(settings.smtp_user);
  email.setEMailPassword(settings.smtp_pass);
  email.setSMTPPort(settings.smtp_port);
  email.setIsSecure(true);

  //MEM CORRUPT BETWEEN HERE

  //webserver
  Serial.println(F("WebServer Setup..."));
  oled.clear();
  oled.drawString(0, 0, "WebServer...");
  oled.display();
  SPIFFS.begin();
  const char * headerkeys[] = {"User-Agent", "Cookie"} ;
  size_t headerkeyssize = sizeof(headerkeys) / sizeof(char*);
  server.collectHeaders(headerkeys, headerkeyssize);
  server.on("/rest", HTTP_GET, rest_status);
  server.on("/admin/login", HTTP_GET, rest_login_get);
  server.on("/admin/login", HTTP_POST, rest_login);
  server.on("/admin/control", HTTP_GET, rest_control_status);
  server.on("/admin/control", HTTP_POST, rest_control);
  server.on("/admin/settings", HTTP_POST, rest_admin_set);
  server.on("/admin/settings", HTTP_GET, rest_admin_get);
  server.on("/admin/debug", HTTP_GET, rest_debug);
  server.on("/", []() { //redirect to index
    server.sendHeader("Location", "/index.html");
    server.send(300, "text/html", "<a href=\"/index.html\">click here</a>");
  });
  server.serveStatic("/", SPIFFS, "/www/");
  for (uint16_t c; c < 256; c++) for (uint8_t b = 0; b < 31; b++) web_login_cookies[c][b] = randomASCII();
  server.begin();
  oled.drawString(0, 12, F("OK"));
  oled.display();
  delay(100);

  oled.clear();
  oled.display();

  //AND HERE

  //PRG button interrupt
  attachInterrupt(digitalPinToInterrupt(0), disp_button_down, FALLING);

  Serial.println(F("Booted\n"));
  digitalWrite(LED_BUILTIN, HIGH);
}

void update_display() {
  static uint32_t last_disp_update = 0;
  if (millis() - last_disp_update > 1000 / OLED_FPS) {
    draw_display_boilerplate();
    oled.setFont(Lato_Thin_12);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);

    switch (disp_page) {
      case PAGE_STATUS: {
          String status_time = "Last Status ( ";
          uint32_t seconds_since = ntp.getEpochTime() - last_wt_status_timestamp;
          status_time += (seconds_since / 60) > 999 ? ">999" : (String)(int)round((seconds_since / 60));
          status_time += "m ago):";
          oled.drawString(63, 12, status_time);
          oled.setFont(Lato_Thin_20);
          oled.drawString(63, 24, status_to_text[last_wt_status]);
        }
        break;

      case PAGE_WT: {
          String volt_string = "Battery: ";
          volt_string += last_wt_battery_voltage;
          volt_string += 'V';
          oled.drawString(63, 12, volt_string);
          if ((last_wt_status && 0b00010000) or (last_wt_status && 0b00100000)) { //only show while watering
            String water_string = "";
            water_string += last_liters_left;
            water_string += " L left";
            oled.drawString(63, 25, water_string);
            String water2_string = "of ";
            water2_string += last_liters_called;
            water2_string += " L";
            oled.drawString(63, 38, water2_string);
          }
        }
        break;

      case PAGE_LORA: {
          String rssi_string = "RSSI: ";
          rssi_string += last_lora_rssi;
          oled.drawString(63, 12, rssi_string);
          String snr_string = "SNR: ";
          snr_string += last_lora_snr;
          oled.drawString(63, 25, snr_string);
          String fe_string = "FE: ";
          fe_string += last_lora_freq_error;
          oled.drawString(63, 38, fe_string);
        }
        break;

      case PAGE_WIFI: {
          String ip_string = "IP: ";
          ip_string += WiFi.localIP().toString();
          oled.drawString(63, 12, ip_string);
          String ssid_string = "SSID: ";
          ssid_string += WiFi.SSID();
          oled.drawString(63, 25, ssid_string);
          String rssi_string = "RSSI: ";
          rssi_string +=  WiFi.RSSI();
          oled.drawString(63, 38, rssi_string);
        }
        break;
    }

    oled.display();

    last_disp_update = millis();
  }
}

void afterpacket_stuff() {
  //lora_outgoing_queue_last_tx = 0;
  lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] = 0;
  lora_outgoing_packet_id++;
  if (lora_outgoing_packet_id < 1) lora_outgoing_packet_id == 1; //never let it go to 0, that causes bugs
  lora_outgoing_queue_idx++;
  if (lora_outgoing_queue_idx >= 4) lora_outgoing_queue_idx = 0;
}

void send_ack(byte packet_id) {
  lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
  lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
  lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_ACK;
  lora_outgoing_queue[lora_outgoing_queue_idx][3] = packet_id;

  //i should really just make them next_tx_millis and remaining_attempts instead of this fuckshit
  //lora_outgoing_queue_last_tx = millis() - LORA_RETRANSMIT_TIME + 2600; //ack only sent 1000ms after
  lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] =  LORA_RETRANSMIT_TRIES - 1; //there is no response to ACKs so this ensures ther is only one ACK sent
  lora_outgoing_packet_id++;
  if (lora_outgoing_packet_id < 1) lora_outgoing_packet_id == 1; //never let it go to 0, that causes bugs
  lora_outgoing_queue_idx++;
  if (lora_outgoing_queue_idx >= 4) lora_outgoing_queue_idx = 0;
}

void clear_packet(byte packet_id) {
  for (uint8_t p = 0; p < 4; p++) {
    if (packet_id == lora_outgoing_queue[p][1]) {
      Serial.print(F(" * Clearing Packet ID: "));
      Serial.println(lora_outgoing_queue[p][1]);
      for (uint8_t b = 0; b < 48; b++) lora_outgoing_queue[p][b] = 0; //clear packet
      //lora_outgoing_queue_last_tx = 0;
      lora_outgoing_queue_tx_attempts[p] = LORA_RETRANSMIT_TRIES;
    }
  }
}

void handle_lora() {
  uint32_t last_lora_tx = 0;

  //recieve
  auto possible_packet_size = LoRa.parsePacket(); //the onRecieve() callback seems to just cause an interrupt wich is too long for the ESP so i am doing it this way
  if (possible_packet_size > 0) {
    Serial.println(F("Possible packet incoming."));
    handle_lora_packet(possible_packet_size);
  }

  for (uint8_t p_idx = 0; p_idx < 4; p_idx++) {
    bool is_empty = true;
    for (uint8_t i = 0; i < 48; i++) if (lora_incoming_queue[p_idx][i] != 0) {
        is_empty = false;  //check for data in packet
        break;
      }
    if (/*!is_empty*/lora_incoming_queue[p_idx][0] == LORA_MAGIC) {
      Serial.println(F("Incoming LoRa Packet:"));
      Serial.print(F(" * Length: "));
      Serial.println(lora_incoming_queue_len[p_idx]);
      Serial.print(F(" * Content: "));
      for (uint8_t b = 0; b < min(lora_incoming_queue_len[p_idx], uint16_t(48)); b++) {
        if (lora_incoming_queue[p_idx][b] < 0x10) Serial.write('0');
        Serial.print(lora_incoming_queue[p_idx][b], HEX);
        Serial.write(' ');
      }
      Serial.println();

      /*
         0 -> magic
         1 -> packet id
         2 -> packet type
         3... -> data
      */

      if (lora_incoming_queue[p_idx][0] == LORA_MAGIC) { //if magic correct
        Serial.println(F(" * Magic Correct."));
        
        lora_outgoing_queue_last_tx = 0; //after getting a packet, respond immediatly
        
        bool already_recieved = false;
        for (uint8_t i = 0; i < 16; i++) if (lora_incoming_queue[p_idx][1] == lora_last_incoming_message_IDs[i]) already_recieved = true;

        bool do_ack = true;
        if (!already_recieved) {
          bool dedup_this = true;
          Serial.print(F(" * Packet type: "));
          switch (lora_incoming_queue[p_idx][2]) {
            case PACKET_TYPE_WS_ACK: {
                Serial.println(F("WS ACK"));
                clear_packet(lora_incoming_queue[p_idx][3]);
                dedup_this = false;
                do_ack = false;
              }
              break;

            case PACKET_TYPE_STATUS: {
                Serial.println(F("Status"));
                last_wt_status = lora_incoming_queue[p_idx][3];

                union {
                  uint16_t bat_v =  0;
                  byte bat_b[2];
                };
                bat_b[0] = lora_incoming_queue[p_idx][4];
                bat_b[1] = lora_incoming_queue[p_idx][5];
                last_wt_battery_voltage = (double)bat_v / 100;
                last_wt_status_timestamp = last_wt_battery_voltage_timestamp = ntp.getEpochTime();

                if (last_wt_status && 0b01110000) send_email_alert(MAIL_ALERT_GENERAL);
                else if (last_wt_status && 0b01000000) send_email_alert(MAIL_ALERT_WATER);
                else if (last_wt_status && 0b01010000) send_email_alert(MAIL_ALERT_BATTERY);

              }
              break;

            case PACKET_TYPE_WATER: {
                Serial.println(F("Water"));

                last_wt_status = lora_incoming_queue[p_idx][3];

                last_liters_left = 0;
                last_liters_left |= lora_incoming_queue[p_idx][4];
                last_liters_left <<= 8;
                last_liters_left |= lora_incoming_queue[p_idx][5];

                last_liters_called = 0;
                last_liters_called |= lora_incoming_queue[p_idx][6];
                last_liters_left <<= 8;
                last_liters_called |= lora_incoming_queue[p_idx][7];

                last_wt_status_timestamp = last_wt_liters_timestamp = ntp.getEpochTime();
              }
              break;

            case PACKET_TYPE_REBOOT: {
                Serial.println(F("Reboot"));
                last_wt_reboot_timestamp = ntp.getEpochTime();
                for (uint8_t i = 0; i < 16; i++) lora_last_incoming_message_IDs[i] = 0; //counter on other side reset, so we reset too
              }
              break;

            case PACKET_TYPE_AUTH_CHALLANGE: {
                Serial.println(F("Challange"));

                last_auth_packet_millis = millis();
                clear_packet(lora_incoming_queue[p_idx][3]);

                memcpy(last_wt_challange, &lora_incoming_queue[p_idx][4], 16); //save challange, should have starrted using memcpy a while ago tbh...

                if (auth_state == AUTH_STEP_WAIT_CHALLANGE) {
                  last_auth_challange_packet_id = lora_incoming_queue[p_idx][1];
                  do_ack = false; //only skip ack packet if successful
                }
                else Serial.println(F("Not waiting for a Challange."));
              }
              break;

            case PACKET_TYPE_CMD_OK:
            case PACKET_TYPE_NO_CHALLANGE:
            case PACKET_TYPE_CMD_AUTH_FAIL:
            case PACKET_TYPE_CMD_DISABLED:
              Serial.println(F("CMD Response"));

              last_auth_packet_millis = millis();
              clear_packet(lora_incoming_queue[p_idx][3]);

              if (auth_state == AUTH_STEP_WAIT_CMD_SUCCESS) {
                auth_state = AUTH_STEP_IDLE;
                last_auth_cmd_response = lora_incoming_queue[p_idx][2];
                lora_auth_packet_processing = 255;
              }
              break;

            case PACKET_TYPE_TEST: {
                Serial.println(F("Test"));
              }
              break;

            default:
              break;
          }

          if (dedup_this) {
            lora_last_incoming_message_IDs[lora_last_incoming_message_IDs_idx] = lora_incoming_queue[p_idx][1];
            lora_last_incoming_message_IDs_idx++;
            if (lora_last_incoming_message_IDs_idx >= 16) lora_last_incoming_message_IDs_idx = 0;
          }
          last_recieved_packet_time = ntp.getEpochTime();
        }
        else Serial.println(F("Packet already recieved."));
        if (do_ack) send_ack(lora_incoming_queue[p_idx][1]); //respond so retransmits wont occur
      }

      for (uint8_t i = 0; i < 48; i++) lora_incoming_queue[p_idx][i] = 0;
      Serial.println();
      rx_indicator_blink = 0.5 * OLED_FPS;
    }
  }

  //handle auth state thing

  static uint8_t last_auth_state = 255;
  if (last_auth_state != auth_state) {
    Serial.print(F("Auth State: "));
    Serial.println(auth_state);
    last_auth_state = auth_state;
  }


  if (millis() - last_auth_packet_millis > 30000) {//give up afer hearing nothing for 30 seconds
    for (uint8_t b = 0; b < 16; b++) lora_auth_cmd_queue[lora_auth_packet_processing][b] = 0; //clear authed packet

    lora_auth_packet_processing = 255; //no more packet in process
    for (uint8_t b = 0; b < 16; b++) last_wt_challange[b] = 0; //invalid because used

    auth_state = AUTH_STEP_IDLE;
  }

  switch (auth_state) {
    case AUTH_STEP_IDLE: {
        for (uint8_t p = 0; p < 4; p++) {
          bool is_valid = false;
          for (uint8_t b = 0; b < 16; b++) if (lora_auth_cmd_queue[p][b] != 0) is_valid = true;
          if (is_valid) {
            lora_auth_packet_processing = p;

            lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
            lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
            lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_REQUST_CHALLANGE;
            afterpacket_stuff();

            last_auth_packet_millis = millis(); //so that it does not get killed right away
            auth_state = AUTH_STEP_TX_CHALLANGE_REQUEST;
            break;
          }
        }
      }
      break;
    case AUTH_STEP_TX_CHALLANGE_REQUEST: {
        if (lora_auth_packet_processing >= 4) { //just in case an invalid request is made
          auth_state = AUTH_STEP_IDLE;
          break;
        }

        //wait for tx done (if async works).
        if (true) auth_state = AUTH_STEP_WAIT_CHALLANGE;
      }
      break;
    case AUTH_STEP_WAIT_CHALLANGE: {
        if (lora_auth_packet_processing >= 4) {//just in case
          auth_state = AUTH_STEP_IDLE;
          break;
        }

        bool is_empty = true;
        for (uint8_t b = 0; b < 16; b++) if (last_wt_challange[b] != 0) is_empty = false;
        if (!is_empty) {
          //generate response and put in queue

          byte resulting_hash[32];

          mbedtls_md_context_t hash_ctx;
          mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
          mbedtls_md_init(&hash_ctx);
          mbedtls_md_setup(&hash_ctx, mbedtls_md_info_from_type(md_type), 0);
          mbedtls_md_starts(&hash_ctx);
          mbedtls_md_update(&hash_ctx, (const unsigned char *) last_wt_challange, 16);
          mbedtls_md_update(&hash_ctx, (const unsigned char *) settings.lora_security_key, 16);
          mbedtls_md_finish(&hash_ctx, resulting_hash);
          mbedtls_md_free(&hash_ctx);

          lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
          lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
          lora_outgoing_queue[lora_outgoing_queue_idx][2] = lora_auth_cmd_queue[lora_auth_packet_processing][0];
          lora_outgoing_queue[lora_outgoing_queue_idx][3] = last_auth_challange_packet_id; //anclude ACKing

          memcpy(&lora_outgoing_queue[lora_outgoing_queue_idx][4], &lora_auth_cmd_queue[lora_auth_packet_processing][1], 16); //append authed data
          uint8_t hash_begin_pos = gw_to_ws_packet_type_to_length(lora_auth_cmd_queue[lora_auth_packet_processing][0]) - 32/*minus hash*/ + 3/*overhead bytes*/;
          if (hash_begin_pos + 32 >= 48) Serial.println(F("Packet too big to fit with hash!")); //anti mem corrupt
          else {
            memcpy(&lora_outgoing_queue[lora_outgoing_queue_idx][hash_begin_pos], &resulting_hash, sizeof(resulting_hash));
            Serial.print(F("Using hash: "));
            for (uint8_t b = 0; b < sizeof(resulting_hash); b++) {
              if (resulting_hash[b] < 0x10) Serial.print(0);
              Serial.print(resulting_hash[b], HEX);
              Serial.print(' ');
            }
            Serial.println();
          }

          afterpacket_stuff();

          for (uint8_t b = 0; b < 16; b++) lora_auth_cmd_queue[lora_auth_packet_processing][b] = 0; //clear authed packet

          lora_auth_packet_processing = 255; //no more packet in process
          for (uint8_t b = 0; b < 16; b++) last_wt_challange[b] = 0; //invalid because used
          auth_state = AUTH_STEP_TX_ANSWER;
        };
      }
      break;
    case AUTH_STEP_TX_ANSWER: {
        //wait for tx done (if async works).
        if (true) auth_state = AUTH_STEP_WAIT_CMD_SUCCESS;
      }
      break;
    case AUTH_STEP_WAIT_CMD_SUCCESS: {
        //state is changed to next by receive/decode code
      }
      break;
  }
  //Serial.print(F("Auth State: "));
  //Serial.println(auth_state);

  //queue handle
  if (lora_tx_ready and millis() - lora_outgoing_queue_last_tx > LORA_RETRANSMIT_TIME) {
    for (uint8_t p_idx = 0; p_idx < 4; p_idx++) {
      bool is_empty = true;
      for (uint8_t i = 0; i < 48; i++) if (lora_outgoing_queue[p_idx][i] != 0) {
          is_empty = false;  //check for data in packet
          break;
        }
      if (!is_empty and millis() - lora_outgoing_queue_last_tx > LORA_RETRANSMIT_TIME and lora_tx_ready) {
        Serial.println(F("Sending LoRa Packet: "));

        //kind of hacky way to display the tx indicator on the display. comment out if it causes visual problems
        oled.setColor(WHITE);
        oled.fillRect(tx_indicator_pos, 55, 8, 8);
        oled.display();

        lora_tx_ready = false;
        LoRa.idle(); //no recieving while transmitting!
        LoRa.beginPacket();

        uint8_t lora_bytes = gw_to_ws_packet_type_to_length(lora_outgoing_queue[p_idx][2]) + 3 ; // check 2nd byte (packet type), get data length and add 3 for magic + packet id+ packett type
        Serial.print(F(" * Length: "));
        Serial.println(lora_bytes);

        Serial.print(F(" * Content: "));
        for (uint8_t b = 0; b < lora_bytes; b++) {
          LoRa.write(lora_outgoing_queue[p_idx][b]);
          if (lora_outgoing_queue[p_idx][b] < 0x10) Serial.write('0');
          Serial.print(lora_outgoing_queue[p_idx][b], HEX);
          Serial.write(' ');
        }
        LoRa.endPacket(/*true*/false); //tx in not async mode becaus that never seems to work
        //only in not async
        handle_lora_tx_done();
        //
        Serial.println();

        lora_outgoing_queue_last_tx = millis();
        lora_outgoing_queue_tx_attempts[p_idx]++;
        if (lora_outgoing_queue_tx_attempts[p_idx] >= LORA_RETRANSMIT_TRIES) {
          for (uint8_t i = 0; i < 48; i++) lora_outgoing_queue[p_idx][i] = 0; //clear packet if unsuccessful
          //lora_outgoing_queue_last_tx = 0;
          lora_outgoing_queue_tx_attempts[p_idx] = 0;
        }

        oled.setColor(BLACK);
        oled.fillRect(tx_indicator_pos, 55, 8, 8);
        oled.setColor(WHITE);
        //tx_indicator_blink = 0.5 * OLED_FPS;
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
