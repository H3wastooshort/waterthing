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
//#include <Crypto.h> //https://github.com/intrbiz/arduino-crypto

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
  char conf_ssid[16] = "WaterthingGW\0\0\0";
  char conf_pass[16] = "524901457882057";
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
uint32_t lora_outgoing_queue_last_tx[4] = {0};
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
uint8_t lora_auth_packet_processing = 255;
byte last_auth_cmd_response = 0xFF;
byte last_auth_challange_packet_id = 0xFF;

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


char randomASCII() { //give random lowercase ascii
  return std::min(std::max(double(std::round(LoRa.random() / 10)) + double('a'), double('z')), double('a'));
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
void draw_display_boilerplate() {
  oled.clear();
  oled.setColor(WHITE);
  oled.setFont(Lato_Thin_8);
  oled.drawRect(0, 11, 127, 42);

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


  //todo: add indicators on bottom

  //oled.display();
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
  String cookie_header = server.header("Cookie");
  uint16_t lc_start = cookie_header.indexOf("login_cookie=");
  uint16_t lc_end = cookie_header.substring(lc_start).indexOf("; ");

  String login_cookie = cookie_header.substring(lc_start, lc_end);

  Serial.println(F("Authed Access Attempt"));
  Serial.print(F(" * Cookie: "));
  Serial.println(login_cookie);
  Serial.print(F(" * Cookie Length: "));
  Serial.println(login_cookie.length());

  bool authed = false;
  if (login_cookie.length() == 32) {
    for (uint16_t c; c <= 255; c++) { //check if login cookie valid
      String correct_l_cookie = web_login_cookies[c];
      if (login_cookie.equals(correct_l_cookie)) {
        authed = true;
        break;
      }
    }
  }

  Serial.print(F(" * "));
  Serial.println(authed ? F("SUCCESS") : F("FAIL"));
  Serial.println();
  return authed;
}

void rest_status() {
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
  server.send(200, "application/json", json_stuff);
}


void rest_login() {
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
    for (uint8_t b = 0; b < 32; b++) web_login_cookies[web_login_cookies_idx][b] = randomASCII();
    String cookiestring = "login_cookie=";
    cookiestring += web_login_cookies[web_login_cookies_idx];
    cookiestring += "; Path=/admin/; SameSite=Strict; Max-Age=86400"; //cookie kept for a day

    server.sendHeader("Set-Cookie", cookiestring);
    server.send(200, "application/json", "{\"success\":\"Authenticated\",\"login_cookie\":\"" + cookiestring + "\"}");
    Serial.println(F(" * SUCCESS"));
  }
  else {
    server.send(403, "application/json", "{\"error\":\"Credentials invalid.\"}");
    Serial.println(F(" * FAIL"));
  }

  Serial.println();
}

void rest_control() {
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
    water_call = server.hasArg("plain") ? req["call_for_water"] : String(server.arg("plain")).toInt();

    lora_auth_cmd_queue[lora_auth_cmd_queue_idx][0] = PACKET_TYPE_ADD_WATER;
    lora_auth_cmd_queue[lora_auth_cmd_queue_idx][1] = water_call_b[0];
    lora_auth_cmd_queue[lora_auth_cmd_queue_idx][2] = water_call_b[1];

    lora_auth_cmd_queue_idx++;
    if (lora_auth_cmd_queue_idx >= 16) lora_auth_cmd_queue_idx = 0;
  }

  if (req["cancel_water"].is<bool>() or server.hasArg("cancel_water")) {
    for (uint8_t b = 0; b < 16; b++) lora_auth_cmd_queue[lora_auth_cmd_queue_idx][b] = 0;

    lora_auth_cmd_queue[lora_auth_cmd_queue_idx][0] = PACKET_TYPE_CANCEL_WATER;

    lora_auth_cmd_queue_idx++;
    if (lora_auth_cmd_queue_idx >= 16) lora_auth_cmd_queue_idx = 0;
  }

  char buf[512];
  serializeJson(resp, buf);
  server.send(status_code, "application/json", buf);
}

void rest_control_status() {
  uint16_t status_code = 200;
  DynamicJsonDocument resp(128);

  uint8_t left_q = 0;
  for (uint8_t p = 0; p < 4; p + 16) {
    bool is_empty = true;
    for (uint8_t b = 0; b < 16; b++) if (lora_auth_cmd_queue[p][b] != 0) is_empty = false;
    if (!is_empty) left_q++;
  }

  resp["auth_state"] = auth_state;
  resp["left_in_queue"] = left_q;

  char buf[256];
  serializeJson(resp, buf);
  server.send(status_code, "application/json", buf);
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
  DynamicJsonDocument stuff(4096);

  //settings
  stuff["settings"]["conf_ssid"] = settings.conf_ssid;
  stuff["settings"]["conf_pass"] = settings.conf_pass;
  stuff["settings"]["lora_security_key"] = settings.lora_security_key;
  stuff["settings"]["alert_email"] = settings.alert_email;
  stuff["settings"]["smtp_server"] = settings.smtp_server;
  stuff["settings"]["smtp_user"] = settings.smtp_user;
  //stuff["settings"]["smtp_pass"] = settings.smtp_pass;
  stuff["settings"]["smtp_port"] = settings.smtp_port;
  stuff["settings"]["web_user"] = settings.web_user;
  stuff["settings"]["web_pass"] = settings.web_pass;
  stuff["settings"]["display_brightness"] = settings.display_brightness;

  //lora queues
  stuff["lora_tx"]["next_packet_id"] = lora_outgoing_packet_id;
  for (uint8_t i = 0; i < 4; i++) for (uint8_t b = 0; b < 48; b++) stuff["lora_tx"]["send_queue"]["entries"][i][b] = lora_outgoing_queue[i][b];
  for (uint8_t i = 0; i < 4; i++) stuff["lora_tx"]["send_queue"]["entry_attempts"][i] = lora_outgoing_queue_tx_attempts[i];

  for (uint8_t i = 0; i < 16; i++) stuff["lora_rx"]["last_packet_IDs"][i] = lora_last_incoming_message_IDs[i];
  for (uint8_t i = 0; i < 4; i++) for (uint8_t b = 0; b < 48; b++) stuff["lora_rx"]["recive_queue"]["entries"][i][b] = lora_incoming_queue[i][b];
  for (uint8_t i = 0; i < 4; i++) stuff["lora_rx"]["recive_queue"]["entry_len"][i] = lora_incoming_queue_len[i];

  char buf[8192];
  serializeJson(stuff, buf);
  server.send(status_code, "application/json", buf);
}

/*
  enum mail_alert_enum {
  MAIL_ALERT_WATER = 0,
  MAIL_ALERT_BAT,
  MAIL_ALERT_RADIO_SILENCE,
  MAIL_ALERT_GENERAL
  };

  void send_email_alert(mail_alert_enum alert_type) { //TODO: fugure out why "variable or field 'send_email_alert' declared void" when i use the mail_alert_enum
  Serial.println(F("Sending mail alert:"));
  EMailSender::EMailMessage msg;
  msg.mime = "text/html";

  switch (alert_type) { //read in first part of mail and add values
    case MAIL_ALERT_WATER:
      File msg_body_file;
      msg.subject = "[WT] ACHTUNG: Wassertank Leer!";
      msg_body_file = SPIFFS.open("/mail/de_alert_water.html", "r");
      while (msg_body_file.available()) msg.message += msg_body_file.read();
      msg_body_file.close();
      break;
    case MAIL_ALERT_BAT:
      File msg_body_file;
      msg.subject = "[WT] ACHTUNG: Batterie Leer!";
      msg_body_file = SPIFFS.open("/mail/de_alert_battery.html", "r");
      while (msg_body_file.available()) msg.message += msg_body_file.read();
      msg_body_file.close();
      msg.message += last_wt_battery_voltage;
      msg.message += 'V';
      break;
    case MAIL_ALERT_WATER:
      File msg_body_file;
      msg.subject = "[WT] ACHTUNG: Systemfehler!";
      msg_body_file = SPIFFS.open("/mail/de_alert_gen_fail.html", "r");
      while (msg_body_file.available()) msg.message += msg_body_file.read();
      msg_body_file.close();
      //                     XXXXTMU?
      if (last_wt_status & 0b00001000) msg.message += " * Tanksensoren Werte Unsinnig";
      if (last_wt_status & 0b00000100) msg.message += " * RTC fehlt.";
      if (last_wt_status & 0b00000010) msg.message += " * RTC nicht eingestellt.";
      break;
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
*/

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
  oled.setColor(WHITE);
  oled.setFont(Lato_Thin_12);
  oled.setContrast(settings.display_brightness);
  oled.flipScreenVertically();

  //eeprom setup
  Serial.println(F("EEPROM Setup..."));
  oled.clear();
  oled.drawString(0, 0, "EEPROM...");
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
    LoRa.setSignalBandwidth(250E3);
    LoRa.setCodingRate4(8); //sf,bw,cr make a data rate of 366 bits per sec or 45,75 bytes per sec
    LoRa.enableCrc();
    //LoRa.onTxDone(handle_lora_tx_done); //uncomment when async fixed
    //LoRa.onReceive(handle_lora_packet);
    LoRa.receive();

    //boot packet
    lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
    lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
    lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_GW_REBOOT;

    lora_outgoing_queue_last_tx[lora_outgoing_queue_idx] = millis() - LORA_RETRANSMIT_TIME + 10000;
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
  Serial.println(F("Connecting WiFi..."));
  oled.clear();
  oled.drawString(0, 0, "WiFi...");
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

  //webserver
  Serial.println(F("WebServer Setup..."));
  oled.clear();
  oled.drawString(0, 0, "WebServer...");
  oled.display();
  SPIFFS.begin();
  server.on("/rest", HTTP_GET, rest_status);
  server.on("/admin/login_rest", HTTP_POST, rest_login);
  server.on("/admin/control_rest", HTTP_GET, rest_control_status);
  server.on("/admin/control_rest", HTTP_POST, rest_control);
  server.on("/admin/settings_rest", HTTP_POST, rest_admin_set);
  server.on("/admin/settings_rest", HTTP_GET, rest_admin_get);
  server.on("/admin/debug_rest", HTTP_GET, rest_debug);
  server.on("/", []() { //redirect to index
    server.sendHeader("Location", "/index.html");
    server.send(300, "text/html", "<a href=\"/index.html\">click here</a>");
  });
  server.serveStatic("/", SPIFFS, "/www/");
  for (uint16_t c; c <= 255; c++) for (uint8_t b = 0; b < 32; b++) web_login_cookies[c][b] = randomASCII();
  server.begin();
  oled.drawString(0, 12, F("OK"));
  oled.display();
  delay(100);

  oled.clear();
  oled.display();

  Serial.println(F("Booted\n"));
  digitalWrite(LED_BUILTIN, HIGH);
}

void update_display() {
  static uint32_t last_disp_update = 0;
  if (millis() - last_disp_update > 1000 / OLED_FPS) {
    draw_display_boilerplate();

    oled.setFont(Lato_Thin_12);
    oled.setTextAlignment(TEXT_ALIGN_CENTER);
    String status_time = "Last Status ( ";
    uint32_t seconds_since = ntp.getEpochTime() - last_wt_status_timestamp;
    status_time += (seconds_since / 60) > 999 ? ">999" : (String)(int)round((seconds_since / 60));
    status_time += "m ago):";
    oled.drawString(63, 12, status_time);
    oled.setFont(Lato_Thin_20);
    oled.drawString(63, 24, status_to_text[last_wt_status]);
    oled.display();

    last_disp_update = millis();
  }
}

void send_ack(byte packet_id) {
  lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
  lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
  lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_ACK;
  lora_outgoing_queue[lora_outgoing_queue_idx][3] = packet_id;

  lora_outgoing_queue_last_tx[lora_outgoing_queue_idx] = millis() - LORA_RETRANSMIT_TIME + 2500; //ack only sent 1000ms after
  lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] =  LORA_RETRANSMIT_TRIES - 1; //there is no response to ACKs so this ensures ther is only one ACK sent
  lora_outgoing_packet_id++;
  if (lora_outgoing_packet_id < 1) lora_outgoing_packet_id == 1; //never let it go to 0, that causes bugs
  lora_outgoing_queue_idx++;
  if (lora_outgoing_queue_idx >= 4) lora_outgoing_queue_idx = 0;
}

void clear_packet(byte packet_id) {
  for (uint8_t p = 0; p < 4; p++) {
    if (packet_id == lora_outgoing_queue[p][1]) {
      for (uint8_t b = 0; b < 48; b++) lora_outgoing_queue[p][b] = 0; //clear packet
      lora_outgoing_queue_last_tx[p] = 0;
      lora_outgoing_queue_tx_attempts[p] = LORA_RETRANSMIT_TRIES;
      Serial.print(F(" * Cleared Packet ID: "));
      Serial.println(lora_outgoing_queue[p][1]);
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
    if (/*!is_empty*/lora_incoming_queue[p_idx][0] == 42) {
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

      if (lora_incoming_queue[p_idx][0] == 42) { //if magic correct
        Serial.println(F("Magic Correct."));
        bool already_recieved = false;
        for (uint8_t i = 0; i < 16; i++) if (lora_incoming_queue[p_idx][1] == lora_last_incoming_message_IDs[i]) already_recieved = true;

        bool do_ack = true;
        if (!already_recieved) {
          bool dedup_this = true;
          Serial.print(F("Packet type: "));
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
                if (auth_state == AUTH_STEP_WAIT_CHALLANGE) {
                  last_auth_challange_packet_id = lora_incoming_queue[p_idx][1];
                  auth_state = AUTH_STEP_TX_ANSWER;
                  clear_packet(lora_incoming_queue[p_idx][3]);
                  do_ack = false;
                }
              }
              break;

            case PACKET_TYPE_CMD_OK:
            case PACKET_TYPE_CMD_AUTH_FAIL:
            case PACKET_TYPE_CMD_DISABLED:
              if (auth_state == AUTH_STEP_WAIT_CMD_SUCCESS) {
                Serial.println(F("CMD Response"));
                auth_state = AUTH_STEP_IDLE;
                last_auth_cmd_response = lora_incoming_queue[p_idx][2];
                clear_packet(lora_incoming_queue[p_idx][3]);
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
    }
  }

  //handle auth state thing
  switch (auth_state) {
    case AUTH_STEP_IDLE: {
        for (uint8_t p = 0; p < 4; p + 16) {
          bool is_empty = true;
          for (uint8_t b = 0; b < 16; b++) if (lora_auth_cmd_queue[p][b] != 0) is_empty = false;
          if (!is_empty) {
            lora_auth_packet_processing = p;
            break;
          }

          if (lora_auth_packet_processing != 255) {
            auth_state = AUTH_STEP_TX_CHALLANGE_REQUEST;
          }
        }
        break;
      }
    case AUTH_STEP_TX_CHALLANGE_REQUEST: {
        //wait for tx done (if async works).
        if (true) auth_state = AUTH_STEP_WAIT_CHALLANGE;
        break;
      }
    case AUTH_STEP_WAIT_CHALLANGE: {
        if (last_wt_challange != 0) {
          //generate response and put in queue

          byte val_to_hash[32];

          mbedtls_md_context_t hash_ctx;
          mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
          mbedtls_md_init(&hash_ctx);
          mbedtls_md_setup(&hash_ctx, mbedtls_md_info_from_type(md_type), 0);
          mbedtls_md_starts(&hash_ctx);
          mbedtls_md_update(&hash_ctx, (const unsigned char *) last_wt_challange, 16);
          mbedtls_md_update(&hash_ctx, (const unsigned char *) settings.lora_security_key, 16);
          mbedtls_md_finish(&hash_ctx, val_to_hash);
          mbedtls_md_free(&hash_ctx);

          lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
          lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
          for (uint8_t b = 0; b < 16; b++) lora_outgoing_queue[lora_outgoing_queue_idx][2 + b] = lora_auth_cmd_queue[lora_auth_packet_processing][b]; //append all of authed packet queue entry. the tx code will know what the true length is

          lora_outgoing_queue_last_tx[lora_outgoing_queue_idx] = 0;
          lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] = 0;
          lora_outgoing_packet_id++;
          if (lora_outgoing_packet_id < 1) lora_outgoing_packet_id == 1; //never let it go to 0, that causes bugs
          lora_outgoing_queue_idx++;
          if (lora_outgoing_queue_idx >= 4) lora_outgoing_queue_idx = 0;

          for (uint8_t b = 0; b < 16; b++) lora_auth_cmd_queue[lora_auth_packet_processing][b] = 0; //clear authed packet

          lora_auth_packet_processing = 255; //no more packet in process
          for (uint8_t b = 0; b < 16; b++) last_wt_challange[b] = 0; //invalid because used
          auth_state = AUTH_STEP_TX_ANSWER;
        };
        break;
      }
    case AUTH_STEP_TX_ANSWER: {
        //wait for tx done (if async works).
        if (true) auth_state = AUTH_STEP_WAIT_CMD_SUCCESS;
        break;
      }
    case AUTH_STEP_WAIT_CMD_SUCCESS: {
        //state is changed to next by receive/decode code
        break;
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
        Serial.println(F("Sending LoRa Packet: "));

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

        lora_outgoing_queue_last_tx[p_idx] = millis();
        lora_outgoing_queue_tx_attempts[p_idx]++;
        if (lora_outgoing_queue_tx_attempts[p_idx] >= LORA_RETRANSMIT_TRIES) {
          for (uint8_t i = 0; i < 48; i++) lora_outgoing_queue[p_idx][i] = 0; //clear packet if unsuccessful
          lora_outgoing_queue_last_tx[p_idx] = 0;
          lora_outgoing_queue_tx_attempts[p_idx] = 0;
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
