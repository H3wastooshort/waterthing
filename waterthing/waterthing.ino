//Bewässerungssystem

/*
  ====IMPORTANT====
  If you run into space issues (you will), install the Optiboot bootloader on your board (it saves about 1 kb space)
  and compile with the ATMega328P option in MiniCore (https://github.com/MCUdude/MiniCore)
  as that has the correct space limit and can turn on LTO (Link-time optimizations) to save even more space. good luck
*/

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include "gfx.h"
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>  //https://github.com/PaulStoffregen/DS1307RTC/
#include <math.h>
#include <pcf8574.h>  //https://github.com/MSZ98/pcf8574
#include <SPI.h>
#include <LoRa.h>  //https://github.com/sandeepmistry/arduino-LoRa
#include <avr/wdt.h>
#include "sha256_small.h"  //https://github.com/simonratner/Arduino-SHA-256

#define LOW_WATER_PIN A1
#define TANK_BOTTOM_PIN A2
#define TANK_TOP_PIN A3  //reused for water flow sensor when in direct mode. WARNING! change PCINT suff in setup() too!!
#define RAIN_DETECTOR_PIN 8
#define SENSOR_DEBOUNCE 1000
#define PUMP_PIN 6
#define VALVE_PIN 7
#define CONTROL_RELAY_INVERTED true

//pcf
#define LED_PCF_ADDR 0x3F
#define RED_LED_PIN 5
#define GREEN_LED_PIN 6
#define BLUE_LED_PIN 7
#define TB_SENSOR_LED 2  //tank bottom sensor indicator
#define TT_SENSOR_LED 1  //tank top sensor indicator
#define PUMP_LED 0
#define LORA_TX_LED 3
#define LORA_RX_LED 4

#define DEBOUNCE_DELAY 150
#define BTN_PIN 5          //PCINT
#define ENCODER_CLK_PIN 3  //WARNING! change PCINT suff in setup() too!!
#define ENCODER_DT_PIN 4
#define ENCODER_INVERT_DIRECTION false

#define LCD_BACKLIGHT_TIMEOUT 10000

#define BATTERY_VOLTAGE_PIN A0

#define EEPROM_MAGIC_NUMBER 42

#define LORA_FREQ 869525000        //869.525mhz is allowed to be used at 100mW 10% duty cycle (360 sec an hour) in germany
#define LORA_TX_PWR 20             //20dbm/100mW is max
#define LORA_RETRANSMIT_TIME 5000  //time between retransmit attempts in ms
#define LORA_RETRANSMIT_TRIES 5
#define LORA_MAGIC 42
#define LORA_TX_INTERVAL 1800000  //time between beacon broadcasts in ms (30min)
#define LORA_MAX_AIRTIME 340     //in seconds

//library stuff
LiquidCrystal_I2C lcd(0x27, 16, 2);
PCF8574 pcf(LED_PCF_ADDR);  //all addr pins to high

enum gfx_IDs {        //enum for naming display gfx ids
  GFX_ID_STATUS = 0,  //redefined dynamically
  GFX_ID_DYN_1 = 1,
  GFX_ID_DYN_2 = 2,
  GFX_ID_DYN_3 = 3,
  GFX_ID_UML_S = 4,
  GFX_ID_UML_A = 5,
  GFX_ID_UML_O = 6,
  GFX_ID_UML_U = 7,
  GFX_ID_ARROW_L = 126,
  GFX_ID_ARROW_R = 127
};

//structs are used to save to EEPROM more easily
struct settings_s {
  uint16_t tank_capacity = 10;                  //in L
  double battery_voltage_adc_divider = 68.267;  //1024 divided by this multiplier equals the battery voltage
  float battery_voltage_cutoff = 10.4;          //if this voltage is reached while the system is idle, a low battery waring is displayed. 0 means disable
  float battery_voltage_reset = 11.4;           //if this voltage is reached while the system is in low batters state, the system is set back to idle
  int16_t max_mins_per_refill = -1;             //error is thrown if the pump takes longer than that to fill the tank (5L/min pump wont take more than 5 minutes to fill a 20L tank)
  uint8_t afterdrain_time = 5;                  //keep draining for this amount of time after a cycle finishes to dry the tank out
  uint16_t clicks_per_liter = 1000;
  bool low_water_on_level = LOW;     //level of low water pin when the low water sensor detects low water. sensor will probably be mounted upside-down so no water means low
  bool tank_bottom_on_level = HIGH;  //level of TANK_BOTTOM_PIN when water has reached the bottom sensor. sensor will probably be mounted upside-down so on means high
  bool tank_top_on_level = LOW;      //level of TANK_TOP_PIN when water has reached the top sensor
  bool rain_detected_on_level = LOW;
  uint8_t rain_minutes_til_block = 5;    //if rain is detected for more than this time, it is officially raining
  uint16_t rain_minutes_til_clear = 60;  //if its not rainf or at least this time, watering will resume as normal
  bool block_water_after_rain = false;
  byte lora_security_key[16];
  uint8_t lora_enable = 0;   //0=off, 1=broadcast only, 2=control
  uint8_t pump_timeout = 0;  //in minutes. if filling the tank takes longer than this value, the system goes into a fail state
  //uint32_t crc; //maybe verify with a crc32 here
} settings;

struct i_timer_s {
  int8_t start_hour = 0;
  int8_t start_minute = 0;
  uint16_t fillings_to_irrigate = 0;  //how many tank fillings to irrigate. when 0, system is turned off.
  uint8_t last_watering_day = 0;
  uint16_t liters_to_pump = 0;  //when in direct mode (no tank) how much liters to pump out
} irrigation_timer;

//and also just to organize things
struct c_error_s {
  bool rtc_missing = false;
  bool rtc_unset = false;
  bool tank_sensors_irrational = false;
  bool lora_missing = false;
  bool pump_timed_out = false;
} component_errors;

//enums for easier code reading
enum system_state_e {
  STATUS_IDLE = 0,         //doing nothing
  STATUS_PUMPING = 1,      //pumping water in tank
  STATUS_EMPTYING = 2,     //emptying tank
  STATUS_AFTERDRAIN = 3,   //fully drain out tank
  STATUS_NO_WATER = 4,     //cant get water from tank
  STATUS_LOW_BATTERY = 5,  //low battery voltage
  STATUS_NO_TIME = 6,      //clock not set
  STATUS_GENERAL_FAIL = 7
};

//global system variables
enum system_state_e system_state = STATUS_IDLE;
tmElements_t current_time;
byte mcusr_copy;

struct sensor_s {
  bool low_water = false;
  bool tank_bottom = false;
  bool tank_top = false;
  bool rain_detected = false;
  uint32_t water_flow_clicks = 0;
  uint32_t rain_start_millis = 0xFFFFFFFF;
  uint32_t rain_end_millis = 0xFFFFFFFF;
  float battery_voltage = 0;
} sensor_values;

uint16_t tank_fillings_remaining = 0;  //if over 0, run pumping stuff til 0. is in tank fillings normally, in liters in direct mode
uint64_t cycle_finish_millis = 0xFFFFFFFF;
uint32_t pumping_start_millis = 0;  //0 if never started

//lora variables
// it would have been a better idea to have an outgoing and incoming array with structs in it instead of multiple for each attribute
uint8_t lora_outgoing_packet_id = 1;      //increments every packet
byte lora_outgoing_queue[4][48] = { 0 };  //holds up to 4 messages that are to be sent with max 48 bits. first byte is magic, 2nd message id, 3nd is message type, rest is data. if all 0, no message in slot. set to 0 after up to 0 retransmits
uint8_t lora_outgoing_queue_idx = 0;      //idx where to write
uint32_t lora_outgoing_queue_last_tx = 0;
uint8_t lora_outgoing_queue_tx_attempts[4] = { 0 };
bool lora_tx_ready = true;
uint32_t lora_tx_start_millis = 0;
uint32_t lora_airtime = 0;  //in millis

byte lora_incoming_queue[4][48] = { 0 };      //holds up to 4 messages that are to be sent with max 48 bits.
uint8_t lora_incoming_queue_idx = 0;          //idx where to write
uint16_t lora_incoming_queue_len[4] = { 0 };  //lengths of recieved packages
byte lora_last_incoming_message_IDs[16] = { 255 };
uint8_t lora_last_incoming_message_IDs_idx = 0;

uint32_t last_auth_packet_millis = 0;

//shared stuff start
enum lora_packet_types_ws_to_gw {  //water system to gateway
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

enum lora_packet_types_gw_to_ws {  //gateway to water system
  PACKET_TYPE_CURRENT_TIME = 0,
  PACKET_TYPE_ADD_WATER = 1,
  PACKET_TYPE_CANCEL_WATER = 2,
  PACKET_TYPE_SET_TIMER = 3,
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
    case PACKET_TYPE_AUTH_CHALLANGE: return 17; break;
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
    case PACKET_TYPE_SET_TIMER: return 37; break;
    case PACKET_TYPE_GW_REBOOT: return 0; break;
  }
}
//shared stuff end

//auth stuff
byte auth_challange[16] = { 0 };  //all 0 means invalid
byte last_auth_cr_packet_id = 0xFF;

//global menu variables
int8_t menu_page = 0;          // 0-> main page, 1 -> Timer Setup, 2 -> Tank Setup, 3 -> Clock Setup
int8_t menu_entry_cursor = 0;  //-2 -> not in menu system, 0 -> page selection, 1-255 differs per page
bool menu_editing = false;
uint32_t last_display_update = 0;  //this is a global var to allow calling for immediate update by setting this to 0
uint8_t button_queue[32];          //0 means no btn, 1 means down, 2 means up, 3 means enter
uint8_t button_queue_add_pos = 0;  //0-31
uint32_t last_interface_interaction = 0;
bool redraw_display_fully = false;

//menu vars for each page
//1 manuell
uint16_t page_1_irrigate_order = 0;

//pages
enum pages_enum {
  PAGE_STATUS = 0,
  PAGE_MAN = 1,
  PAGE_RAIN = 2,
  PAGE_TIMER = 3,
  PAGE_LORA = 4,
  PAGE_TANK = 5,
  PAGE_PUMP = 6,
  PAGE_CLOCK1 = 7,
  PAGE_CLOCK2 = 8,
  PAGE_BATTERY = 9,
  PAGE_SENSORS = 10
};

#define N_OF_PAGES 11
const char* page_names[N_OF_PAGES] = { "Status", "Manuell", "Regen", "Timer", "LoRa", "Tank", "Pumpe", "Uhrzeit", "Datum", "Akku", "Sensor" };
const uint8_t page_max_cursor[N_OF_PAGES] = { 0, 1, 3, 3, 3, 2, 1, 3, 3, 2, 3 };

int16_t literToTanks(uint16_t liters_to_convert) {
  if (liters_to_convert % settings.tank_capacity == 0) {
    return liters_to_convert / settings.tank_capacity;
  } else {
    return ((float)liters_to_convert / (float)settings.tank_capacity) + 1;
  }
}

void lcd_print_menu_bracket(uint8_t for_menu_entry, bool ending_bracket) {
  if (ending_bracket) lcd.print(menu_entry_cursor == for_menu_entry ? (menu_editing ? '}' : ']') : ')');
  else lcd.print(menu_entry_cursor == for_menu_entry ? (menu_editing ? '{' : '[') : '(');
}


uint32_t last_menu_entry_change = 0;
void change_menu_entry(bool dir) {  //true is up
  if (menu_entry_cursor == 0) {
    menu_page += dir ? 1 : -1;
    if (menu_page < 0) menu_page = 0;
    if (menu_page >= N_OF_PAGES) menu_page = N_OF_PAGES - 1;
    return;
  }
  switch (menu_page) {
    case PAGE_MAN:  //manual
      if (menu_entry_cursor == 1) {
        if (tank_fillings_remaining == 0) {
          if (settings.tank_capacity > 0) {
            page_1_irrigate_order += dir ? settings.tank_capacity : settings.tank_capacity * -1;
            if (page_1_irrigate_order > 0xFF00 /* overflow */) page_1_irrigate_order = 0;
            if (page_1_irrigate_order > 990) page_1_irrigate_order = 990;
          } else {
            page_1_irrigate_order += dir ? 1 : -1;
            if (page_1_irrigate_order > 60000 /* overflow */) page_1_irrigate_order = 0;
            if (page_1_irrigate_order > 50000) page_1_irrigate_order = 50000;
          }
        }
      }
      break;

    case PAGE_RAIN:
      switch (menu_entry_cursor) {
        case 2:
          settings.rain_minutes_til_block += dir ? 1 : -1;
          if (settings.rain_minutes_til_block > 0xF0) settings.rain_minutes_til_block = 0;
          if (settings.rain_minutes_til_block >= 99) settings.rain_minutes_til_block = 99;
          break;
        case 3:
          settings.rain_minutes_til_clear += dir ? 1 : -1;
          if (settings.rain_minutes_til_clear > 0xFFF0) settings.rain_minutes_til_clear = 0;
          if (settings.rain_minutes_til_clear >= 999) settings.rain_minutes_til_clear = 999;
          break;
        default:
          break;
      }
      break;

    case PAGE_TIMER:
      switch (menu_entry_cursor) {
        case 1:
          irrigation_timer.start_hour += dir ? 1 : -1;
          if (irrigation_timer.start_hour >= 23) irrigation_timer.start_hour = 23;
          if (irrigation_timer.start_hour < 0) irrigation_timer.start_hour = 0;
          break;
        case 2:
          irrigation_timer.start_minute += dir ? 1 : -1;
          if (irrigation_timer.start_minute >= 59) irrigation_timer.start_minute = 59;
          if (irrigation_timer.start_minute < 0) irrigation_timer.start_minute = 0;
          break;
        case 3:
          if (settings.tank_capacity > 0) {
            irrigation_timer.fillings_to_irrigate += dir ? 1 : -1;
            if (irrigation_timer.fillings_to_irrigate < 0 or irrigation_timer.fillings_to_irrigate > 0xFFF0) irrigation_timer.fillings_to_irrigate = 0;
            if (irrigation_timer.fillings_to_irrigate >= literToTanks(900)) irrigation_timer.fillings_to_irrigate = literToTanks(900);
          } else {
            irrigation_timer.liters_to_pump += dir ? 1 : -1;
            if (irrigation_timer.liters_to_pump < 0 or irrigation_timer.liters_to_pump > 0xFFF0) irrigation_timer.liters_to_pump = 0;
            if (irrigation_timer.liters_to_pump >= 999) irrigation_timer.liters_to_pump = 999;
          }
          break;

        default:
          break;
      }
      break;

    case PAGE_TANK:
      switch (menu_entry_cursor) {
        case 1:
          settings.tank_capacity += dir ? 1 : -1;
          if (settings.tank_capacity < 0 or settings.tank_capacity > 0xFFF0) settings.tank_capacity = 0;
          if (settings.tank_capacity >= 200) settings.tank_capacity = 200;
          break;
        case 2:
          if (settings.tank_capacity > 0) {
            settings.afterdrain_time += dir ? 1 : -1;
            if (settings.afterdrain_time > 250 /*overflow*/) settings.afterdrain_time = 0;
            if (settings.afterdrain_time >= 90) settings.afterdrain_time = 90;
          } else {
            if (millis() - last_menu_entry_change > 250) settings.clicks_per_liter += dir ? 1 : -1;
            else if (millis() - last_menu_entry_change > 150) settings.clicks_per_liter += dir ? 10 : -10;
            else settings.clicks_per_liter += dir ? 100 : -100;

            if (settings.clicks_per_liter > 60000 /*overflow*/) settings.clicks_per_liter = 1;
            if (settings.clicks_per_liter >= 50000) settings.clicks_per_liter = 50000;
          }
          break;
      }
      break;

    case PAGE_PUMP:
      switch (menu_entry_cursor) {
        case 1:
          settings.pump_timeout += dir ? 1 : -1;
          if (settings.pump_timeout > 250) settings.pump_timeout = 250;
          if (settings.pump_timeout < 0) settings.pump_timeout = 0;
          break;
      }
      break;

    case PAGE_CLOCK1:
      switch (menu_entry_cursor) {
        case 1:
          current_time.Hour += dir ? 1 : -1;
          if (current_time.Hour < 0 or current_time.Hour > 25) current_time.Hour = 0;
          if (current_time.Hour >= 23) current_time.Hour = 23;
          break;
        case 2:
          current_time.Minute += dir ? 1 : -1;
          if (current_time.Minute < 0 or current_time.Minute > 64) current_time.Minute = 0;
          if (current_time.Minute >= 59) current_time.Minute = 59;
          break;
        case 3:
          current_time.Second += dir ? 1 : -1;
          if (current_time.Second < 0 or current_time.Second > 64) current_time.Second = 0;
          if (current_time.Second >= 59) current_time.Second = 59;
          break;
        default:
          break;
      }
      RTC.write(current_time);
      break;

    case PAGE_CLOCK2:
      switch (menu_entry_cursor) {
        case 1:
          current_time.Day += dir ? 1 : -1;
          if (current_time.Day >= 31) current_time.Day = 31;
          if (current_time.Day < 1) current_time.Day = 1;
          break;
        case 2:
          current_time.Month += dir ? 1 : -1;
          if (current_time.Month >= 12) current_time.Month = 12;
          if (current_time.Month < 1) current_time.Month = 1;
          break;
        case 3:
          current_time.Year += dir ? 1 : -1;
          break;
        default:
          break;
      }
      RTC.write(current_time);
      break;

    case PAGE_BATTERY:
      switch (menu_entry_cursor) {
        case 1:
          settings.battery_voltage_cutoff += dir ? 0.1 : -0.1;
          if (settings.battery_voltage_cutoff >= 48) settings.battery_voltage_cutoff = ceil(1023 / settings.battery_voltage_adc_divider);
          if (settings.battery_voltage_cutoff < 0) settings.battery_voltage_cutoff = 0;
          break;
        case 2:
          settings.battery_voltage_reset += dir ? 0.1 : -0.1;
          if (settings.battery_voltage_reset >= 48) settings.battery_voltage_reset = ceil(1023 / settings.battery_voltage_adc_divider);
          if (settings.battery_voltage_reset < 0) settings.battery_voltage_reset = 0;
          break;
          break;
      }
      break;

    case PAGE_SENSORS:  //only toggles, all done in edit_change_callback()
    default:
    case -1:
      break;
  }
  last_menu_entry_change = millis();
}

void edit_change_callback() {
  switch (menu_page) {
    case PAGE_MAN:
      if (menu_entry_cursor == 1) {
        if (tank_fillings_remaining == 0) {
          if (!menu_editing) {  //if leaving edit mode
            if (settings.tank_capacity > 0) tank_fillings_remaining = literToTanks(page_1_irrigate_order);
            else tank_fillings_remaining = page_1_irrigate_order;
            /*//noser Serial.print(F("Manual water call for "));
              //noser Serial.print(page_1_irrigate_order);
              //noser Serial.print(F("L wich makes "));
              //noser Serial.print(tank_fillings_remaining);
              //noser Serial.println(F(" tank fillings"));*/
          }
        } else {
          system_state = STATUS_IDLE;
          tank_fillings_remaining = 0;
          menu_editing = false;
          ////noser Serial.print(F("Watering canceled"));
        }
      }
      break;

    case PAGE_RAIN:
      switch (menu_entry_cursor) {
        case 1:
          settings.block_water_after_rain = !settings.block_water_after_rain;
          menu_editing = false;
          EEPROM.put(0, settings);
          break;

        case 2:
        case 3:
          if (!menu_editing) EEPROM.put(0, settings);
          break;
      }
      break;

    case PAGE_TIMER:
      if (!menu_editing) {  //if leaving edit mode
        if (menu_entry_cursor > 0) {
          //if the time already happened today, be done.
          if (irrigation_timer.start_hour > current_time.Hour) irrigation_timer.last_watering_day = 0;
          else if (irrigation_timer.start_hour == current_time.Hour and irrigation_timer.start_minute > current_time.Minute) irrigation_timer.last_watering_day = 0;
          if (irrigation_timer.start_hour < current_time.Hour) irrigation_timer.last_watering_day = current_time.Day;
          else if (irrigation_timer.start_hour == current_time.Hour and irrigation_timer.start_minute <= current_time.Minute) irrigation_timer.last_watering_day = current_time.Day;
          EEPROM.put(0 + sizeof(settings), irrigation_timer);  //save timer settings when leaving
        }
      }
      break;

    case PAGE_LORA:
      if (menu_entry_cursor == 3) {
        for (uint8_t b = 0; b < 16; b++) settings.lora_security_key[b] = random(0, 255);
        EEPROM.put(0, settings);
        menu_editing = false;
      }
      if (menu_entry_cursor == 2 and !menu_editing) redraw_display_fully = true;
      if (menu_entry_cursor == 1) {
        settings.lora_enable++;
        if (settings.lora_enable >= 3) settings.lora_enable = 0;
        EEPROM.put(0, settings);
        menu_editing = false;
      }
      break;

    case PAGE_TANK:
    case PAGE_PUMP:
    case PAGE_BATTERY:
      if (!menu_editing) {  //if leaving edit mode
        if (menu_entry_cursor > 0) {
          EEPROM.put(0, settings);  //save settings when leaving
        }
      }
      break;

    case PAGE_CLOCK1:
      break;
    case PAGE_CLOCK2:
      break;

    case PAGE_SENSORS:
      switch (menu_entry_cursor) {
        case 1:
          settings.tank_top_on_level = !settings.tank_top_on_level;
          break;
        case 2:
          settings.tank_bottom_on_level = !settings.tank_bottom_on_level;
          break;
        case 3:
          settings.low_water_on_level = !settings.low_water_on_level;
          break;

        default:
          break;
      }
      EEPROM.put(0, settings);
      if (menu_entry_cursor > 0) menu_editing = false;
      break;

    default:
    case -1:
      break;
  }
}

void up_callback() {
  if (menu_editing) {
    change_menu_entry(true);
  } else {
    menu_entry_cursor++;
    if (menu_entry_cursor > page_max_cursor[menu_page]) menu_entry_cursor = page_max_cursor[menu_page];
  }
  last_display_update = 0;
  last_interface_interaction = millis();
}

void down_callback() {
  if (menu_editing) {
    change_menu_entry(false);
  } else {
    menu_entry_cursor--;
    if (menu_entry_cursor < 0) menu_entry_cursor = 0;
  }
  last_display_update = 0;
  last_interface_interaction = millis();
}

void btn_callback() {
  menu_editing = !menu_editing;
  edit_change_callback();
  last_display_update = 0;
  last_interface_interaction = millis();
}

uint32_t last_btn_down = 0;
ISR(PCINT2_vect) {  //port D
  delay(1);
  if (millis() - last_btn_down >= DEBOUNCE_DELAY) {
    if (digitalRead(BTN_PIN)) return;  //ignore rising edge
    last_btn_down = millis();
    if (button_queue_add_pos >= 32) return;  //too many buttons in queue, ignoring
    button_queue[button_queue_add_pos] = 3;  //3 is for enter button
    button_queue_add_pos++;
  }
}

uint32_t last_encoder_clock = 0;
void handle_encoder_clk() {
  delay(2);
  bool immidiate_dt = digitalRead(ENCODER_DT_PIN);  //read dt as quickly as possible

  if (digitalRead(ENCODER_CLK_PIN)) return;

  if (button_queue_add_pos >= 32) return;  //too many buttons in queue, ignoring

  if (millis() - last_encoder_clock <= DEBOUNCE_DELAY) return;  //to quick, may be a bounce
  if (immidiate_dt != ENCODER_INVERT_DIRECTION) {
    button_queue[button_queue_add_pos] = 2;  //2 is for up
  } else {
    button_queue[button_queue_add_pos] = 1;  //1 is for down
  }
  button_queue_add_pos++;
  last_encoder_clock = millis();
}

void set_status_led(bool r, bool g, bool b) {
  digitalWrite(pcf, RED_LED_PIN, !r);
  digitalWrite(pcf, GREEN_LED_PIN, !g);
  digitalWrite(pcf, BLUE_LED_PIN, !b);
}

ISR(PCINT1_vect) {                         //port B (analog pins)
  if (settings.tank_capacity > 0) return;  //against bugs when not in direct mode
  static bool water_flow_click_halver = false;
  water_flow_click_halver = !water_flow_click_halver;
  if (water_flow_click_halver) return;
  sensor_values.water_flow_clicks++;
  if (sensor_values.water_flow_clicks >= settings.clicks_per_liter)
    if (settings.tank_capacity > 0) tank_fillings_remaining--;  //decrement liter count if in direct mode
  if (tank_fillings_remaining > 60000) tank_fillings_remaining = 0;
}

void handle_lora_packet(int packet_size) {  //TODO: maybe move magic checking here
  if (packet_size <= 48) {                  //3840 is an erronious recieve for example.
    digitalWrite(pcf, LORA_RX_LED, LOW);
    for (uint8_t b = 0; b < 48; b++) lora_incoming_queue[lora_incoming_queue_idx][b] = 0;  //firstly clear

    for (uint8_t b = 0; (b < min(packet_size, 48)) and LoRa.available(); b++) {
      lora_incoming_queue[lora_incoming_queue_idx][b] = LoRa.read();
    }

    lora_incoming_queue_len[lora_incoming_queue_idx] = min(packet_size, 48);
    lora_incoming_queue_idx++;
    if (lora_incoming_queue_idx >= 4) lora_incoming_queue_idx = 0;
  }
  LoRa.flush();  //clear packet if for some reason the is anything left
}

void handle_lora_tx_done() {
  LoRa.receive();
  lora_tx_ready = true;
  lora_airtime += millis() - lora_tx_start_millis;  //add tx time
  digitalWrite(pcf, LORA_TX_LED, HIGH);
}

void array_hexprint(uint8_t* arr, uint8_t len) {
  for (uint8_t b = 0; b < len; b++) {
    if (arr[b] < 0x10) Serial.write('0');
    Serial.print(arr[b], HEX);
    Serial.write(' ');
  }
  Serial.println();
}

void setup() {
  wdt_enable(WDTO_8S);

  //pump stuff
  pinMode(LOW_WATER_PIN, INPUT_PULLUP);
  pinMode(TANK_BOTTOM_PIN, INPUT_PULLUP);
  pinMode(TANK_TOP_PIN, INPUT_PULLUP);
  pinMode(RAIN_DETECTOR_PIN, INPUT_PULLUP);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, CONTROL_RELAY_INVERTED);
  digitalWrite(VALVE_PIN, CONTROL_RELAY_INVERTED);

  //vbat
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  analogReference(INTERNAL /*1V1*/);  //set ADC voltage reference to stable internal 1.1V reference. uncomment the 1V1 part for arduino megas
  randomSeed(analogRead(BATTERY_VOLTAGE_PIN));

  //buttons
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  pinMode(BTN_PIN, INPUT_PULLUP);

  set_status_led(1, 1, 1);

  Serial.begin(9600);
  ////noser Serial.println(F("H3 Bewässerungssystem\nhttps://blog.hacker3000.cf/waterthing.php"));


  /*  wdt_reset();
    // Serial.print(F("Reset Cause: "));
     Serial.print(F("Reset: "));
    mcusr_copy = MCUSR;
    MCUSR = 0;
    if (mcusr_copy & (1 << WDRF)) {
      // Serial.print(F("WATCHDOG! Firmware may be unstable!"));
      Serial.print(F("WATCHDOG!"));
    }
    else if (mcusr_copy & (1 << BORF)) {
      // Serial.print(F("BROWNOUT! Check your Arduino's Power Suppy!"));
      Serial.print(F("BROWNOUT!"));
    }
    else if (mcusr_copy & (1 << EXTRF)) {
      // Serial.print(F("External. For example the Reset Button."));
      Serial.print(F("External."));
    }
    else if (mcusr_copy & (1 << PORF)) {
      Serial.print(F("Power on."));
    }
    Serial.println();*/

  wdt_reset();

  //noser Serial.println(F("LCD setup..."));
  Wire.setClock(400000);  //faster drawing
  lcd.init();
  lcd.backlight();
  /*lcd.createChar(GFX_ID_IDLE, gfx_idle);
    //lcd.createChar(GFX_ID_DROP, gfx_drop);
    lcd.createChar(GFX_ID_NO_WATER, gfx_no_water);
    lcd.createChar(GFX_ID_FILL, gfx_fill);
    lcd.createChar(GFX_ID_DRAIN, gfx_drain);*/
  /*lcd.createChar(GFX_ID_STATUS, gfx_error);
  lcd.createChar(GFX_ID_DYN_1, gfx_error);
  lcd.createChar(GFX_ID_DYN_2, gfx_error);
  lcd.createChar(GFX_ID_DYN_3, gfx_error);*/
  lcd.createChar(GFX_ID_UML_S, gfx_uml_s);
  lcd.createChar(GFX_ID_UML_A, gfx_uml_a);
  lcd.createChar(GFX_ID_UML_O, gfx_uml_o);
  lcd.createChar(GFX_ID_UML_U, gfx_uml_u);
  delay(50);
  lcd.clear();
  lcd.home();
  /*lcd.print(F("Bew\x05sserungs"));
  lcd.setCursor(0, 1);
  lcd.print(F("System by H3"));
  delay(1000);*/

  /*lcd.clear();
  lcd.home();
  lcd.print(F("Blog-Artikel/Doku"));
  lcd.setCursor(0, 1);
  lcd.print(F("-> hacker3000.cf"));
  delay(1000);*/

  wdt_reset();

  //check for LED pcf
  // Serial.println(F("Checking for LED PCF..."));
  lcd.clear();
  lcd.home();
  lcd.print(F("LED PCF8574..."));
  lcd.setCursor(0, 1);
  Wire.beginTransmission(LED_PCF_ADDR);
  if (Wire.endTransmission() == 0) {
    for (uint8_t pin = 0; pin < 8; pin++) {
      pinMode(pcf, pin, OUTPUT);
      digitalWrite(pcf, pin, HIGH);
      delay(50);
    }
    for (uint8_t pin = 0; pin < 8; pin++) {
      digitalWrite(pcf, pin, LOW);
      delay(50);
    }
    lcd.print(F("OK"));
  } else {
    lcd.print(F("Missing"));
    //Serial.println(F("LED PCF missing."));
    //noser Serial.println(F("PCF"));
    delay(900);
  }
  delay(100);

  wdt_reset();

  //EEPROM
  // Serial.println(F("Reading EEPROM..."));
  lcd.clear();
  lcd.home();
  lcd.print(F("EEPROM..."));
  lcd.setCursor(0, 1);
  lcd.print(F("Halte f\x07r Reset"));
  delay(900);
  lcd.setCursor(0, 1);
  for (uint8_t c = 0; c < 16; c++) lcd.write(' ');
  lcd.setCursor(0, 1);
  if ((EEPROM.read(EEPROM.length() - 1) == EEPROM_MAGIC_NUMBER) and digitalRead(BTN_PIN)) {  //hold btn on boot to reset
    EEPROM.get(0, settings);
    EEPROM.get(0 + sizeof(settings), irrigation_timer);
    //EEPROM.get(0+sizeof(settings)+sizeof(irrigation_timer), something_else);
    lcd.print(F("OK"));
  } else {
    for (uint8_t b = 0; b < 16; b++) settings.lora_security_key[b] = random(0, 255);
    EEPROM.put(0, settings);
    EEPROM.put(0 + sizeof(settings), irrigation_timer);
    //EEPROM.put(0+sizeof(settings)+sizeof(irrigation_timer), something_else);
    EEPROM.write(EEPROM.length() - 1, EEPROM_MAGIC_NUMBER);
    lcd.print(F("Initialized"));
    delay(900);
  }
  delay(100);

  wdt_reset();

  //rtc setup
  // Serial.println(F("Setting up RTC..."));
  lcd.clear();
  lcd.home();
  lcd.print(F("RTC..."));
  lcd.setCursor(0, 1);
  if (RTC.read(current_time)) {  //if valid date read
    lcd.print(F("OK"));
  } else {
    if (RTC.chipPresent()) {  //if date invalid but RTC present
      String rtc_unset = F("RTC NOT SET");
      Serial.println(rtc_unset);
      lcd.print(rtc_unset);
      delay(1000);
    } else {  //if RTC missing
      String rtc_missing = F("RTC MISSING!");
      Serial.println(rtc_missing);
      lcd.print(rtc_missing);
      while (true) {
        set_status_led(1, 0, 0);
        delay(100);
        set_status_led(0, 0, 0);
        delay(100);
      }
    }
  }
  delay(100);

  wdt_reset();

  // Serial.println(F("Setting up LoRa..."));
  lcd.clear();
  lcd.home();
  lcd.print(F("LoRa..."));
  lcd.setCursor(0, 1);
  LoRa.setPins(10, 9, 2);
  LoRa.setSPIFrequency(1E6);  //1mhz is way fast
  if (LoRa.begin(LORA_FREQ)) {
    LoRa.idle();
    LoRa.setSyncWord(0x12);
    LoRa.setTxPower(LORA_TX_PWR);
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(8);  //sf,bw,cr make a data rate of 366 bits per sec or 45,75 bytes per sec
    LoRa.enableCrc();
    //LoRa.onTxDone(handle_lora_tx_done);
    //LoRa.onReceive(handle_lora_packet);
    LoRa.receive();

    randomSeed(LoRa.random());

    //boot packet
    lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
    lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
    lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_REBOOT;

    lora_outgoing_queue_last_tx = millis() - LORA_RETRANSMIT_TIME + 3000;
    lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] = 0;
    lora_outgoing_packet_id++;
    if (lora_outgoing_packet_id < 1) lora_outgoing_packet_id == 1;  //never let it go to 0, that causes bugs
    lora_outgoing_queue_idx++;
    if (lora_outgoing_queue_idx >= 4) lora_outgoing_queue_idx = 0;

    lcd.print(F("OK"));
    component_errors.lora_missing = false;
  } else {
    lcd.print(F("Missing"));
    Serial.println(F("LoRa missing."));
    delay(900);
    component_errors.lora_missing = true;
  }
  delay(100);

  wdt_reset();

  //buttons etc
  for (uint8_t q_pos = 0; q_pos < 32; q_pos++) {  //make sure they are 0
    button_queue[q_pos] = 0;
  }

  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), handle_encoder_clk, FALLING);

  //button
  //              dcb
  PCICR |= 0b00000100;  //allow pcint on port d
  //      Pin 76543210
  PCMSK2 |= 0b00100000;  //enable pcint for pin

  //water sensor
  //              dcb
  PCICR |= 0b00000010;  //allow pcint on port c
  //      Pin 76543210
  PCMSK1 |= 0b00001000;  //enable pcint for pin A3


  // Serial.println(F("System Started."));
  Serial.println();
  set_status_led(0, 0, 0);
  wdt_reset();
}

void print_page_basics() {
  static uint8_t last_sys_state = 255;
  static uint8_t last_page = 255;
  static uint8_t last_min = 255;
  static uint8_t last_cursor = 255;
  static bool last_rain = false;
  if (last_sys_state == system_state and last_page == menu_page and last_min == current_time.Minute and (menu_entry_cursor + menu_editing ? 0x80 : 0) == last_cursor and last_rain == sensor_values.rain_detected and !redraw_display_fully) goto skip_page_basics;

  switch (system_state) {
    case STATUS_IDLE:
      if (settings.tank_capacity > 0 ? irrigation_timer.fillings_to_irrigate == 0 : irrigation_timer.liters_to_pump == 0) lcd.createChar(GFX_ID_STATUS, gfx_off);
      else if (irrigation_timer.last_watering_day == current_time.Day) lcd.createChar(GFX_ID_STATUS, gfx_done);
      else if (sensor_values.rain_detected) lcd.createChar(GFX_ID_STATUS, gfx_rain_detected);
      else lcd.createChar(GFX_ID_STATUS, gfx_idle);
      break;
    case STATUS_PUMPING:
      lcd.createChar(GFX_ID_STATUS, gfx_fill);
      break;
    case STATUS_EMPTYING:
      lcd.createChar(GFX_ID_STATUS, gfx_drain);
      break;
    case STATUS_AFTERDRAIN:
      lcd.createChar(GFX_ID_STATUS, gfx_afterdrain);
      break;
    case STATUS_NO_WATER:
      lcd.createChar(GFX_ID_STATUS, gfx_no_water);
      break;
    case STATUS_LOW_BATTERY:
      lcd.createChar(GFX_ID_STATUS, gfx_low_bat);
      break;
    case STATUS_NO_TIME:
    case STATUS_GENERAL_FAIL:
    default:
      lcd.createChar(GFX_ID_STATUS, gfx_error);
      break;
  }

  lcd.clear();
  lcd.home();
  lcd_print_menu_bracket(0, false);
  lcd.print(menu_page);
  lcd_print_menu_bracket(0, true);
  lcd.print(F(""));
  lcd.print(page_names[menu_page]);
  lcd.setCursor(10, 0);

  lcd.write(byte(GFX_ID_STATUS));
  char clock_buf[5];
  sprintf(clock_buf, "%02u:%02u", current_time.Hour, current_time.Minute);
  lcd.print(clock_buf);

skip_page_basics:
  lcd.setCursor(0, 1);

  last_cursor = menu_entry_cursor + menu_editing ? 0x80 : 0;
  last_sys_state = system_state;
  last_page = menu_page;
  last_min = current_time.Minute;
  last_rain = sensor_values.rain_detected;
  redraw_display_fully = false;
}

void update_display() {
  if (millis() - last_display_update > 1000) {
    //digitalWrite(pcf, ACTIVITY_LED, LOW);
    uint16_t uint16_temp;
    char char_5_temp[5];
    uint8_t disp_pad = 8;
    //uint32_t display_draw_start_time = millis();
    print_page_basics();
    switch (menu_page) {
      default:
        // Serial.println(F("Unknown Menu Entry Selected!"));
        lcd.print(F("PAGE UNKNOWN!!"));
        //menu_page = 0;
        break;
      case PAGE_STATUS:
        switch (system_state) {
          case STATUS_IDLE:
            if (settings.tank_capacity > 0 ? irrigation_timer.fillings_to_irrigate == 0 : irrigation_timer.liters_to_pump == 0) {
              lcd.print(F("Ausgeschaltet"));
            } else if (irrigation_timer.last_watering_day == current_time.Day) {
              lcd.print(F("Fertig f\x07r Heute"));
            } else if (sensor_values.rain_detected) {
              lcd.print(F("Regen erkannt"));
            } else {
              lcd.print(F("Stby. bis"));
              lcd.setCursor(11, 1);
              sprintf(char_5_temp, "%02u:%02u", irrigation_timer.start_hour, irrigation_timer.start_minute);
              lcd.print(char_5_temp);
            }
            break;
          case STATUS_EMPTYING:
            lcd.print(F("Leere Tank  "));
            lcd.print(tank_fillings_remaining);
            lcd.print(F("/"));
            lcd.print(settings.tank_capacity > 0 ? irrigation_timer.fillings_to_irrigate : irrigation_timer.liters_to_pump);
            break;
          case STATUS_PUMPING:
            lcd.print(F("F\x07lle Tank  "));
            lcd.print(tank_fillings_remaining);
            lcd.print(F("/"));
            lcd.print(settings.tank_capacity > 0 ? irrigation_timer.fillings_to_irrigate : irrigation_timer.liters_to_pump);
            break;
          case STATUS_AFTERDRAIN:
            char ad_buf[16];
            if (settings.afterdrain_time < 15) {
              sprintf(ad_buf, "Nachlauf %03u/%03u", ((uint64_t)settings.afterdrain_time * 60) - (uint64_t(millis() - cycle_finish_millis) / 1000L), settings.afterdrain_time * 60);
            } else {
              sprintf(ad_buf, "Nachlauf %02um/%02um", (uint64_t)settings.afterdrain_time - ((uint64_t(millis() - cycle_finish_millis) / 1000L) / 60L), settings.afterdrain_time);
            }
            lcd.print(ad_buf);
            break;
          case STATUS_NO_WATER:
            lcd.print(F("WASSER LEER!!"));
            break;
          case STATUS_NO_TIME:
            lcd.print(F("Uhrzeit fehlt!"));
            break;
          case STATUS_LOW_BATTERY:
            lcd.print(F("Akku Leer!"));
            lcd.setCursor(11, 1);
            dtostrf(sensor_values.battery_voltage, 4, 1, char_5_temp);
            lcd.print(char_5_temp);
            lcd.write('V');
            break;

          default:
          case STATUS_GENERAL_FAIL:
            bool no_error_desc_found = true;

            if (component_errors.pump_timed_out) {
              lcd.print(F("Pump "));
              no_error_desc_found = false;
            }

            if (component_errors.tank_sensors_irrational) {  //tank sensor error
              lcd.print(F("TS "));
              no_error_desc_found = false;
            }

            if (component_errors.rtc_missing or component_errors.rtc_unset) {  //rtc error
              lcd.print(F("RTC "));
              no_error_desc_found = false;
            }

            if (no_error_desc_found) lcd.print(F("Algem. "));

            lcd.print(F("Fehler"));

            if (component_errors.rtc_unset) system_state = STATUS_NO_TIME;
            break;
        }
        break;
      case PAGE_MAN:
        if (tank_fillings_remaining == 0 and system_state == STATUS_IDLE) {
          lcd.print(F("Gie\x04\x65 "));
          lcd_print_menu_bracket(1, false);
          lcd.print(page_1_irrigate_order);
          lcd.print('L');
          lcd_print_menu_bracket(1, true);
          lcd.print(F(" jetzt"));  //the jetzt gets cut off when selecting more than 0L but thats not too bad
        } else {
          lcd_print_menu_bracket(1, false);
          lcd.print(F("Abbrechen"));
          lcd_print_menu_bracket(1, true);
        }
        disp_pad = 0;
        break;

      case PAGE_TIMER:
        lcd_print_menu_bracket(1, false);
        if (irrigation_timer.start_hour < 10) lcd.print(0);
        lcd.print(irrigation_timer.start_hour);
        lcd_print_menu_bracket(1, true);
        lcd.print(F(":"));
        lcd_print_menu_bracket(2, false);
        if (irrigation_timer.start_minute < 10) lcd.print(0);
        lcd.print(irrigation_timer.start_minute);
        lcd_print_menu_bracket(2, true);
        lcd.write(byte(126));
        lcd_print_menu_bracket(3, false);
        uint16_temp = settings.tank_capacity > 0 ? round(irrigation_timer.fillings_to_irrigate * settings.tank_capacity) : irrigation_timer.liters_to_pump;
        if (uint16_temp < 10) lcd.print(0);
        if (uint16_temp < 100) lcd.print(0);
        lcd.print(uint16_temp);
        lcd.print(F("L"));
        lcd_print_menu_bracket(3, true);
        disp_pad = 4;
        break;

      case PAGE_RAIN:
        lcd.createChar(GFX_ID_DYN_1, gfx_rain);
        lcd.createChar(GFX_ID_DYN_2, gfx_rise);
        lcd.createChar(GFX_ID_DYN_3, gfx_fall);
        lcd.setCursor(0, 1);  //this is needed so the disp takes the custom char

        lcd.write(GFX_ID_DYN_1);
        lcd_print_menu_bracket(1, false);
        lcd.print(settings.block_water_after_rain ? 'B' : 'I');
        lcd_print_menu_bracket(1, true);

        lcd.write(GFX_ID_DYN_2);
        lcd_print_menu_bracket(2, false);
        lcd.print(settings.rain_minutes_til_block);
        lcd.write('m');
        lcd_print_menu_bracket(2, true);

        lcd.write(GFX_ID_DYN_3);
        lcd_print_menu_bracket(3, false);
        lcd.print(settings.rain_minutes_til_clear);
        lcd.write('m');
        lcd_print_menu_bracket(3, true);

        break;

      case PAGE_TANK:
        lcd.createChar(GFX_ID_DYN_1, gfx_drop);
        if (settings.tank_capacity > 0) lcd.createChar(GFX_ID_DYN_2, gfx_clock);
        else {
          lcd.createChar(GFX_ID_DYN_2, gfx_flow);
          lcd.createChar(GFX_ID_DYN_3, gfx_pulse);
        }
        lcd.setCursor(0, 1);  //this is needed so the disp takes the custom char

        //lcd.print(F("Vl"));
        lcd.write(byte(GFX_ID_DYN_1));
        lcd_print_menu_bracket(1, false);
        if (settings.tank_capacity > 0) {
          if (settings.tank_capacity < 10) lcd.print(0);
          if (settings.tank_capacity < 100) lcd.print(0);
          lcd.print(settings.tank_capacity);
          lcd.print(F("L"));
        } else lcd.print(F("D"));
        lcd_print_menu_bracket(1, true);

        if (settings.tank_capacity > 0) {
          lcd.setCursor(8, 1);
          lcd.write(byte(GFX_ID_DYN_2));
          //lcd.print(F("Nl"));
          lcd_print_menu_bracket(2, false);
          if (settings.afterdrain_time < 10) lcd.print(0);
          lcd.print(settings.afterdrain_time);
          lcd.print(F("min"));
          lcd_print_menu_bracket(2, true);
        } else {
          lcd.write(' ');
          lcd.write(byte(GFX_ID_DYN_2));
          lcd_print_menu_bracket(2, false);
          if (settings.clicks_per_liter < 10) lcd.print(0);
          if (settings.clicks_per_liter < 100) lcd.print(0);
          if (settings.clicks_per_liter < 1000) lcd.print(0);
          if (settings.clicks_per_liter < 10000) lcd.print(0);
          lcd.print(settings.clicks_per_liter);
          lcd.write(byte(GFX_ID_DYN_3));
          lcd.print(F("/L"));
          lcd_print_menu_bracket(2, true);
        }
        disp_pad = 0;
        break;

      case PAGE_PUMP:
        lcd.createChar(GFX_ID_DYN_1, gfx_clock);
        lcd.setCursor(0, 1);  //this is needed so the disp takes the custom char

        lcd.write(byte(GFX_ID_DYN_1));
        lcd.write('!');
        lcd_print_menu_bracket(1, false);
        if (settings.pump_timeout < 10) lcd.print(0);
        if (settings.pump_timeout < 100) lcd.print(0);
        lcd.print(settings.pump_timeout);
        lcd.print(F("min"));
        lcd_print_menu_bracket(1, true);
        disp_pad = 5;
        break;

      case PAGE_LORA:
        if (menu_page == PAGE_LORA and menu_entry_cursor == 2 and menu_editing) {
          lcd.setCursor(0, 0);
          for (uint8_t b = 0; b < 8; b++) {
            if (settings.lora_security_key[b] < 0x10) lcd.print(0);
            lcd.print(settings.lora_security_key[b], HEX);
          }
          lcd.setCursor(0, 1);
          for (uint8_t b = 8; b < 16; b++) {
            if (settings.lora_security_key[b] < 0x10) lcd.print(0);
            lcd.print(settings.lora_security_key[b], HEX);
          }
        } else {
          lcd.createChar(GFX_ID_DYN_1, gfx_radio);
          lcd.createChar(GFX_ID_DYN_3, gfx_key);
          lcd.setCursor(0, 1);

          lcd.write(GFX_ID_DYN_1);
          lcd_print_menu_bracket(1, false);
          switch (settings.lora_enable) {
            case 0:
              lcd.print(F("OFF"));
              break;
            case 1:
              lcd.print(F("TX"));
              break;
            case 2:
              lcd.print(F("TX+RX"));
              break;

            default:
              lcd.write('?');
              break;
          }
          lcd_print_menu_bracket(1, true);

          lcd_print_menu_bracket(2, false);
          lcd.write('V');
          lcd.write(GFX_ID_DYN_3);
          lcd_print_menu_bracket(2, true);

          lcd_print_menu_bracket(3, false);
          lcd.write('G');
          lcd.write(GFX_ID_DYN_3);
          lcd_print_menu_bracket(3, true);
        }
        break;

      case PAGE_CLOCK1:
        lcd_print_menu_bracket(1, false);
        if (current_time.Hour < 10) lcd.print(0);
        lcd.print(current_time.Hour);
        lcd_print_menu_bracket(1, true);
        lcd.print(F(":"));
        lcd_print_menu_bracket(2, false);
        if (current_time.Minute < 10) lcd.print(0);
        lcd.print(current_time.Minute);
        lcd_print_menu_bracket(2, true);
        lcd.print(F(":"));
        lcd_print_menu_bracket(3, false);
        if (current_time.Second < 10) lcd.print(0);
        lcd.print(current_time.Second);
        lcd_print_menu_bracket(3, true);
        break;

      case PAGE_CLOCK2:
        lcd_print_menu_bracket(1, false);
        if (current_time.Day < 10) lcd.print(0);
        lcd.print(current_time.Day);
        lcd_print_menu_bracket(1, true);
        lcd.print(F("."));
        lcd_print_menu_bracket(2, false);
        if (current_time.Month < 10) lcd.print(0);
        lcd.print(current_time.Month);
        lcd_print_menu_bracket(2, true);
        lcd.print(F("."));
        lcd_print_menu_bracket(3, false);
        lcd.print(tmYearToCalendar(current_time.Year));
        lcd_print_menu_bracket(3, true);
        break;

      case PAGE_BATTERY:
        lcd.createChar(GFX_ID_DYN_3, gfx_hyst);
        lcd.setCursor(0, 1);  //this is needed so the disp takes the custom char

        lcd.write('0');

        //lcd.print(F("OFF"));
        //lcd.write(byte(GFX_ID_ARROW_L));
        lcd_print_menu_bracket(1, false);
        dtostrf(settings.battery_voltage_cutoff, 4, 1, char_5_temp);
        lcd.print(char_5_temp);
        lcd_print_menu_bracket(1, true);

        lcd.write(byte(GFX_ID_DYN_3));

        lcd_print_menu_bracket(2, false);
        dtostrf(settings.battery_voltage_reset, 4, 1, char_5_temp);
        lcd.print(char_5_temp);
        lcd_print_menu_bracket(2, true);

        //lcd.write(byte(GFX_ID_ARROW_R));
        //lcd.print(F("ON"));

        lcd.write('1');

        disp_pad = 4;
        break;

      case PAGE_SENSORS:
        lcd.createChar(GFX_ID_DYN_1, gfx_tank_top);
        lcd.createChar(GFX_ID_DYN_2, gfx_tank_bottom);
        lcd.createChar(GFX_ID_DYN_3, gfx_no_water);
        lcd.setCursor(0, 1);  //this is needed so the disp takes the custom char

        lcd.write(GFX_ID_DYN_1);
        lcd_print_menu_bracket(1, false);
        lcd.print(settings.tank_top_on_level ? 'H' : 'L');
        lcd_print_menu_bracket(1, true);

        lcd.write(' ');
        lcd.write(GFX_ID_DYN_2);
        lcd_print_menu_bracket(2, false);
        lcd.print(settings.tank_bottom_on_level ? 'H' : 'L');
        lcd_print_menu_bracket(2, true);

        lcd.write(' ');
        lcd.write(GFX_ID_DYN_3);
        lcd_print_menu_bracket(3, false);
        lcd.print(settings.low_water_on_level ? 'H' : 'L');
        lcd_print_menu_bracket(3, true);

        disp_pad = 2;
        break;
    }
    last_display_update = millis();

    for (; disp_pad < 250; disp_pad--) lcd.print(' ');  //make sure rest of line is clear. also doin it the wrong way round :P

    if (millis() - last_interface_interaction > LCD_BACKLIGHT_TIMEOUT) lcd.noBacklight();
    else lcd.backlight();

    // Serial.print(F("Display drawing time [ms]: "));
    // Serial.println(last_display_update - display_draw_start_time);
    //digitalWrite(pcf, ACTIVITY_LED, HIGH);
  }

  //status led
  switch (system_state) {
    case STATUS_IDLE:
      if (settings.tank_capacity > 0 ? irrigation_timer.fillings_to_irrigate == 0 : irrigation_timer.liters_to_pump == 0) set_status_led(1, 0, 1);  //turned off
      else if (irrigation_timer.last_watering_day == current_time.Day) set_status_led(0, 1, 0);                                                     //done
      else if (sensor_values.rain_detected) set_status_led(1, 1, 0);                                                                                //rain
      else set_status_led(0, 1, 0);                                                                                                                 //waiting
      break;
    case STATUS_AFTERDRAIN:
    case STATUS_EMPTYING:
      set_status_led(0, 1, 1);
      break;
    case STATUS_PUMPING:
      set_status_led(0, 0, 1);
      break;
    case STATUS_NO_WATER:
    case STATUS_NO_TIME:
    case STATUS_LOW_BATTERY:
      set_status_led(1, 0, 0);
      break;

    default:
    case STATUS_GENERAL_FAIL:
      set_status_led(1, 0, 0);
      break;
  }

  digitalWrite(pcf, PUMP_LED, digitalRead(PUMP_PIN));
  digitalWrite(pcf, TT_SENSOR_LED, !sensor_values.tank_top);
  digitalWrite(pcf, TB_SENSOR_LED, !sensor_values.tank_bottom);
}

void handle_pump_stuff() {
  if ((sensor_values.tank_top and !sensor_values.tank_bottom) and settings.tank_capacity > 0) {
    component_errors.tank_sensors_irrational = true;
    system_state = STATUS_GENERAL_FAIL;
  }
  if ((irrigation_timer.start_hour <= current_time.Hour and irrigation_timer.start_minute <= current_time.Minute) and (irrigation_timer.last_watering_day != current_time.Day) and !sensor_values.rain_detected) {  //
    tank_fillings_remaining += (settings.tank_capacity > 0) ? irrigation_timer.fillings_to_irrigate : irrigation_timer.liters_to_pump;
    irrigation_timer.last_watering_day = current_time.Day;
    EEPROM.put(0 + sizeof(settings), irrigation_timer);
  }
  switch (system_state) {
    case STATUS_IDLE:
      digitalWrite(PUMP_PIN, CONTROL_RELAY_INVERTED);
      digitalWrite(VALVE_PIN, CONTROL_RELAY_INVERTED);
      //the further sth is down in this case, the higher it's priority

      if (sensor_values.battery_voltage <= settings.battery_voltage_cutoff and tank_fillings_remaining == 0) system_state = STATUS_LOW_BATTERY;  //low battery is lowest priority. this is so a currently running irrigation is completed even if the pump motor drops the voltage

      if (tank_fillings_remaining > 0 and !sensor_values.low_water) {
        sensor_values.water_flow_clicks = 0;
        pumping_start_millis = millis();
        system_state = STATUS_PUMPING;
      }

      if (sensor_values.low_water) system_state = STATUS_NO_WATER;
      if (component_errors.rtc_unset) system_state = STATUS_NO_TIME;
      if (component_errors.rtc_missing) system_state = STATUS_GENERAL_FAIL;
      if (millis() - cycle_finish_millis < (uint64_t(settings.afterdrain_time) * 60L * 1000L)) system_state = STATUS_AFTERDRAIN;
      if (sensor_values.tank_bottom) system_state = STATUS_EMPTYING;  //if at boot there is still water in the tank, empty it.
      break;

    case STATUS_AFTERDRAIN:
      digitalWrite(PUMP_PIN, CONTROL_RELAY_INVERTED);
      digitalWrite(VALVE_PIN, !CONTROL_RELAY_INVERTED);
      if (tank_fillings_remaining > 0) {
        sensor_values.water_flow_clicks = 0;  //return if for some reason there is more irrigation commanded
        pumping_start_millis = millis();
        system_state = STATUS_PUMPING;
      }
      if (millis() - cycle_finish_millis > (uint64_t(settings.afterdrain_time) * 60L * 1000L)) system_state = STATUS_IDLE;
      break;

    case STATUS_EMPTYING:
      digitalWrite(PUMP_PIN, CONTROL_RELAY_INVERTED);
      digitalWrite(VALVE_PIN, !CONTROL_RELAY_INVERTED);
      if (!sensor_values.tank_bottom and !sensor_values.tank_top) {
        tank_fillings_remaining--;
        if (tank_fillings_remaining > 60000) tank_fillings_remaining = 0;  //against integer rollover
        if (sensor_values.low_water) {
          system_state = STATUS_NO_WATER;
          return;
        }
        if (tank_fillings_remaining == 0) {
          cycle_finish_millis = millis();
          system_state = STATUS_AFTERDRAIN;
        } else {
          pumping_start_millis = millis();
          system_state = STATUS_PUMPING;
        }
      }
      break;

    case STATUS_PUMPING:
      digitalWrite(PUMP_PIN, !CONTROL_RELAY_INVERTED);
      digitalWrite(VALVE_PIN, CONTROL_RELAY_INVERTED);
      if ((settings.pump_timeout > 0) and (millis() - pumping_start_millis > uint32_t(settings.pump_timeout) * 60L * 1000L)) {
        component_errors.pump_timed_out = true;
        system_state = STATUS_GENERAL_FAIL;
      }

      else if (settings.tank_capacity > 0) {
        if (sensor_values.tank_top or sensor_values.low_water) {
          system_state = STATUS_EMPTYING;
        }
      } else {
        if (tank_fillings_remaining == 0 /*sensor_values.water_flow_clicks > (settings.clicks_per_liter * irrigation_timer.liters_to_pump)*/ or sensor_values.low_water) {
          system_state = STATUS_IDLE;
        }
      }
      break;

    case STATUS_NO_WATER:
      digitalWrite(PUMP_PIN, CONTROL_RELAY_INVERTED);
      digitalWrite(VALVE_PIN, CONTROL_RELAY_INVERTED);
      if (!sensor_values.low_water) system_state = STATUS_IDLE;
      break;

    case STATUS_NO_TIME:
      digitalWrite(PUMP_PIN, CONTROL_RELAY_INVERTED);
      digitalWrite(VALVE_PIN, CONTROL_RELAY_INVERTED);
      if (!component_errors.rtc_unset) {
        system_state = STATUS_IDLE;
        return;
      }
      break;

    case STATUS_LOW_BATTERY:
      digitalWrite(PUMP_PIN, CONTROL_RELAY_INVERTED);
      digitalWrite(VALVE_PIN, CONTROL_RELAY_INVERTED);
      if (sensor_values.battery_voltage >= settings.battery_voltage_reset) system_state = STATUS_IDLE;
      break;

    default:
    case STATUS_GENERAL_FAIL:
      digitalWrite(PUMP_PIN, CONTROL_RELAY_INVERTED);
      digitalWrite(VALVE_PIN, CONTROL_RELAY_INVERTED);
      /*no returning from pump timeout*/
      if (((sensor_values.tank_top) <= (sensor_values.tank_bottom)) or (settings.tank_capacity == 0)) component_errors.tank_sensors_irrational = false;  //if the top sensor is the same or lower than the bottom sensor, its fine
      if (!component_errors.rtc_missing and !component_errors.tank_sensors_irrational and !component_errors.pump_timed_out) system_state = STATUS_IDLE;
      break;
  }
}

void read_sensors_and_clock() {
  if (RTC.read(current_time)) {
    component_errors.rtc_missing = false;
    component_errors.rtc_unset = false;
  } else {
    if (RTC.chipPresent()) {
      component_errors.rtc_missing = false;
      component_errors.rtc_unset = true;
      system_state = STATUS_NO_TIME;
    } else {
      component_errors.rtc_missing = true;
      component_errors.rtc_unset = false;
    }
  }
  sensor_values.battery_voltage = (float)analogRead(BATTERY_VOLTAGE_PIN) / settings.battery_voltage_adc_divider;

  //float sw debounce
  bool lw = (settings.low_water_on_level == digitalRead(LOW_WATER_PIN));
  static bool last_changed_lw = false;
  static uint32_t last_lw_change = 0;
  if (lw != last_changed_lw) {
    last_lw_change = millis();
    last_changed_lw = lw;
  }
  if (millis() - last_lw_change > SENSOR_DEBOUNCE) sensor_values.low_water = lw;

  if (settings.tank_capacity > 0) {  //ignore in direct mode
    bool tb = (settings.tank_bottom_on_level == digitalRead(TANK_BOTTOM_PIN));
    static bool last_changed_tb = false;
    static uint32_t last_tb_change = 0;
    if (tb != last_changed_tb) {
      last_tb_change = millis();
      last_changed_tb = tb;
    }
    if (millis() - last_tb_change > SENSOR_DEBOUNCE) sensor_values.tank_bottom = tb;

    bool tt = (settings.tank_top_on_level == digitalRead(TANK_TOP_PIN));
    static bool last_changed_tt = false;
    static uint32_t last_tt_change = 0;
    if (tt != last_changed_tt) {
      last_tt_change = millis();
      last_changed_tt = tt;
    }
    if (millis() - last_tt_change > SENSOR_DEBOUNCE) sensor_values.tank_top = tt;
  }


  //rain

  //debounce input
  bool rain_condition_raw = (settings.rain_detected_on_level == digitalRead(RAIN_DETECTOR_PIN));
  static bool rain_condition_now = false;
  static bool last_changed_db_rc = false;
  static uint32_t last_db_rc_change = 0;
  if (rain_condition_raw != last_changed_db_rc) {
    last_db_rc_change = millis();
    last_changed_db_rc = rain_condition_raw;
  }
  if (millis() - last_db_rc_change > SENSOR_DEBOUNCE * 10) rain_condition_now = rain_condition_raw;


  static uint32_t last_rc_change = 0;
  static bool last_rain_condition = false;

  //only set start/end millis on change
  if (last_rain_condition != rain_condition_now) {
    if (rain_condition_now) {
      //dont set rain_start while currently detecting rain so that a short interruption of the signal does not un-detect it for the til_block time
      if (!sensor_values.rain_detected) sensor_values.rain_start_millis = millis();

    } else sensor_values.rain_end_millis = millis();
    last_rain_condition = rain_condition_now;
  }

  //TL;DR if start time is long enough ago OR end time is recent enough THEN rain is detected
  sensor_values.rain_detected =
    //if difference BETWEEN current time /*OR last rains time*/ COMPARED TO rain start IS BIGGER THAN configured time
    (/*sensor_values.rain_end_millis - sensor_values.rain_start_millis > (settings.rain_minutes_til_block * 60L * 1000L)
     or*/
     millis() - (uint32_t)sensor_values.rain_start_millis
     > ((uint32_t)settings.rain_minutes_til_block * 60L * 1000L))

    and  //AND

    //current time minus end time IS SMALLER THAN configured time OR its raining
    (
      (
        (millis() - sensor_values.rain_end_millis
         < (uint32_t)settings.rain_minutes_til_clear * 60L * 1000L)
         
        and (sensor_values.rain_end_millis - sensor_values.rain_start_millis > settings.rain_minutes_til_block * 60L * 1000L))

      /*or (millis() - sensor_values.rain_start_millis < (settings.rain_minutes_til_clear * 60L * 1000L) * /)*/
      or rain_condition_now)

    and settings.block_water_after_rain  //AND rain is set to block irrigation
    ;
}

void handle_serial() {
  if (Serial.available()) {
    //digitalWrite(pcf, ACTIVITY_LED, LOW);
    uint8_t controlCharacter = Serial.read();

    /*if (controlCharacter == 'd') up_callback();
    if (controlCharacter == 'a') down_callback();
    if (controlCharacter == 's') btn_callback();*/

    if (controlCharacter == 'V') {  //dump all sensor values to serial
      //Serial.println(F("Sensors:"));

      /*Serial.print(s_star);  Serial.print(F("Low Water: "));
        Serial.println((sensor_values.low_water) ? F("Water too low") : F("Water fine"));
        Serial.print(s_star);  Serial.print(F("Bottom Tank Swimmer: "));
        Serial.println((sensor_values.tank_bottom) ? F("Under Water") : F("Dry"));
        Serial.print(s_star);  Serial.print(F("Top Tank Swimmer: "));
        Serial.println((sensor_values.tank_top) ? F("Under Water") : F("Dry"));
        Serial.print(s_star);  Serial.print(F("Battery Voltage: "));
        Serial.print(sensor_values.battery_voltage );
        Serial.print(F("V\r\n * Water flow clicks: "));
        Serial.println(sensor_values.water_flow_clicks);
        Serial.print(s_star);  Serial.print(s_star); Serial.print(F("Rain: "));
        Serial.println((sensor_values.rain_detected) ? F("Detected") : F("Somewhere else"));*/

      /*Serial.print(s_star);  Serial.print(F("WaterLow: "));
        Serial.println(sensor_values.low_water);
        Serial.print(s_star);  Serial.print(F("TankBot: "));
        Serial.println(sensor_values.tank_bottom);
        Serial.print(s_star);  Serial.print(F("TankTop: "));
        Serial.println(sensor_values.tank_top);
        Serial.print(s_star);  Serial.print(F("BatV: "));
        Serial.print(sensor_values.battery_voltage );
        Serial.print(F("V\r\n * FlowClicks: "));
        Serial.println(sensor_values.water_flow_clicks);*/

      /*Serial.print(s_star);
      Serial.print("Rain: ");
      Serial.println(sensor_values.rain_detected);*/
      //Serial.print(s_star);
      //Serial.print("RS: ");
      Serial.println(sensor_values.rain_start_millis);
      //Serial.print(s_star);
      //Serial.print("RE: ");
      Serial.println(sensor_values.rain_end_millis);

      /*Serial.println(F("System: "));

        // Serial.print(F("Reset Cause: "));
        Serial.print(F("Reset: "));

        if (mcusr_copy & (1 << WDRF)) {
        // Serial.print(F("WATCHDOG! Firmware may be unstable!"));
        Serial.print(F("WATCHDOG!"));
        }
        else if (mcusr_copy & (1 << BORF)) {
        // Serial.print(F("BROWNOUT! Check your Arduino's Power Suppy!"));
        Serial.print(F("BROWNOUT!"));
        }
        else if (mcusr_copy & (1 << EXTRF)) {
        // Serial.print(F("External. For example the Reset Button."));
        Serial.print(F("External"));
        }
        else if (mcusr_copy & (1 << PORF)) {
        Serial.print(F("Power on"));
        }
        Serial.println();*/

      /*Serial.print(s_star); Serial.print(F("Uptime: "));
        Serial.print(round(millis() / 1000));
        Serial.print('s');
        Serial.println();*/

      /*Serial.print(s_star); Serial.print(F("ADC div: "));
        Serial.println(settings.battery_voltage_adc_divider);

        Serial.print(s_star); Serial.print(F("Lora Missing: "));
        Serial.println(component_errors.lora_missing ? F("YES") : F("No"));

        Serial.print(s_star); Serial.print(F("LoRa Enable: "));
        Serial.println(settings.lora_enable);*/

      /*Serial.print(s_star); Serial.print(F("AirT: "));
        Serial.print(lora_airtime / 1000); Serial.print(F("s/")); Serial.print(LORA_MAX_AIRTIME); Serial.print(F("s ("));
        Serial.print(((float)lora_airtime / ((float)LORA_MAX_AIRTIME * 1000)) * 100); Serial.println(F("%)"));*/

      /*Serial.print(s_star);
      Serial.print(F("AirT: "));
      Serial.print(lora_airtime / 1000);
      Serial.write('/');
      Serial.print(LORA_MAX_AIRTIME);
      Serial.println((lora_airtime > LORA_MAX_AIRTIME * 1000L) ? F(" USED UP") : F(" OK"));

      Serial.println();*/
    }

    if (controlCharacter == 'G') {  //generate new lora key
      for (uint8_t b = 0; b < 16; b++) settings.lora_security_key[b] = random(0, 255);
      EEPROM.put(0, settings);

      //Serial.println(F("New Key"));
      controlCharacter = 'K';
    }

    if (controlCharacter == 'K') {  //generate new lora key
      //Serial.print(F("LoRa Key: "));
      array_hexprint(settings.lora_security_key, 16);
    }

    /*if (controlCharacter == 'P') {
      lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
      lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
      lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_TEST;
      lora_outgoing_queue[lora_outgoing_queue_idx][3] = 0xDE;
      lora_outgoing_queue[lora_outgoing_queue_idx][4] = 0xEA;
      lora_outgoing_queue[lora_outgoing_queue_idx][5] = 0xDB;
      lora_outgoing_queue[lora_outgoing_queue_idx][6] = 0xEE;
      lora_outgoing_queue[lora_outgoing_queue_idx][7] = 0xF1;

      afterpacket_stuff();
      }*/

    /*if (controlCharacter == 'R') { //reset all settings
      //todo: write this
      }*/

    if (controlCharacter == 'B') {  //reboot
      wdt_enable(WDTO_15MS);
      while (true)
        ;
      ;
    }

    if (controlCharacter == 'C') {  //resistor divider cal, enter as C13.20 for 13.20V or C05.00 for 5V
      // Serial.println("Calibrating Vbat ADC divider:");
      //Serial.println(F("Cal Vbat div:"));
      Serial.println(F("DIVn:"));
      uint32_t smoothed_adc_val = 0;
      for (uint8_t s = 0; s < 64; s++) {
        smoothed_adc_val += analogRead(BATTERY_VOLTAGE_PIN);
      }

      smoothed_adc_val /= 64;
      //noser Serial.print(s_star); Serial.print(F("ADC: "));
      //noser Serial.println(smoothed_adc_val);

      String volt_buf;
      for (uint8_t c = 0; c < 5; c++) {  //read part after C into mem
        while (!Serial.available())
          ;
        ;
        char read_char = Serial.read();

        if (read_char == '\n' or read_char == '\r') {  //if end of line
          while (Serial.available()) Serial.read();    //clear serial buf
          break;
        } else volt_buf += read_char;
      }
      float correct_volt = volt_buf.toFloat();  //convert it to float


      //noser Serial.print(s_star); //noser Serial.print(F("V entered: "));
      //noser Serial.println(correct_volt);

      settings.battery_voltage_adc_divider = smoothed_adc_val / correct_volt;
      EEPROM.put(0, settings);
    }
  }
  //digitalWrite(pcf, ACTIVITY_LED, HIGH);
}

void do_stored_buttons() {
  for (uint8_t q_pos = 0; q_pos <= min(31, button_queue_add_pos); q_pos++) {  //make sure they are 0
    switch (button_queue[q_pos]) {
      case 1:
        down_callback();
        break;
      case 2:
        up_callback();
        break;
      case 3:
        btn_callback();
        break;
      case 0:
      //return; //save time by stopping at first sight of 0, not needed due to only going up button_queue_add_pos
      default:
        break;
    }
    button_queue[q_pos] = 0;  //clear position so it wont get executed again
  }
  button_queue_add_pos = 0;
}

void send_ack(byte packet_id) {
  lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
  lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
  lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_WS_ACK;
  lora_outgoing_queue[lora_outgoing_queue_idx][3] = packet_id;

  //lora_outgoing_queue_last_tx = millis() - LORA_RETRANSMIT_TIME + 2500; //ack only sent 1000ms after
  lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] = LORA_RETRANSMIT_TRIES - 1;
  lora_outgoing_packet_id++;
  if (lora_outgoing_packet_id < 1) lora_outgoing_packet_id == 1;  //never let it go to 0, that causes bugs
  lora_outgoing_queue_idx++;
  if (lora_outgoing_queue_idx >= 4) lora_outgoing_queue_idx = 0;
}

void clear_packet(byte packet_id) {
  for (uint8_t p = 0; p < 4; p++) {
    if (packet_id == lora_outgoing_queue[p][1]) {
      //noser Serial.print(s_star); //noser Serial.print(F("Clearing Packet ID: "));
      //noser Serial.println(packet_id);
      for (uint8_t b = 0; b < 48; b++) lora_outgoing_queue[p][b] = 0;  //clear packet
      //lora_outgoing_queue_last_tx = 0;
      lora_outgoing_queue_tx_attempts[p] = LORA_RETRANSMIT_TRIES;
      //noser Serial.print(s_star); //noser Serial.print(F("Cleared Packet Slot: "));
      //noser Serial.println(p);
    }
  }
}

void afterpacket_stuff() {
  //lora_outgoing_queue_last_tx = 0;
  lora_outgoing_queue_tx_attempts[lora_outgoing_queue_idx] = 0;
  lora_outgoing_packet_id++;
  if (lora_outgoing_packet_id < 1) lora_outgoing_packet_id == 1;  //never let it go to 0, that causes bugs
  lora_outgoing_queue_idx++;
  if (lora_outgoing_queue_idx >= 4) lora_outgoing_queue_idx = 0;
}

bool check_lora_auth(uint8_t packet_num, uint8_t resp_offset /*TODO: get this from packet type length -32 -3*/, byte cmd_packet_id) {  //resp always 32bytes, no need for its length
  Serial.println(F("Auth:"));
  bool chal_valid = false;
  for (uint8_t b = 0; b < 16; b++)
    if (auth_challange[b] != 0) chal_valid = true;
  if (!chal_valid) {
    Serial.print(s_star);
    Serial.println(F("No Valid Chal"));

    lora_outgoing_queue[lora_outgoing_queue_idx][0] = 42;
    lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
    lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_NO_CHALLANGE;
    lora_outgoing_queue[lora_outgoing_queue_idx][3] = cmd_packet_id;  //include packet id to not need ACK
    afterpacket_stuff();

    return false;
  }

  byte* auth_response = &lora_incoming_queue[packet_num][resp_offset];

  //make hash
  Sha256 sha;
  byte* hash;
  sha.init();
  sha.write(auth_challange, 16);
  Serial.print(s_star);
  Serial.print(F("Chal: "));
  array_hexprint(auth_challange, 16);
  Serial.println();

  sha.write(settings.lora_security_key, 16);
  /*Serial.print(s_star);
  Serial.print(F("Key: "));
  array_hexprint(settings.lora_security_key,16);
  */

  sha.write(&lora_incoming_queue[packet_num][2], 1);                //write packet type
  sha.write(&lora_incoming_queue[packet_num][4], resp_offset - 4);  //from the 4th byte (3protocol and 1ack before) read for (distance from hash begin)
  Serial.print(s_star);
  Serial.print(F("Dat: "));
  array_hexprint(&lora_incoming_queue[packet_num][4], resp_offset - 4);
  Serial.println();

  hash = sha.result();
  Serial.print(s_star);
  Serial.print(F("Cor: "));
  for (uint8_t b = 0; b < 32; b++) {
    Serial.print(hash[b], HEX);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print(s_star);
  Serial.print(F("Rcv: "));
  for (uint8_t b = 0; b < 32; b++) {
    Serial.print(auth_response[b], HEX);
    Serial.print(' ');
  }
  Serial.println();

  //compare hash
  bool are_same = true;
  for (uint8_t b = 0; b < 32; b++)
    if (auth_response[b] != hash[b]) are_same = false;  //i should just use memcmp() for this why did i not think of that?!
  Serial.print(s_star);

  if (are_same) {
    Serial.println(F("AUTH OK"));

    lora_outgoing_queue[lora_outgoing_queue_idx][0] = 42;
    lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
    lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_CMD_OK;
    lora_outgoing_queue[lora_outgoing_queue_idx][3] = cmd_packet_id;  //include packet id to not need ACK
    afterpacket_stuff();
  } else {
    Serial.println(F("UNAUTHED"));

    lora_outgoing_queue[lora_outgoing_queue_idx][0] = 42;
    lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
    lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_CMD_AUTH_FAIL;
    lora_outgoing_queue[lora_outgoing_queue_idx][3] = cmd_packet_id;  //include packet id to not need ACK
    afterpacket_stuff();
  }

  for (uint8_t b = 0; b < 16; b++) auth_challange[b] = 0;  //clear chal
  return are_same;
}

void handle_lora() {
  if (component_errors.lora_missing or settings.lora_enable == 0) return;  // if there is no lora, dont even bother

  //recieve
  if (settings.lora_enable >= 2) {
    auto possible_packet_size = LoRa.parsePacket();  //the onRecieve() callback seems to just FUCKING CRASH sometimes
    if (possible_packet_size > 0) {
      //noser Serial.println(F("Possible packet incoming."));
      handle_lora_packet(possible_packet_size);
    }

    for (uint8_t p_idx = 0; p_idx < 4; p_idx++) {
      bool is_empty = true;
      for (uint8_t i = 0; i < 48; i++)
        if (lora_incoming_queue[p_idx][i] != 0) {
          is_empty = false;  //check for data in packet
          break;
        }
      if (!is_empty) {
        //digitalWrite(pcf, ACTIVITY_LED, LOW);
        //Serial.println(F("Incoming LoRa Packet:"));
        Serial.println(F("Pkt RX:"));
        Serial.print(s_star);
        Serial.print(F("L: "));  //Serial.print(F("Length: "));
        Serial.println(lora_incoming_queue_len[p_idx]);
        Serial.print(s_star);
        Serial.print(F("C: "));  //Serial.print(F("Content: "));
        array_hexprint(lora_incoming_queue[p_idx], min(lora_incoming_queue_len[p_idx], 48));
        Serial.println();

        if (lora_incoming_queue[p_idx][0] == LORA_MAGIC) {  //if magic correct
          bool already_recieved = false;
          for (uint8_t i = 0; i < 16; i++)
            if (lora_incoming_queue[p_idx][1] == lora_last_incoming_message_IDs[i]) already_recieved = true;

          lora_outgoing_queue_last_tx = 0;  //after getting a packet, respond immediatly

          bool do_ack = true;
          if (!already_recieved) {
            bool dedup_this = true;
            Serial.print(s_star);
            Serial.print(F("T: "));  //Serial.print(F("Magic Correct.\r\n * Packet type: "));
            switch (lora_incoming_queue[p_idx][2]) {
              case PACKET_TYPE_ACK:
                {
                  //Serial.println(F("ACK"));
                  clear_packet(lora_incoming_queue[p_idx][3]);
                  dedup_this = false;
                  do_ack = false;
                }
                break;

              case PACKET_TYPE_REQUST_CHALLANGE:
                {
                  //Serial.println(F("Chal. Req."));
                  do_ack = false;

                  for (uint8_t b = 0; b < 16; b++) auth_challange[b] = random(0, 255);  //make new challange

                  lora_outgoing_queue[lora_outgoing_queue_idx][0] = 42;
                  lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
                  lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_AUTH_CHALLANGE;
                  lora_outgoing_queue[lora_outgoing_queue_idx][3] = lora_incoming_queue[p_idx][1];  //include packet id to not need ACK
                  memcpy(&lora_outgoing_queue[lora_outgoing_queue_idx][4], &auth_challange, 16);

                  /*Serial.print(F("TX Chal: ")); //Serial.print(F("Sending Challange: "));
                    for (uint8_t b = 0; b < 16; b++)  Serial.print(auth_challange[b], HEX);*/
                  Serial.println();

                  afterpacket_stuff();
                }
                break;

              case PACKET_TYPE_ADD_WATER:
                {
                  //Serial.println(F("Water Call"));

                  clear_packet(lora_incoming_queue[p_idx][3]);
                  do_ack = false;

                  if (!check_lora_auth(p_idx, 6, lora_incoming_queue[p_idx][1])) break;

                  union {
                    uint16_t liters = 0;
                    byte liters_b[2];
                  };

                  liters_b[0] = lora_incoming_queue[p_idx][4];
                  liters_b[1] = lora_incoming_queue[p_idx][5];

                  tank_fillings_remaining = (settings.tank_capacity > 0) ? literToTanks(liters) : liters;
                }
                break;

              case PACKET_TYPE_CANCEL_WATER:
                {
                  //Serial.println(F("Irr. Cancel"));

                  clear_packet(lora_incoming_queue[p_idx][3]);
                  do_ack = false;

                  if (!check_lora_auth(p_idx, 4, lora_incoming_queue[p_idx][1])) break;

                  tank_fillings_remaining = 0;
                  system_state = STATUS_IDLE;
                }
                break;

              case PACKET_TYPE_SET_TIMER:
                {
                  //Serial.println(F("Set Timer:"));

                  if (!check_lora_auth(p_idx, 8, lora_incoming_queue[p_idx][1])) break;

                  union {
                    uint16_t liters = 0;
                    byte liters_b[2];
                  };
                  liters_b[0] = lora_incoming_queue[p_idx][6];
                  liters_b[1] = lora_incoming_queue[p_idx][7];

                  irrigation_timer.liters_to_pump = liters;
                  irrigation_timer.fillings_to_irrigate = literToTanks(liters);

                  irrigation_timer.start_hour = lora_incoming_queue[p_idx][4];
                  irrigation_timer.start_minute = lora_incoming_queue[p_idx][5];

                  EEPROM.put(0 + sizeof(settings), irrigation_timer);

                  clear_packet(lora_incoming_queue[p_idx][3]);
                  do_ack = false;

                  /*Serial.print(s_star);
                  Serial.print(F("H: "));
                  Serial.println(irrigation_timer.start_hour);
                  Serial.print(s_star);
                  Serial.print(F("M: "));
                  Serial.println(irrigation_timer.start_minute);
                  Serial.print(s_star);
                  Serial.print(F("L: "));
                  Serial.println(irrigation_timer.liters_to_pump);
                  Serial.print(s_star);
                  Serial.print(F("T: "));
                  Serial.println(irrigation_timer.fillings_to_irrigate);
                  Serial.println();*/
                }
                break;

              case PACKET_TYPE_GW_REBOOT:
                {
                  //Serial.println(F("GW Reboot"));
                  for (uint8_t i = 0; i < 16; i++) lora_last_incoming_message_IDs[i] = 0;  //counter on other side reset, so we reset too
                }
                break;


              default: break;
            }

            if (dedup_this) {
              lora_last_incoming_message_IDs[lora_last_incoming_message_IDs_idx] = lora_incoming_queue[p_idx][1];
              lora_last_incoming_message_IDs_idx++;
              if (lora_last_incoming_message_IDs_idx >= 16) lora_last_incoming_message_IDs_idx = 0;
            }
          }
          //noser else Serial.println(F("Packet already recieved."));
          if (do_ack) send_ack(lora_incoming_queue[p_idx][1]);
        }
        for (uint8_t i = 0; i < 48; i++) lora_incoming_queue[p_idx][i] = 0;  //clear after processing

        Serial.println();
        //digitalWrite(pcf, ACTIVITY_LED, HIGH);
      }
    }
    digitalWrite(pcf, LORA_RX_LED, HIGH);
  }

  //airtime reset
  static uint8_t last_airtime_rest_hour = 0;
  static uint32_t last_airtime_rest_millis = 0;
  if (component_errors.rtc_missing or component_errors.rtc_unset) {     //if RTC is missing
    if (millis() - last_airtime_rest_millis > (60 * 60 * 2 * 1000L)) {  //reset airtime every 2 hours by millis
      lora_airtime = 0;
      last_airtime_rest_millis = millis();
    }
  } else if (last_airtime_rest_hour != current_time.Hour and millis() - last_airtime_rest_millis > (60 * 59 * 1000L)) {  //else reset on xx:00 time unless less than 59 minutes passed since last reset
    last_airtime_rest_hour = current_time.Hour;
    lora_airtime = 0;
    last_airtime_rest_millis = millis();
  }

  //queue handle
  if ((lora_airtime < LORA_MAX_AIRTIME * 1000L) and lora_tx_ready and millis() - lora_outgoing_queue_last_tx > LORA_RETRANSMIT_TIME) {
    wdt_reset();
    for (uint8_t p_idx = 0; p_idx < 4; p_idx++) {
      bool is_empty = true;
      for (uint8_t i = 0; i < 48; i++)
        if (lora_outgoing_queue[p_idx][i] != 0) {
          is_empty = false;  //check for data in packet
          break;
        }

      if (!is_empty and millis() - lora_outgoing_queue_last_tx > LORA_RETRANSMIT_TIME and lora_tx_ready) {
        //digitalWrite(pcf, ACTIVITY_LED, LOW);
        digitalWrite(pcf, LORA_TX_LED, LOW);
        //Serial.println(F("Sending LoRa Packet: "));
        Serial.println(F("TX Pkt: "));

        lora_tx_ready = false;
        LoRa.idle();  //no recieving while transmitting!
        LoRa.beginPacket();

        uint8_t lora_bytes = ws_to_gw_packet_type_to_length(lora_outgoing_queue[p_idx][2]) + 3;  // check 2nd byte (packet type), get data length and add 3 for magic + packet id+ packett type
        Serial.print(s_star);
        Serial.print(F("L: "));  //Serial.print(F("Length: "));
        Serial.println(lora_bytes);

        Serial.print(s_star);
        Serial.print(F("C: "));  //Serial.print(F("Content: "));
        for (uint8_t b = 0; b < lora_bytes; b++) {
          LoRa.write(lora_outgoing_queue[p_idx][b]);
          if (lora_outgoing_queue[p_idx][b] < 0x10) Serial.write('0');
          Serial.print(lora_outgoing_queue[p_idx][b], HEX);
          Serial.write(' ');
        }
        wdt_reset();
        lora_tx_start_millis = millis();
        LoRa.endPacket(/*true*/ false);  //tx in not async mode becaus that never seems to work
        handle_lora_tx_done();
        Serial.println();
        //noser Serial.print(F("Sent"));

        lora_outgoing_queue_last_tx = millis();
        lora_outgoing_queue_tx_attempts[p_idx]++;
        if (lora_outgoing_queue_tx_attempts[p_idx] >= LORA_RETRANSMIT_TRIES) {
          for (uint8_t i = 0; i < 48; i++) lora_outgoing_queue[p_idx][i] = 0;  //clear packet if unsuccessful
          //lora_outgoing_queue_last_tx = 0;
          lora_outgoing_queue_tx_attempts[p_idx] = 0;
        }
        //noser Serial.println(F(" and Done."));
        Serial.println();
        //digitalWrite(pcf, ACTIVITY_LED, HIGH);
      }
    }
  }

  //tx making
  if (settings.lora_enable >= 1) {
    static uint32_t last_lora_tx = 0;

    //make status byte ==========================
    byte current_status_byte = 0;
    current_status_byte = uint8_t(min(0xF, system_state));  //fill 4 right bits with system state 0000SSSS
    switch (system_state) {                                 //add extra status SSSSEEEE
      case STATUS_IDLE:
        current_status_byte <<= 4;  //shift bits over SSSS0000
        if (settings.tank_capacity > 0 ? irrigation_timer.fillings_to_irrigate == 0 : irrigation_timer.liters_to_pump == 0) current_status_byte |= 0x01;
        else if (irrigation_timer.last_watering_day == current_time.Day) current_status_byte |= 0x02;
        else if (sensor_values.rain_detected) current_status_byte |= 0x03;
        else current_status_byte |= 0x00;
        break;

      case STATUS_GENERAL_FAIL:
        //shift each error in
        current_status_byte <<= 1;
        current_status_byte |= uint8_t(component_errors.tank_sensors_irrational);
        current_status_byte <<= 1;
        current_status_byte |= uint8_t(component_errors.rtc_missing);
        current_status_byte <<= 1;
        current_status_byte |= uint8_t(component_errors.rtc_unset);
        current_status_byte <<= 1;
        current_status_byte |= uint8_t(component_errors.pump_timed_out);
        break;

      default:
        current_status_byte <<= 4;  //shift bits over SSSS0000
        break;
    }
    //status_byte end =================================


    static byte last_system_states_arr[8] = { 255, 255, 255, 255, 255, 255, 255, 255 };  //just in case my stuff bounces between states a few times
    static uint8_t lss_idx = 0;
    last_system_states_arr[lss_idx] = current_status_byte;
    lss_idx++;
    if (lss_idx >= 8) lss_idx = 0;
    bool state_stable = true;
    for (uint8_t s = 1; s < 8; s++)
      if (last_system_states_arr[s] != last_system_states_arr[s - 1]) {
        state_stable = false;  //if states 2-8 not stable, wait
        // Serial.println(F("State not Stable"));
        // Serial.println(current_status_byte, HEX);
        break;
      }

    //FIXME: for some reason sometimes changes dont trigger tx
    static byte last_system_state = 0xFF;
    if (((last_system_state != current_status_byte) or (millis() - last_lora_tx > LORA_TX_INTERVAL)) and state_stable) {  //if there was a change or the timer ran out and the state is stable
      last_system_state = current_status_byte;
      //Serial.println(F("Made new status to be sent"));
      //if new state, make lora packet
      lora_outgoing_queue[lora_outgoing_queue_idx][0] = LORA_MAGIC;
      lora_outgoing_queue[lora_outgoing_queue_idx][1] = lora_outgoing_packet_id;
      lora_outgoing_queue[lora_outgoing_queue_idx][3] = current_status_byte;

      if (system_state != STATUS_PUMPING and system_state != STATUS_EMPTYING) {  //if not watering
        //[status byte][battery voltage upper][battery voltage lower]
        lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_STATUS;

        union {
          uint16_t bat_v;
          byte bat_b[2];
        };
        bat_v = float(sensor_values.battery_voltage * 100);

        lora_outgoing_queue[lora_outgoing_queue_idx][4] = bat_b[0];  //lower half of uint16
        lora_outgoing_queue[lora_outgoing_queue_idx][5] = bat_b[1];  //upper half of uint16
      } else {                                                       //if watering
        //[status byte][left upper][left lower][called upper][called lower]
        lora_outgoing_queue[lora_outgoing_queue_idx][2] = PACKET_TYPE_WATER;

        lora_outgoing_queue[lora_outgoing_queue_idx][3] = current_status_byte;

        union {
          uint16_t liters_left_int;
          byte liters_left_byte[2];
        };
        liters_left_int = settings.tank_capacity > 0 ? ((tank_fillings_remaining + 1) * settings.tank_capacity) : tank_fillings_remaining;
        lora_outgoing_queue[lora_outgoing_queue_idx][4] = liters_left_byte[0];  //lower half of uint16
        lora_outgoing_queue[lora_outgoing_queue_idx][5] = liters_left_byte[1];  //upper half of uint16

        union {
          uint16_t liters_called_int;
          byte liters_called_byte[2];
        };
        liters_called_int = settings.tank_capacity > 0 ? (irrigation_timer.fillings_to_irrigate * settings.tank_capacity) : irrigation_timer.liters_to_pump;
        lora_outgoing_queue[lora_outgoing_queue_idx][6] = liters_called_byte[0];  //lower half of uint16
        lora_outgoing_queue[lora_outgoing_queue_idx][7] = liters_called_byte[1];  //upper half of uint16
      }

      last_lora_tx = millis();
      afterpacket_stuff();
    }
  }
}

void loop() {
  // Serial.println('S');
  read_sensors_and_clock();
  // Serial.println('P');
  handle_pump_stuff();
  // Serial.println('D');
  update_display();

  // Serial.println('U');
  handle_serial();
  // Serial.println('B');
  do_stored_buttons();
  // Serial.println('L');
  handle_lora();
  // Serial.println('-');
  // Serial.println();

  wdt_reset();
  delay(10);
}
