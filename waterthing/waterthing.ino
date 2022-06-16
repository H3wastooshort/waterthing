//Bewässerungssystem
/*
 * Pin 7 -> Wassermangel Sensor / Fass Schwimmer
 * Pin 8 -> Tank Leer Schwimmer
 * Pin 9 -> Tank Voll Schwimmer
 * Pin 10 -> Relais für Pumpe + Magnetventil vor Tank
 * Pin 11 -> Relais für Magnetventil nach Tank
*/
/*
Zyklus:
 * Pumpe An
 * Warten Bis Tank Voll
 * Pumpe Aus
 * Ventil Auf
 * Warten bis Tank leer
 * wiederholen bis gewünschte menge bewässert
*/
/*
========
Achtumg/Attention
========
Die Standard sprintf() funktion von Arduino kann kein float. Diese compiler flags müssen hinzugefügt werden
The default sprintf() fuction in Arduino cant do floats. You have to add these compiler flags

ArduinoIDE either platform.txt:
compiler.c.elf.extra_flags=-Wl,-u,vfprintf -lprintf_flt -lm

or (recommended) boards.txt:
uno.build.extra_flags=-Wl,-u,vfprintf -lprintf_flt -lm

PlatformIO platformio.ini:
build_flags = -Wl,-u,vfprintf -lprintf_flt -lm
*/

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <math.h>

#define LOW_WATER_PIN 7
#define TANK_BOTTOM_PIN 8
#define TANK_TOP_PIN 9
#define PUMP_PIN 10
#define VALVE_PIN 11
#define LOW_WATER_ON_LEVEL LOW //level of low water pin when the low water sensor detects low water. sensor will probably be mounted upside-down so no water means low
#define TANK_BOTTOM_ON_LEVEL HIGH //level of TANK_BOTTOM_PIN when water has reached the bottom sensor. sensor will probably be mounted upside-down so on means high
#define TANK_TOP_ON_LEVEL LOW //level of TANK_TOP_PIN when water has reached the top sensor

#define RED_LED_PIN A1
#define GREEN_LED_PIN A2
#define BLUE_LED_PIN A3

#define DEBOUNCE_DELAY 100
#define BTN_PIN 2
#define ENCODER_CLK_PIN 3
#define ENCODER_DT_PIN 4
#define ENCODER_INVERT_DIRECTION false

#define BATTERY_VOLTAGE_PIN A0
#define BATTERY_VOLTAGE_ADC_DIVIDER 68.267 //1024 divided by this multiplier equals the battery voltage

#define EEPROM_MAGIC_NUMBER 42

//library stuff
LiquidCrystal_I2C lcd(0x27,16,2);

//dynamic status
const byte gfx_off[8] = {B01110, B01010, B01110, B00000, B11011, B10010, B11011, B10010};
//const byte gfx_idle[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B10101, B00000}; //... version
const byte gfx_idle[8] = {B00100, B00100, B01110, B10111, B11111, B01110, B00000, B10101}; //drop with ... version
//const byte gfx_idle[8] = {B00110, B00011, B01100, B00110, B11000, B01000, B10000, B11000}; //zZZ version
const byte gfx_fill[8] = {B00100, B10101, B01110, B00100, B00000, B10001, B10001, B11111};
const byte gfx_drain[8] = {B10001, B10001, B11111, B00000, B00100, B10101, B01110, B00100};
const byte gfx_no_water[8] = {B00001, B00001, B01000, B01101, B01100, B10110, B11110, B01100};
const byte gfx_low_bat[8] = {B01110, B10001, B10101, B10101, B10001, B10101, B10001, B11111};
const byte gfx_error[8] = {B10101, B10101, B10101, B10101, B00000, B00000, B10101, B00000};

//symbol graphics
const byte gfx_hyst[8] = {B00111, B01010, B01010, B01010, B01010, B01010, B01010, B11100};
const byte gfx_drop[8] = {B00100, B00100, B01110, B01110, B10111, B11111, B01110, B00000};

//umlaut graphics
const byte gfx_uml_s[8] = {B01100, B10010, B10010, B11100, B10010, B10010, B10100, B10000};
const byte gfx_uml_a[8] = {B01010, B00000, B01110, B00001, B01111, B10001, B01111, B00000};
const byte gfx_uml_o[8] = {B01010, B00000, B01110, B10001, B10001, B10001, B01110, B00000};
const byte gfx_uml_u[8] = {B01010, B00000, B10001, B10001, B10001, B10011, B01101, B00000};

enum gfx_IDs { //enum for naming display gfx ids
  /*GFX_ID_IDLE = 0,
  GFX_ID_NO_WATER = 1, //GFX_ID_DROP = 1,
  GFX_ID_FILL = 2,
  GFX_ID_DRAIN = 3,*/
  GFX_ID_STATUS = 0, //redefined dynamically
  GFX_ID_DROP = 1,
  GFX_ID_HYST = 2,
  GFX_ID_UML_S = 4,
  GFX_ID_UML_A = 5,
  GFX_ID_UML_O = 6,
  GFX_ID_UML_U = 7,
  GFX_ID_ARROW_R = 127,
  GFX_ID_ARROW_L = 126
};

//structs are used to save to EEPROM more easily
struct settings_s {
  uint8_t tank_capacity = 10; //in L
  float battery_voltage_cutoff = 10.4; //if this voltage is reached while the system is idle, a low battery waring is displayed. 0 means disable
  float battery_voltage_reset = 11.4; //if this voltage is reached while the system is in low batters state, the system is set back to idle
  int16_t max_mins_per_refill = -1; //error is thrown if the pump takes longer than that to fill the tank (5L/min pump wont take more than 5 minutes to fill a 20L tank)
} settings;

struct i_timer_s {
  int8_t start_hour = 0;
  int8_t start_minute = 0;
  uint16_t fillings_to_irrigate = 0; //when 0, system is turned off
  uint8_t last_watering_day = 0;
} irrigation_timer;

//and also just to organize things
struct c_error_s{
  bool rtc_missing = false;
  bool rtc_unset = false;
  bool tank_sensors_irrational = false;
} component_errors;

//enums for easier code reading
enum system_state_e { 
  STATUS_IDLE, //doing nothing
  STATUS_EMPTYING, //emptying tank
  STATUS_PUMPING, //pumping water in tank
  STATUS_NO_WATER, //cant get water from tank
  STATUS_NO_TIME, //clock not set
  STATUS_LOW_BATTERY, //low battery voltage

  STATUS_GENERAL_FAIL
};

//global system variables
enum system_state_e system_state = STATUS_IDLE;
tmElements_t current_time;
float battery_voltage = 13.8;

struct raw_sens_s {
  bool low_water = !LOW_WATER_ON_LEVEL;
  bool tank_bottom = !TANK_BOTTOM_ON_LEVEL;
  bool tank_top = !TANK_TOP_ON_LEVEL;
} raw_sensors;

uint16_t tank_fillings_remaining = 0; //if over 0, run pumping stuff til 0

//global menu variables
int8_t menu_page = 0; // 0-> main page, 1 -> Timer Setup, 2 -> Tank Setup, 3 -> Clock Setup
int8_t menu_entry_cursor = 0; //-2 -> not in menu system, 0 -> page selection, 1-255 differs per page
bool menu_editing = false;
uint32_t last_display_update = 0; //this is a global var to allow calling for immediate update by setting this to 0
uint8_t button_queue[32]; //0 means no btn, 1 means down, 2 means up, 3 means enter
uint8_t button_queue_add_pos = 0; //0-31

//menu vars for each page
//1 manuell
uint16_t page_1_irrigate_order = 0;

//pages
enum pages_enum {
  PAGE_STATUS = 0,
  PAGE_MAN = 1,
  PAGE_TIMER = 2,
  PAGE_TANK = 3,
  PAGE_CLOCK1 = 4,
  PAGE_CLOCK2 = 5,
  PAGE_BATTERY = 6
};

#define N_OF_PAGES 7
const char* page_names[N_OF_PAGES] = {"Status", "Manuell", "Timer", "Tank", "Uhr1", "Uhr2", "Akku"};
const uint8_t page_max_cursor[N_OF_PAGES] = {0, 1, 3, 1, 2, 3, 2};


int16_t literToTanks(uint16_t liters_to_convert) {
  if (liters_to_convert % settings.tank_capacity == 0) {
    return liters_to_convert / settings.tank_capacity;
  }
  else {
    return ((float)liters_to_convert / (float)settings.tank_capacity)+1;
  }
}

void lcd_print_menu_bracket(uint8_t for_menu_entry, bool ending_bracket) {
  if (ending_bracket) lcd.print(menu_entry_cursor == for_menu_entry ? (menu_editing ? F("}") : F("]")) : F(")"));
  else lcd.print(menu_entry_cursor == for_menu_entry ? (menu_editing ? F("{") : F("[")) : F("("));
}

void change_menu_entry(bool dir) { //true is up
  if (menu_entry_cursor == 0) {
      menu_page += dir ? 1 : -1;
      if (menu_page >= N_OF_PAGES) menu_page = N_OF_PAGES-1;
      if (menu_page < 0) menu_page = 0;
    return;
  }
  switch (menu_page) {
    case PAGE_MAN: //manual
      if (menu_entry_cursor == 1) {
        if (tank_fillings_remaining == 0) {
          page_1_irrigate_order += dir ? settings.tank_capacity : settings.tank_capacity*-1;
          if (page_1_irrigate_order > 990) page_1_irrigate_order = 990;
          if (page_1_irrigate_order < 0) page_1_irrigate_order = 0;
        }
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
          irrigation_timer.fillings_to_irrigate += dir ? 1 : -1;
          if (irrigation_timer.fillings_to_irrigate >= literToTanks(900)) irrigation_timer.fillings_to_irrigate = literToTanks(900);
          if (irrigation_timer.fillings_to_irrigate < 0 or irrigation_timer.fillings_to_irrigate == 0xFFFF) irrigation_timer.fillings_to_irrigate = 0;
          break;

        default:
          break;
      }
      break;

    case PAGE_TANK:
      settings.tank_capacity += dir ? 1 : -1;
      if (settings.tank_capacity >= 200) settings.tank_capacity = 200;
      if (settings.tank_capacity < 0) settings.tank_capacity = 0;
      break;

    case PAGE_CLOCK1:
      switch (menu_entry_cursor) {
        case 1:
          current_time.Hour += dir ? 1 : -1;
          if (current_time.Hour >= 23) current_time.Hour = 23;
          if (current_time.Hour < 0 or current_time.Hour > 25) current_time.Hour = 0;
          break;
        case 2:
          current_time.Minute += dir ? 1 : -1;
          if (current_time.Minute >= 59) current_time.Minute = 59;
          if (current_time.Minute < 0 or current_time.Minute > 61) current_time.Minute = 0;
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
          if (settings.battery_voltage_cutoff >= 48) settings.battery_voltage_cutoff = ceil(1023 / BATTERY_VOLTAGE_ADC_DIVIDER);
          if (settings.battery_voltage_cutoff < 0) settings.battery_voltage_cutoff = 0;
          break;
        case 2:
          settings.battery_voltage_reset += dir ? 0.1 : -0.1;
          if (settings.battery_voltage_reset >= 48) settings.battery_voltage_reset = ceil(1023 / BATTERY_VOLTAGE_ADC_DIVIDER);
          if (settings.battery_voltage_reset < 0) settings.battery_voltage_reset = 0;
          break;
          break;
      }
      break;

    default:
    case -1:
      break;
  }
}

void edit_change_callback() {
  switch (menu_page) {
    case PAGE_MAN:
      if (menu_entry_cursor == 1) {
        if (tank_fillings_remaining == 0) {
          if (!menu_editing) { //if leaving edit mode
            tank_fillings_remaining = literToTanks(page_1_irrigate_order);
            Serial.print(F("Manual water call for "));
            Serial.print(page_1_irrigate_order);
            Serial.print(F("L wich makes "));
            Serial.print(tank_fillings_remaining);
            Serial.println(F(" tank fillings"));
          }
        }
        else {
          system_state = STATUS_IDLE;
          tank_fillings_remaining = 0;
          menu_editing = false;
          Serial.print(F("Watering canceled"));
        }
      }
      break;
    case PAGE_TIMER:
      if (!menu_editing) { //if leaving edit mode
        if (menu_entry_cursor > 1) {
          irrigation_timer.last_watering_day = 0;
          EEPROM.put(0+sizeof(settings), irrigation_timer); //save timer settings when leaving
        }
      }
      break;
    case PAGE_TANK:
    case PAGE_BATTERY:
      if (!menu_editing) { //if leaving edit mode
        if (menu_entry_cursor > 1) {
          EEPROM.put(0, settings); //save settings when leaving
        }
      }
      break;
    case PAGE_CLOCK1:
      break;
    case PAGE_CLOCK2:
      break;


    default:
    case -1:
      break;
  }
}

void up_callback() {
  if (menu_editing) {
    change_menu_entry(true);
  }
  else {
    menu_entry_cursor++;
    if (menu_entry_cursor > page_max_cursor[menu_page]) menu_entry_cursor = 0;
  }
  last_display_update = 0;
}

void down_callback() {
  if (menu_editing) {
    change_menu_entry(false);
  }
  else {
    menu_entry_cursor--;
    if (menu_entry_cursor < 0) menu_entry_cursor = 0;
  }
  last_display_update = 0;
}

void btn_callback() {
  menu_editing = !menu_editing;
  edit_change_callback();
  last_display_update = 0;
}

uint32_t last_btn_down = 0;
void handle_btn_up() {
  if (millis() - last_btn_down >= DEBOUNCE_DELAY) {
    if (button_queue_add_pos >= 32) return; //too many buttons in queue, ignoring
    button_queue[button_queue_add_pos] = 3; //3 is for enter button
    button_queue_add_pos++;
  }
  last_btn_down = millis();
}

uint32_t last_encoder_clock = 0;
void handle_encoder_clk() {
  bool immidiate_dt = digitalRead(ENCODER_DT_PIN); //read dt as quickly as possible

  if (button_queue_add_pos >= 32) return; //too many buttons in queue, ignoring
  if (millis() - last_encoder_clock <= DEBOUNCE_DELAY) return; //to quick, may be a bounce
  if (immidiate_dt != ENCODER_INVERT_DIRECTION) {
    button_queue[button_queue_add_pos] = 2; //2 is for up
  }
  else {
    button_queue[button_queue_add_pos] = 1; //1 is for down
  }
  button_queue_add_pos++;
  last_encoder_clock = millis();
}


void set_status_led(bool r, bool g, bool b) {
  digitalWrite(RED_LED_PIN,r);
  digitalWrite(GREEN_LED_PIN,g);
  digitalWrite(BLUE_LED_PIN,b);
}

void setup() {
  //pump stuff
  pinMode(LOW_WATER_PIN, INPUT_PULLUP);
  pinMode(TANK_BOTTOM_PIN, INPUT_PULLUP);
  pinMode(TANK_TOP_PIN, INPUT_PULLUP);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);

  //vbat
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  analogReference(INTERNAL/*1V1*/); //set ADC voltage reference to stable internal 1.1V reference. uncomment the 1V1 part for arduino megas

  //rgb led
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);

  //buttons
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  pinMode(BTN_PIN, INPUT_PULLUP);

  set_status_led(1,1,0);

  Serial.begin(9600);
  Serial.println(F("H3 Bewässerungssystem\nhttps://blog.hacker3000.cf/waterthing.php"));

  Serial.println(F("LCD setup..."));
  Wire.setClock(400000); //faster drawing
  lcd.init();
  lcd.backlight();
  /*lcd.createChar(GFX_ID_IDLE, gfx_idle);
  //lcd.createChar(GFX_ID_DROP, gfx_drop);
  lcd.createChar(GFX_ID_NO_WATER, gfx_no_water);
  lcd.createChar(GFX_ID_FILL, gfx_fill);
  lcd.createChar(GFX_ID_DRAIN, gfx_drain);*/
  lcd.createChar(GFX_ID_STATUS, gfx_error);
  lcd.createChar(GFX_ID_DROP, gfx_drop);
  lcd.createChar(GFX_ID_HYST, gfx_hyst);
  lcd.createChar(GFX_ID_UML_S, gfx_uml_s);
  lcd.createChar(GFX_ID_UML_A, gfx_uml_a);
  lcd.createChar(GFX_ID_UML_O, gfx_uml_o);
  lcd.createChar(GFX_ID_UML_U, gfx_uml_u);
  delay(50);
  lcd.clear();
  lcd.home();
  lcd.print(F("Bew\x05sserungs"));
  lcd.setCursor(0,1);
  lcd.print(F("System by H3"));
  delay(1000);
  lcd.clear();
  lcd.home();
  lcd.print(F("Blog-Artikel/Doku"));
  lcd.setCursor(0,1);
  lcd.print(F("-> hacker3000.cf"));
  delay(1000);

  //EEPROM
  Serial.println(F("Reading EEPROM..."));
  lcd.clear();
  lcd.home();
  lcd.print(F("Reading EEPROM..."));
  lcd.setCursor(0, 1);
  if (EEPROM.read(EEPROM.length() - 1) == EEPROM_MAGIC_NUMBER) {
    EEPROM.get(0,settings);
    EEPROM.get(0+sizeof(settings), irrigation_timer);
    //EEPROM.get(0+sizeof(settings)+sizeof(irrigation_timer), something_else);
    lcd.print(F("OK"));
  }
  else {
    EEPROM.put(0,settings);
    EEPROM.put(0+sizeof(settings), irrigation_timer);
    //EEPROM.put(0+sizeof(settings)+sizeof(irrigation_timer), something_else);
    EEPROM.write(EEPROM.length() - 1, EEPROM_MAGIC_NUMBER);
    lcd.print(F("Initialized"));
    delay(900);
  }
  delay(100);

  //rtc setup
  Serial.println(F("Setting up RTC..."));
  lcd.clear();
  lcd.home();
  lcd.print(F("RTC Setup..."));
  lcd.setCursor(0, 1);
  if (RTC.read(current_time)) { //if valid date read
    lcd.print(F("OK"));
  }
  else {
    if (RTC.chipPresent()) { //if date invalid but RTC present
      Serial.println(F("RTC NOT SET"));
      lcd.print(F("RTC NOT SET"));
      delay(1000);
    }
    else { //if RTC missing
      Serial.println(F("RTC MISSING!"));
      lcd.print(F("RTC MISSING"));
      while (true) {
        set_status_led(1,0,0);
        delay(100);
        set_status_led(0,0,0);
        delay(100);
      }
    }
  }
  delay(100);

  for (uint8_t q_pos = 0; q_pos < 32; q_pos++) { //make sure they are 0
    button_queue[q_pos] = 0;
  }
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), handle_btn_up, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), handle_encoder_clk, FALLING);

  Serial.println();
  set_status_led(0,0,0);
}

void print_page_basics() {
  switch (system_state) {
    case STATUS_IDLE:
      
    if (irrigation_timer.fillings_to_irrigate == 0) lcd.createChar(GFX_ID_STATUS, gfx_off);
      else lcd.createChar(GFX_ID_STATUS, gfx_idle);
      break;
    case STATUS_PUMPING:
      lcd.createChar(GFX_ID_STATUS, gfx_fill);
      break;
    case STATUS_EMPTYING:
      lcd.createChar(GFX_ID_STATUS, gfx_drain);
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
  lcd_print_menu_bracket(0,false);
  lcd.print(menu_page);
  lcd_print_menu_bracket(0,true);
  lcd.print(F(""));
  lcd.print(page_names[menu_page]);
  lcd.setCursor(10,0);

  lcd.write(byte(GFX_ID_STATUS));
  char clock_buf[5];
  sprintf(clock_buf, "%02u:%02u", current_time.Hour, current_time.Minute);
  lcd.print(clock_buf);
  lcd.setCursor(0,1);
}

void update_display() {
  if (millis() - last_display_update > 1000) {
      //uint32_t display_draw_start_time = millis();
      print_page_basics();
      switch (menu_page) {
        default:
          Serial.println(F("Unknown Menu Entry Selected!"));
          lcd.print(F("PAGE UNKNOWN!!"));
          //menu_page = 0;
          break;
        case PAGE_STATUS:
            switch (system_state) {
              case STATUS_IDLE:
                if (irrigation_timer.fillings_to_irrigate == 0) {
                  lcd.print(F("Ausgeschaltet"));
                }
                else if (irrigation_timer.last_watering_day == current_time.Day) {
                  lcd.print(F("Fertig f\x07r Heute"));
                }
                else {
                  lcd.print(F("Stby. bis"));
                  lcd.setCursor(11,1);
                  char til_buf[5];
                  sprintf(til_buf, "%02u:%02u", irrigation_timer.start_hour, irrigation_timer.start_minute);
                  lcd.print(til_buf);
                }
                break;
              case STATUS_EMPTYING:
                lcd.print(F("Leere Tank  "));
                lcd.print(tank_fillings_remaining);
                lcd.print(F("/"));
                lcd.print(irrigation_timer.fillings_to_irrigate);
                break;
              case STATUS_PUMPING:
                lcd.print(F("F"));lcd.write(byte(GFX_ID_UML_U));lcd.print(F("lle Tank  "));
                lcd.print(tank_fillings_remaining);
                lcd.print(F("/"));
                lcd.print(irrigation_timer.fillings_to_irrigate);
                break;
              case STATUS_NO_WATER:
                lcd.print(F("WASSER LEER!!"));
                break;
              case STATUS_NO_TIME:
                lcd.print(F("Uhrzeit fehlt!"));
                break;
              case STATUS_LOW_BATTERY:
                lcd.print(F("Akku Leer!"));
                lcd.setCursor(11,1);
                char vbat_buf[5];
                sprintf(vbat_buf, "%04.1fV", battery_voltage); //% signals start of number inset command, 0 means use 0 for padding instead of whitespace, 4 means pad the result to 4 characters wide, .1 means only 1 decimal, f means the number is a float
                lcd.print(vbat_buf);
                break;

              default:
              case STATUS_GENERAL_FAIL:
                bool no_error_desc_found = true;

                if (component_errors.tank_sensors_irrational) { //tank sensor error
                  lcd.print(F("TS "));
                  no_error_desc_found = false;
                }

                if (component_errors.rtc_missing or component_errors.rtc_unset) { //rtc error
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
          if (tank_fillings_remaining == 0) {
            lcd.print(F("Gie"));lcd.write(byte(GFX_ID_UML_S));lcd.print(F("e "));
            lcd_print_menu_bracket(1,false);
            lcd.print(page_1_irrigate_order);
            lcd_print_menu_bracket(1,true);
            lcd.print(F("L jetzt")); //the jetzt gets cut off when selecting more than 0L but thats not too bad
          }
          else {
            lcd_print_menu_bracket(1,false);
            lcd.print(F("Abbrechen"));
            lcd_print_menu_bracket(1,true);
          }
          break;
        
        case PAGE_TIMER:
          lcd_print_menu_bracket(1,false);
          if (irrigation_timer.start_hour<10) lcd.print(0);
          lcd.print(irrigation_timer.start_hour);
          lcd_print_menu_bracket(1,true);
          lcd.print(F(":"));
          lcd_print_menu_bracket(2,false);
          if (irrigation_timer.start_minute<10) lcd.print(0);
          lcd.print(irrigation_timer.start_minute);
          lcd_print_menu_bracket(2,true);
          lcd.write(byte(126));
          lcd_print_menu_bracket(3,false);
          lcd.print(round(irrigation_timer.fillings_to_irrigate * settings.tank_capacity));
          lcd_print_menu_bracket(3,true);
          lcd.print(F("L"));
          break;
        
        case PAGE_TANK:
         lcd.print(F("Tank Vol.:"));
          lcd_print_menu_bracket(1,false);
          lcd.print(settings.tank_capacity);
          lcd_print_menu_bracket(1,true);
          lcd.print(F("L"));
          break;
        
        case PAGE_CLOCK1:
          lcd_print_menu_bracket(1,false);
          if (current_time.Hour<10) lcd.print(0);
          lcd.print(current_time.Hour);
          lcd_print_menu_bracket(1,true);
          lcd.print(F(":"));
          lcd_print_menu_bracket(2,false);
          if (current_time.Minute<10) lcd.print(0);
          lcd.print(current_time.Minute);
          lcd_print_menu_bracket(2,true);
          break;
        
        case PAGE_CLOCK2:
          lcd_print_menu_bracket(1,false);
          if (current_time.Day<10) lcd.print(0);
          lcd.print(current_time.Day);
          lcd_print_menu_bracket(1,true);
          lcd.print(F("."));
          lcd_print_menu_bracket(2,false);
          if (current_time.Month<10) lcd.print(0);
          lcd.print(current_time.Month);
          lcd_print_menu_bracket(2,true);
          lcd.print(F("."));
          lcd_print_menu_bracket(3,false);
          lcd.print(tmYearToCalendar(current_time.Year));
          lcd_print_menu_bracket(3,true);
          break;
        
        case PAGE_BATTERY:
          char volt_buf[5];
          
          lcd.print(F("0"));
          //lcd.write(byte(GFX_ID_ARROW_R));
          lcd_print_menu_bracket(1,false);
          sprintf(volt_buf, "%04.1fV", settings.battery_voltage_cutoff);
          lcd.print(volt_buf);
          lcd_print_menu_bracket(1,true);

          lcd.write(byte(GFX_ID_HYST));

          lcd_print_menu_bracket(2,false);
          sprintf(volt_buf, "%04.1fV", settings.battery_voltage_reset);
          lcd.print(volt_buf);
          lcd_print_menu_bracket(2,true);

          //lcd.write(byte(GFX_ID_ARROW_L));
          lcd.print(F("1"));
          break;
    }
    last_display_update = millis();

    //Serial.print(F("Display drawing time [ms]: "));
    //Serial.println(last_display_update - display_draw_start_time);
  }

  //status led
  switch (system_state) {
    case STATUS_IDLE:
      if (irrigation_timer.fillings_to_irrigate == 0) set_status_led(1,0,1);
      else set_status_led(0,1,0);
      break;
    case STATUS_EMPTYING:
      set_status_led(0,0,1);
      break;
    case STATUS_PUMPING:
      set_status_led(0,0,1);
      break;
    case STATUS_NO_WATER:
    case STATUS_NO_TIME:
    case STATUS_LOW_BATTERY:
      set_status_led(1,0,0);
      break;

     default:
     case STATUS_GENERAL_FAIL:
       set_status_led(1,0,0);
       break;
  }
}

void handle_pump_stuff() {
  if (raw_sensors.tank_top == TANK_TOP_ON_LEVEL and raw_sensors.tank_bottom != TANK_BOTTOM_ON_LEVEL) {
    component_errors.tank_sensors_irrational = true;
    system_state = STATUS_GENERAL_FAIL;
  }
  switch (system_state) {
    case STATUS_IDLE:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);
      //the further sth is down in this case, the higher it's priority 

      if (battery_voltage <= settings.battery_voltage_cutoff and tank_fillings_remaining == 0) system_state = STATUS_LOW_BATTERY; //low battery is lowest priority. this is so a currently running irrigation is completed even if the pump motor drops the voltage

      if((irrigation_timer.start_hour <= current_time.Hour and irrigation_timer.start_minute <= current_time.Minute) and (irrigation_timer.last_watering_day != current_time.Day)) { //
        tank_fillings_remaining = irrigation_timer.fillings_to_irrigate;
        irrigation_timer.last_watering_day = current_time.Day;
        EEPROM.put(0+sizeof(settings),irrigation_timer);
      }
      if (tank_fillings_remaining > 0 and raw_sensors.low_water != LOW_WATER_ON_LEVEL) system_state = STATUS_PUMPING;
      
      if (raw_sensors.low_water == LOW_WATER_ON_LEVEL) system_state = STATUS_NO_WATER;
      if (component_errors.rtc_unset) system_state = STATUS_NO_TIME;
      if (component_errors.rtc_missing) system_state = STATUS_GENERAL_FAIL;
      if (raw_sensors.tank_bottom == TANK_BOTTOM_ON_LEVEL) system_state = STATUS_EMPTYING; //if at boot there is still water in the tank, empty it.
      break;

    case STATUS_EMPTYING:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, HIGH);
      if (raw_sensors.tank_bottom != TANK_BOTTOM_ON_LEVEL and raw_sensors.tank_top != TANK_TOP_ON_LEVEL) {
        tank_fillings_remaining--;
        if (tank_fillings_remaining > 60000) tank_fillings_remaining = 0; //against integer rollover
        if (raw_sensors.low_water == LOW_WATER_ON_LEVEL) {
          system_state = STATUS_NO_WATER;
          return;
        }
        if (tank_fillings_remaining==0) {
          system_state = STATUS_IDLE;
        }
        else {
          system_state = STATUS_PUMPING;
        }
      }
      break;

    case STATUS_PUMPING:
      digitalWrite(PUMP_PIN, HIGH);
      digitalWrite(VALVE_PIN, LOW);
      if (raw_sensors.tank_top == TANK_TOP_ON_LEVEL or raw_sensors.low_water == LOW_WATER_ON_LEVEL) {
        system_state = STATUS_EMPTYING;
      }
      break;

    case STATUS_NO_WATER:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);
      if (raw_sensors.low_water != LOW_WATER_ON_LEVEL) system_state = STATUS_IDLE;
      break;

    case STATUS_NO_TIME:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);
      if (!component_errors.rtc_unset) {system_state = STATUS_IDLE; return;}
      break;

    case STATUS_LOW_BATTERY:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);
      if (battery_voltage >= settings.battery_voltage_reset) system_state = STATUS_IDLE;
      break;

     default:
     case STATUS_GENERAL_FAIL:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);
      if ((raw_sensors.tank_top == TANK_TOP_ON_LEVEL) <= (raw_sensors.tank_bottom == TANK_BOTTOM_ON_LEVEL)) component_errors.tank_sensors_irrational = false; //if the top sensor is the same or lower than the bottom sensor, its fine
      if (!component_errors.rtc_missing and !component_errors.tank_sensors_irrational) system_state = STATUS_IDLE;
      break;
  }
}

void read_sensors_and_clock() {
  if (RTC.read(current_time)) {
    component_errors.rtc_missing = false;
    component_errors.rtc_unset = false;
  }
  else {
    if (RTC.chipPresent()) {component_errors.rtc_missing = false; component_errors.rtc_unset = true; system_state = STATUS_NO_TIME;}
    else {component_errors.rtc_missing = true; component_errors.rtc_unset = false;}
  }
  battery_voltage = (float)analogRead(BATTERY_VOLTAGE_PIN) / BATTERY_VOLTAGE_ADC_DIVIDER;

  raw_sensors.low_water = digitalRead(LOW_WATER_PIN);
  raw_sensors.tank_bottom = digitalRead(TANK_BOTTOM_PIN);
  raw_sensors.tank_top = digitalRead(TANK_TOP_PIN);
}

void handle_serial() {
  if (Serial.available()) {
    uint8_t controlCharacter = Serial.read();

    if (controlCharacter == 'd') up_callback();
    if (controlCharacter == 'a') down_callback();
    if (controlCharacter == 's') btn_callback();

    if (controlCharacter == 'V') { //dump all sensor values to serial
      Serial.print(F("Low Water: "));
      Serial.println((raw_sensors.low_water == LOW_WATER_ON_LEVEL) ? F("Water too low") : F("Water fine"));
      Serial.print(F("Bottom Tank Swimmer: "));
      Serial.println((raw_sensors.tank_bottom == TANK_BOTTOM_ON_LEVEL) ? F("Under Water") : F("Dry"));
      Serial.print(F("Top Tank Swimmer: "));
      Serial.println((raw_sensors.tank_top == TANK_TOP_ON_LEVEL) ? F("Under Water") : F("Dry"));
      Serial.print(F("Battery Voltage: "));
      Serial.print(battery_voltage);
      Serial.println('V');

      Serial.println();
    }

    if (controlCharacter == 'R') { //reset all settings
      //todo: write this
    } 
  }
}

void do_stored_buttons() {
  for (uint8_t q_pos = 0; q_pos <= min(31, button_queue_add_pos); q_pos++) { //make sure they are 0
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
    button_queue[q_pos] = 0; //clear position so it wont get executed again
  }
  button_queue_add_pos = 0;
}

void loop() {
  read_sensors_and_clock();
  handle_pump_stuff();
  update_display();

  handle_serial();
  do_stored_buttons();
}
