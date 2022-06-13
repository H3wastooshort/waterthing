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

#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>

#define LOW_WATER_PIN 7
#define TANK_EMPTY_PIN 8
#define TANK_FULL_PIN 9
#define PUMP_PIN 10
#define VALVE_PIN 11

#define RED_LED_PIN A1
#define GREEN_LED_PIN A2
#define BLUE_LED_PIN A3

#define DEBOUNCE_DELAY 100
#define BTN_PIN 2
#define ENCODER_CLK_PIN 3
#define ENCODER_DT_PIN 4

//structs are used to save to EEPROM more easily
struct settings_s {
  uint8_t tank_capacity = 0; //in L
  int16_t max_mins_per_refill = -1; //error is thrown if the pump takes longer than that to fill the tank (5L/min pump wont take more than 5 minutes to fill a 20L tank)
} settings;

struct i_timer_s {
  uint8_t start_hour = 0;
  uint8_t start_minute = 0;
  uint16_t fillings_to_irrigate = 0;
  uint8_t last_watering_day = 0;
} irrigation_timer;

struct c_error_s{
  bool rtc_missing = false;
  bool rtc_unset = false;
} component_errors;

//enums for easier code reading
enum system_state_e { 
  STATUS_IDLE, //doing nothing
  STATUS_EMPTYING, //emptying tank
  STATUS_PUMPING, //pumping water in tank
  STATUS_NO_WATER, //cant get water from tank
  STATUS_NO_TIME, //clock not set

  STATUS_GENERAL_FAIL
};

//global system variables
enum system_state_e system_state = STATUS_IDLE;
tmElements_t current_time;

uint8_t tank_fillings_remaining = 0; //if over 0, run pumping stuff til 0

//global menu variables
int8_t menu_page = 0; //-1 -> not in menu system, 0-> main page, 1 -> Timer Setup, 2 -> Tank Setup, 3 -> Clock Setup
int8_t menu_entry_cursor = 0; //-2 -> not in menu system, 0 -> page selection, 1-255 differs per page
bool menu_editing = false;
uint32_t last_display_update = 0; //this is a global var to allow calling for immediate update by setting this to 0
uint32_t btn_down_time = 0;

//pages

#define N_OF_PAGES 5
const char* page_names[N_OF_PAGES] = {"Status", "Manuell", "Timer", "Tank", "Uhr"};
const uint8_t page_max_cursor[N_OF_PAGES] = {0, 1, 0, 0, 0};

LiquidCrystal_I2C lcd(0x27,16,2);


void change_menu_entry(bool dir) { //true is up
  if (menu_entry_cursor == 0) {
    menu_page += dir ? 1 : -1;
    return;
  }
  switch (menu_page) {
    case 1:


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
  last_display_update = 0;
}

void handle_btn_down() {
  btn_down_time = millis();
}

void handle_btn_up() {
  uint32_t btn_duration = millis() - btn_down_time;
  if (btn_duration >= DEBOUNCE_DELAY) {
    btn_callback();
  }
}


void set_status_led(bool r, bool g, bool b) {
  digitalWrite(RED_LED_PIN,r);
  digitalWrite(GREEN_LED_PIN,g);
  digitalWrite(BLUE_LED_PIN,b);
}

void setup() {
  //pump stuff
  pinMode(LOW_WATER_PIN, INPUT_PULLUP);
  pinMode(TANK_EMPTY_PIN, INPUT_PULLUP);
  pinMode(TANK_FULL_PIN, INPUT_PULLUP);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);

  //rgb led
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);

  set_status_led(1,1,0);

  Serial.begin(9600);

  lcd.init();
  lcd.backlight();


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
      lcd.setCursor(0, 0);
      lcd.print(F("RTC NOT SET"));
      lcd.setCursor(0, 1);
      lcd.print(F("Setings->SetDate")); //Typo on purpose, 16char limit
      delay(5000);
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

  attachInterrupt(digitalPinToInterrupt(BTN_PIN), handle_btn_down, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), handle_btn_up, RISING);

  Serial.println();
  set_status_led(0,0,0);
}

void print_page_basics() {
  lcd.clear();
  lcd.home();
  lcd.print(menu_entry_cursor == 0 ? (menu_editing ? F("{") : F("[")) : F("("));
  lcd.print(menu_page);
  lcd.print(menu_entry_cursor == 0 ? (menu_editing ? F("}") : F("]")) : F(")"));
  lcd.print(F(""));
  lcd.print(page_names[menu_page]);
  lcd.setCursor(11,0);
  char clock_buf[5];
  sprintf(clock_buf, "%02u:%02u", current_time.Hour, current_time.Minute);
  lcd.print(clock_buf);
  lcd.setCursor(0,1);
}

void update_display() {
  if (millis() - last_display_update > 1000) {
      print_page_basics();
      switch (menu_page) {
        default:
          menu_page = 0;
        case -1:
          break;
        case 0:
            switch (system_state) {
              case STATUS_IDLE:
                lcd.print(F("Stby. bis "));
                lcd.setCursor(11,1);
                char til_buf[5];
                sprintf(til_buf, "%02u:%02u", irrigation_timer.start_hour, irrigation_timer.start_minute);
                lcd.print(til_buf);
                break;
              case STATUS_EMPTYING:
                lcd.print(F("Leere Tank  "));
                lcd.print(irrigation_timer.fillings_to_irrigate - tank_fillings_remaining +1);
                lcd.print(F("/"));
                lcd.print(irrigation_timer.fillings_to_irrigate);
                break;
              case STATUS_PUMPING:
                lcd.print(F("Fülle Tank  "));
                lcd.print(irrigation_timer.fillings_to_irrigate - tank_fillings_remaining +1);
                lcd.print(F("/"));
                lcd.print(irrigation_timer.fillings_to_irrigate);
                break;
              case STATUS_NO_WATER:
                lcd.print(F("WASSER LEER!!"));
                break;
              case STATUS_NO_TIME:
                lcd.print(F("Uhrzeit fehlt!"));
                break;

              default:
              case STATUS_GENERAL_FAIL:
                lcd.print(F("Algem. Fehler"));
                break;
            }
          break;
        case 1:
          lcd.print(F("Gieße "));
          lcd.print(menu_entry_cursor == 1 ? (menu_editing ? F("{") : F("[")) : F("("));
          lcd.print(" ");
          lcd.print(menu_entry_cursor == 1 ? (menu_editing ? F("}") : F("]")) : F(")"));
          lcd.print(F("L"));
          break;
      }
    last_display_update = millis();
  }

  //status led
  switch (system_state) {
              case STATUS_IDLE:
                set_status_led(0,1,0);
                break;
              case STATUS_EMPTYING:
                set_status_led(0,0,1);
                break;
              case STATUS_PUMPING:
                set_status_led(0,0,1);
                break;
        case STATUS_NO_WATER:
      set_status_led(1,0,0);
     break;
    case STATUS_NO_TIME:
      set_status_led(1,0,0);
      break;

     default:
     case STATUS_GENERAL_FAIL:
       set_status_led(1,0,0);
       break;
  }
}

void handle_pump_stuff() {
  switch (system_state) {
    case STATUS_IDLE:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);

      if (component_errors.rtc_missing) {system_state = STATUS_GENERAL_FAIL; return;}
      if (component_errors.rtc_unset) {system_state = STATUS_NO_TIME; return;}

      if((irrigation_timer.start_hour < current_time.Hour and irrigation_timer.start_minute < current_time.Minute) and (irrigation_timer.last_watering_day != current_time.Day)) { //
        tank_fillings_remaining = irrigation_timer.fillings_to_irrigate;
        irrigation_timer.last_watering_day != current_time.Day;
      }
      
      if (tank_fillings_remaining > 0 and digitalRead(LOW_WATER_PIN)) system_state = STATUS_PUMPING;
      if (!digitalRead(LOW_WATER_PIN)) system_state = STATUS_NO_WATER;
      break;

    case STATUS_EMPTYING:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, HIGH);
      if (!digitalRead(TANK_EMPTY_PIN)) {
        tank_fillings_remaining--;
        if (tank_fillings_remaining==0 or !digitalRead(LOW_WATER_PIN)) {
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
      if (!digitalRead(TANK_FULL_PIN) or !digitalRead(LOW_WATER_PIN)) {
        system_state = STATUS_EMPTYING;
      }
      break;

    case STATUS_NO_WATER:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);
      if (digitalRead(LOW_WATER_PIN)) system_state = STATUS_IDLE;
      break;

    case STATUS_NO_TIME:
      digitalWrite(PUMP_PIN, LOW);
      digitalWrite(VALVE_PIN, LOW);
      if (!component_errors.rtc_unset) {system_state = STATUS_IDLE; return;}
      break;

     default:
     case STATUS_GENERAL_FAIL:
       break;
  }
}

void read_clock_and_stuff() {
    if (RTC.read(current_time)) {
    component_errors.rtc_missing = false;
    component_errors.rtc_unset = false;
  }
  else {
    if (RTC.chipPresent()) {component_errors.rtc_missing = false; component_errors.rtc_unset = true;}
    else {component_errors.rtc_missing = true; component_errors.rtc_unset = false;}
  }
}

void loop() {
  read_clock_and_stuff();
  handle_pump_stuff();
  update_display();
}
