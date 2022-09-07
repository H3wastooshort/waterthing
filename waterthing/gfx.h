//dynamic status
byte gfx_off[8] = {B01110, B01010, B01110, B00000, B11011, B10010, B11011, B10010};
byte gfx_idle[8] = {B01110, B10001, B10111, B10101, B01110, B00000, B10101, B00000}; //clock 3 dots version
//byte gfx_idle[8] = {B00000, B00000, B00000, B00000, B00000, B00000, B10101, B00000}; //... version
//byte gfx_idle[8] = {B00110, B00011, B01100, B00110, B11000, B01000, B10000, B11000}; //zZZ version
byte gfx_done[8] = {B01000, B01000, B11100, B11101, B01001, B00010, B01010, B00100};
byte gfx_fill[8] = {B00100, B10101, B01110, B00100, B00000, B10001, B10001, B11111};
byte gfx_drain[8] = {B10001, B10001, B11111, B00000, B00100, B10101, B01110, B00100};
byte gfx_afterdrain[8] = {B00000, B10001, B10001, B11111, B00000, B00000, B10101, B00000};
byte gfx_no_water[8] = {B00001, B00001, B01000, B01101, B01100, B10110, B11110, B01100};
byte gfx_low_bat[8] = {B01110, B10001, B10101, B10101, B10001, B10101, B10001, B11111};
byte gfx_rain_detected[8] = {B00100, B00100, B01110, B10111, B11111, B01110, B00000, B10101};
byte gfx_error[8] = {B10101, B10101, B10101, B10101, B00000, B00000, B10101, B00000};

//symbol graphics
byte gfx_hyst[8] = {B00111, B01010, B01010, B01010, B01010, B01010, B01010, B11100};
byte gfx_drop[8] = {B00100, B00100, B01110, B01110, B10111, B11111, B01110, B00000};
byte gfx_clock[8] = {B01110, B00100, B01110, B10001, B10111, B10101, B01110, B00000};
byte gfx_flow[8] = {B00100, B01100, B10110, B11110, B01100, B00010, B11001, B00010};
byte gfx_pulse[8] = {B00000, B01110, B01010, B01010, B01010, B01010, B11011, B00000};
byte gfx_h3[8] = {B00000, B00000, B10110, B10001, B11110, B10001, B10110, B00000};
byte gfx_tank_top[8] = {B10011, B01011, B10011, B00011, B00011, B00011, B00011, B00011};
byte gfx_tank_bottom[8] = {B00011, B00011, B00011, B00011, B00011, B10011, B01011, B10011};
byte gfx_key[8] = {B01110, B10001, B01110, B00100, B01100, B00100, B01100, B00100};
byte gfx_radio[8] = {B00010, B01001, B00101, B10101, B00101, B10001, B10000, B10000};
byte gfx_rain[8] = {B01100, B10010, B10001, B11111, B00000, B10101, B00000, B10101};
byte gfx_rise[8] = {B00011, B00010, B00010, B00010, B00010, B00010, B11110, B00000};
byte gfx_fall[8] = {B11000, B01000, B01000, B01000, B01000, B01000, B01111, B00000};

//umlaut graphics
byte gfx_uml_s[8] = {B01100, B10010, B10010, B11100, B10010, B10010, B10100, B10000};
byte gfx_uml_a[8] = {B01010, B00000, B01110, B00001, B01111, B10001, B01111, B00000};
byte gfx_uml_o[8] = {B01010, B00000, B01110, B10001, B10001, B10001, B01110, B00000};
byte gfx_uml_u[8] = {B01010, B00000, B10001, B10001, B10001, B10011, B01101, B00000};
