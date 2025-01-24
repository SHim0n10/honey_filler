#include <U8g2lib.h>
#include <string>
#include <Preferences.h>
#include <HX711.h>

// read data from storage

Preferences pref;
HX711 scale;

#define LOADCELL_DOUT_PIN  26
#define LOADCELL_SCK_PIN  25

#define outputA 33
#define outputB 32
#define outputSwitch 23

unsigned long _lastIncReadTime = millis(); 
unsigned long _lastDecReadTime = millis(); 
int _pauseLength = 250;
int _fastIncrement = 4;
/*

regular weigh

Menu: 
(menu_index == 1)
-fill  
-------(menu_index == 2)
-------return to menu <<
-------preset 1
-------preset 2
-------preset 3
-------custom
-------------(menu_index == 3 -> press to start (value 0 == exit))
-------------make one time fill
-------change preset       
-------------(menu_index == 4 -> press to choose)
-------------return back <<
-------------change preset 1  (menu_index == 5 -> press to change)
-------------change preset 2  (menu_index == 6 -> press to change)
-------------change preset 3  (menu_index == 7 -> press to change)
-calibration
-------------(menu_index == 8 -> press to continue/change)
-------------please empty the surface
-------------place object and insert weight value

-manual control
-------------(menu_index == 9 -> press to exit)
-------------manual control servo

-digital scale
-------------(menu_index == 0 -> press to menu)

*/
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);
U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 12, /* CS=*/ 14, /* reset=*/ 18);

// BITMAPS

// 'sel_border', 128x21px
const unsigned char epd_bitmap_sel_border [] PROGMEM = {
	0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 
	0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 
	0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 
	0xf8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f
};//(Total bytes used to store images in PROGMEM = 352)

// 'logo-inv', 56x58px
const unsigned char epd_bitmap_logo_inv [] PROGMEM = {
	0x00, 0x00, 0xe0, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0x07, 0x00, 0x00, 0x00, 0x00, 
	0x70, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 
	0x0c, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x38, 0x00, 
	0x00, 0x00, 0x00, 0x0e, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x70, 0x00, 0x00, 0xe0, 
	0xff, 0x07, 0x00, 0xe0, 0xff, 0x03, 0xe0, 0xff, 0x03, 0x00, 0xe0, 0xff, 0x07, 0x70, 0x00, 0x37, 
	0x00, 0x66, 0x00, 0x06, 0x38, 0x00, 0x76, 0x00, 0x67, 0x00, 0x0e, 0x18, 0x00, 0xc6, 0x81, 0x33, 
	0x00, 0x1c, 0x1c, 0x00, 0x8c, 0xc1, 0x38, 0x00, 0x1c, 0x0c, 0x00, 0x1c, 0xe3, 0x1c, 0x00, 0x38, 
	0x0e, 0x00, 0x38, 0x7f, 0x0e, 0x00, 0x30, 0x07, 0x00, 0x30, 0x7e, 0x0e, 0x00, 0x70, 0x07, 0x78, 
	0x30, 0x77, 0x06, 0x1f, 0x60, 0x03, 0xfe, 0x03, 0xe3, 0xe0, 0x3f, 0xe0, 0x07, 0x07, 0x0f, 0xff, 
	0xf8, 0x60, 0xe0, 0x87, 0x03, 0x7c, 0xff, 0x1e, 0xc0, 0x70, 0x8e, 0x01, 0xe0, 0xff, 0x07, 0xc0, 
	0x30, 0x8c, 0x01, 0x80, 0xf7, 0x01, 0x80, 0x39, 0x9c, 0x00, 0x00, 0x7f, 0x00, 0x80, 0x19, 0x98, 
	0x01, 0xc0, 0xf7, 0x01, 0x80, 0x0d, 0x98, 0x01, 0xe0, 0xe7, 0x07, 0xc0, 0x0c, 0xb0, 0x03, 0xfc, 
	0xc3, 0x1f, 0xc0, 0x04, 0x30, 0x07, 0xcf, 0xc1, 0xf9, 0x60, 0x06, 0x70, 0xfe, 0xc3, 0x81, 0xe3, 
	0x3f, 0x06, 0x70, 0x78, 0xe0, 0xff, 0x03, 0x1f, 0x06, 0x38, 0x30, 0xb0, 0xff, 0x06, 0x06, 0x0e, 
	0x18, 0x60, 0x98, 0xc3, 0x1d, 0x07, 0x1c, 0x1c, 0xc0, 0xcf, 0x81, 0xf9, 0x03, 0x18, 0x0c, 0x80, 
	0xc7, 0x80, 0xe3, 0x00, 0x38, 0x0e, 0x00, 0xe0, 0xff, 0x03, 0x00, 0x30, 0x07, 0x00, 0xe0, 0xff, 
	0x03, 0x00, 0x70, 0x07, 0x00, 0xe0, 0x81, 0x03, 0x00, 0x60, 0x03, 0x00, 0x60, 0x00, 0x03, 0x00, 
	0x60, 0x07, 0x00, 0xe0, 0x00, 0x03, 0x00, 0x60, 0x07, 0x00, 0xc0, 0xff, 0x09, 0x00, 0x70, 0x0e, 
	0x00, 0xd8, 0xff, 0x09, 0x00, 0x30, 0x0c, 0x00, 0x9c, 0xe7, 0x1c, 0x00, 0x38, 0x1c, 0x00, 0x1c, 
	0xe7, 0x38, 0x00, 0x1c, 0x18, 0x00, 0x0e, 0x7e, 0x38, 0x00, 0x1c, 0x38, 0x00, 0x0e, 0x3e, 0x70, 
	0x00, 0x0e, 0x70, 0x00, 0x07, 0x1c, 0xf0, 0x00, 0x06, 0xe0, 0xff, 0x07, 0x18, 0xe0, 0xff, 0x07, 
	0xe0, 0xff, 0x07, 0x08, 0xe0, 0xff, 0x03, 0x00, 0x00, 0x06, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 
	0x0e, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x1c, 0x00, 
	0x1c, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x0e, 0x00, 
	0x00, 0x00, 0x00, 0x70, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xff, 0x07, 0x00, 0x00, 0x00, 
	0x00, 0xe0, 0xff, 0x03, 0x00, 0x00
};

// 'digital-scale1', 16x16px
const unsigned char epd_bitmap_digital_scale1 [] PROGMEM = {
	0x80, 0x01, 0x80, 0x01, 0x80, 0x01, 0x9c, 0x39, 0xfe, 0x7f, 0x08, 0x10, 0x14, 0x28, 0x14, 0x28, 
	0x22, 0x44, 0x22, 0x44, 0x41, 0x82, 0x7f, 0xfe, 0x3e, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'digital-scale2', 16x16px
const unsigned char epd_bitmap_digital_scale2 [] PROGMEM = {
	0x80, 0x01, 0x80, 0x01, 0x80, 0x39, 0x80, 0x7d, 0xdc, 0x13, 0x3e, 0x28, 0x08, 0x28, 0x14, 0x44, 
	0x14, 0x44, 0x22, 0x82, 0x22, 0xfe, 0x41, 0x7c, 0x7f, 0x00, 0x3e, 0x00, 0x00, 0x00, 0x00, 0x00
};
// 'digital-scale3', 16x16px
const unsigned char epd_bitmap_digital_scale3 [] PROGMEM = {
	0x80, 0x01, 0x80, 0x01, 0x9c, 0x01, 0xbe, 0x01, 0xc8, 0x3b, 0x14, 0x7c, 0x14, 0x10, 0x22, 0x28, 
	0x22, 0x28, 0x41, 0x44, 0x7f, 0x44, 0x3e, 0x82, 0x00, 0xfe, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00
};

// 'cal-icon1', 16x16px
const unsigned char epd_bitmap_cal_icon1 [] PROGMEM = {
	0x80, 0x01, 0xc0, 0x03, 0xb0, 0x0d, 0x88, 0x11, 0xc4, 0x23, 0xa4, 0x25, 0x12, 0x48, 0x3f, 0xfc, 
	0x3f, 0xfc, 0x12, 0x48, 0xa4, 0x25, 0xc4, 0x23, 0x88, 0x11, 0xb0, 0x0d, 0xc0, 0x03, 0x80, 0x01
};
// 'cal-icon2', 16x16px
const unsigned char epd_bitmap_cal_icon2 [] PROGMEM = {
	0x03, 0x80, 0xc6, 0xc3, 0x3c, 0x6c, 0x18, 0x30, 0xf4, 0x3b, 0x64, 0x2c, 0x52, 0x4e, 0x12, 0x48, 
	0x12, 0x48, 0x72, 0x4a, 0x34, 0x26, 0xdc, 0x2f, 0x0c, 0x18, 0x36, 0x3c, 0xc3, 0x63, 0x01, 0xc0
};

// 'settings-icon1', 16x16px
const unsigned char epd_bitmap_settings_icon1 [] PROGMEM = {
	0x80, 0x01, 0x48, 0x12, 0x54, 0x2a, 0x22, 0x44, 0x04, 0x20, 0x88, 0x11, 0x46, 0x62, 0x21, 0x84, 
	0x21, 0x84, 0x46, 0x62, 0x88, 0x11, 0x04, 0x20, 0x22, 0x44, 0x54, 0x2a, 0x48, 0x12, 0x80, 0x01
};
// 'settings-icon2', 16x16px
const unsigned char epd_bitmap_settings_icon2 [] PROGMEM = {
	0x20, 0x1c, 0x50, 0x12, 0x48, 0x09, 0x4b, 0x35, 0x95, 0x48, 0x09, 0x80, 0x82, 0x71, 0x4c, 0x0a, 
	0x50, 0x32, 0x8e, 0x41, 0x01, 0x90, 0x12, 0xa9, 0xac, 0xd2, 0x90, 0x12, 0x48, 0x0a, 0x38, 0x04
};
// 'fill_icon', 16x16px
const unsigned char epd_bitmap_fill_icon [] PROGMEM = {
	0xfc, 0x3f, 0x02, 0x40, 0xfc, 0x3f, 0xbc, 0x3d, 0x7e, 0x7e, 0xa6, 0x65, 0x5a, 0x5a, 0xba, 0x5d, 
	0x86, 0x61, 0x1e, 0x78, 0xde, 0x7b, 0x1e, 0x78, 0xde, 0x7b, 0x3e, 0x7c, 0x7c, 0x3e, 0xf8, 0x1f
};
// 'fill_icon1', 16x16px
const unsigned char epd_bitmap_fill_icon1 [] PROGMEM = {
	0xfc, 0x3f, 0x02, 0x40, 0xfc, 0x3f, 0xbc, 0x3d, 0x7e, 0x7e, 0xa6, 0x65, 0x5a, 0x5a, 0xba, 0x5d, 
	0x86, 0x61, 0x1e, 0x78, 0xde, 0x7b, 0x1e, 0x78, 0x22, 0x44, 0xc2, 0x43, 0x84, 0x21, 0xf8, 0x1f
};
// 'fill_icon2', 16x16px
const unsigned char epd_bitmap_fill_icon2 [] PROGMEM = {
	0xfc, 0x3f, 0x02, 0x40, 0xfc, 0x3f, 0xbc, 0x3d, 0x7e, 0x7e, 0xa6, 0x65, 0x5a, 0x5a, 0xba, 0x5d, 
	0x86, 0x61, 0xe2, 0x47, 0x22, 0x44, 0xe2, 0x47, 0x22, 0x44, 0xc2, 0x43, 0x84, 0x21, 0xf8, 0x1f
};
// 'fill_icon3', 16x16px
const unsigned char epd_bitmap_fill_icon3 [] PROGMEM = {
	0xfc, 0x3f, 0x02, 0x40, 0xfc, 0x3f, 0xbc, 0x3d, 0x7e, 0x7e, 0xa6, 0x65, 0xa6, 0x65, 0x46, 0x62, 
	0x7a, 0x5e, 0xe2, 0x47, 0x22, 0x44, 0xe2, 0x47, 0x22, 0x44, 0xc2, 0x43, 0x84, 0x21, 0xf8, 0x1f
};
// 'fill_icon4', 16x16px
const unsigned char epd_bitmap_fill_icon4 [] PROGMEM = {
	0xfc, 0x3f, 0x02, 0x40, 0xfc, 0x3f, 0x44, 0x22, 0x82, 0x41, 0x5a, 0x5a, 0xa6, 0x65, 0x46, 0x62, 
	0x7a, 0x5e, 0xe2, 0x47, 0x22, 0x44, 0xe2, 0x47, 0x22, 0x44, 0xc2, 0x43, 0x84, 0x21, 0xf8, 0x1f
};

// 'border', 128x12px
const unsigned char epd_bitmap_border [] PROGMEM = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};



// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 240)
const unsigned char* bitmap_icons[4] [5] = {
  {epd_bitmap_digital_scale1, epd_bitmap_digital_scale2, epd_bitmap_digital_scale1, epd_bitmap_digital_scale3},
	{epd_bitmap_fill_icon, epd_bitmap_fill_icon1, epd_bitmap_fill_icon2, epd_bitmap_fill_icon3, epd_bitmap_fill_icon4},
  {epd_bitmap_cal_icon1, epd_bitmap_cal_icon2},
  {epd_bitmap_settings_icon1, epd_bitmap_settings_icon2}
};

int preset1, preset2, preset3;

//confirmation
uint8_t choice = 0;

uint8_t switch_counter = 0;

int32_t weight = 8888;
uint16_t input_weight = 0;
uint8_t menu_index = 1;
int item_index = 0;
uint8_t item_selected = 0;
int item_sel_previous;
int item_sel_next;
unsigned long oldTime = 0;
unsigned long lastDebounceTime = 0;
unsigned long dt = 500;
int aState;
int aLastState;
int counter = 0; 


uint8_t calibration = 0;
int offset = 0;

uint8_t filling = 0;

//list of items menu attributes
int8_t selected_item = 0;
uint8_t border_index = 0;
uint8_t start_index = 0;

//menu array
const int ITEMS_NUM = 4;
const int MAX_ITEM_LENGTH = 15;

char menu_items [ITEMS_NUM] [MAX_ITEM_LENGTH] = {
  { "Digital scale" },
  { "Fill honey" },
  { "Calibration" },
  { "Control" }
};

//Honey fill menu
const int HONEY_ITEMS_NUM = 6;
const int HONEY_MAX_ITEM_LENGTH = 15;

char honey_menu_items [HONEY_ITEMS_NUM] [HONEY_MAX_ITEM_LENGTH] = {
  { "Menu" },
  { "P1" },
  { "P2" },
  { "P3" },
  { "Custom" },
  { "Change presets" }
};

//Honey preset menu
const int PRESET_ITEMS_NUM = 4;
const int PRESET_MAX_ITEM_LENGTH = 5;

char preset_menu_items [PRESET_ITEMS_NUM] [PRESET_MAX_ITEM_LENGTH] = {
  { "Back" },
  { "P1" },
  { "P2" },
  { "P3" }
};


void encoder_change(int value) {
  if (menu_index == 1) {  // main_menu
    if (value >= 1) {
      if (item_selected+1>=ITEMS_NUM){
        item_selected = 0;
      }
      else {
        item_selected++;
      }
    }
    else if(value <= -1){
      if (item_selected-1<0){
        item_selected = ITEMS_NUM-1;
      }
      else {
        item_selected--;
      }
      }
  item_index = 0;
  }
  else if (menu_index == 2) {  // honey_fill_menu
    if (value >= 1) {
      if (selected_item+1<HONEY_ITEMS_NUM) { // pokial nie je na konci
        if (border_index==4) { // ak je border naspodu
          selected_item++;
          start_index++;
        }
        else {
          border_index++;
          selected_item++;
        }
      }
    }
    else if (value <= -1) {
      if (selected_item-1>=0) { // pokial nie je na zaciatku
        if (border_index==0) { // ak je border navrchu
          selected_item--;
          start_index--;
        }
        else {
          border_index--;
          selected_item--;
        }
      }
    }
  }
  else if (menu_index == 3) {
    if (value >= 1) {
      input_weight += value*5;
      Serial.println(input_weight);
    }
    else if (value <= -1) {
      if (input_weight > 0) {
        if (input_weight+value*5 <= 0) {
          input_weight = 0;
        }
        else {
          input_weight += value*5;
        }
      }
      else if (input_weight < 0) {
        input_weight = 0;
      }
      Serial.println(input_weight);
    }
  }
  else if (menu_index == 4) { // honey_preset_menu
    if (value >= 1) {
      if (selected_item+1<PRESET_ITEMS_NUM) { // pokial nie je na konci
        border_index++;
        selected_item++;
      }
    }
    else if (value <= -1) {
      if (selected_item-1>=0) { // pokial nie je na zaciatku
        border_index--;
        selected_item--;
      }
    }
  }
  else if (menu_index == 5 || menu_index == 6 || menu_index == 7) {
    if (value >= 1) {
      if (input_weight+value*5 <= 0) {
        input_weight = 0;
      }
      else {
        input_weight += value*5;
      }
    }
    else if (value <= -1) {
      if (input_weight > 0) {
        if (input_weight+value*5 <= 0) {
          input_weight = 0;
        }
        else {
          input_weight += value*5;
        }
      }
      else if (input_weight < 0) {
        input_weight = 0;
      }
    }
  }
  else if (menu_index == 10) {
    if (value >= 1) {
      choice = 1;
    }
    else if (value <= -1) {
      choice = 0;
    }
  }
  else if (menu_index == 8) {
    if (value >= 1) {
      if (input_weight+value*5 <= 0) {
        input_weight = 0;
      }
      else {
        input_weight += value*5;
      }
    }
    else if (value <= -1) {
      if (input_weight > 0) {
        if (input_weight+value*5 <= 0) {
          input_weight = 0;
        }
        else {
          input_weight += value*5;
        }
      }
      else if (input_weight < 0) {
        input_weight = 0;
      }
    }
    Serial.println(input_weight);
  }
}

void read_encoder() {
  // Encoder interrupt routine for both pins. Updates counter
  // if they are valid and have rotated a full indent

  static uint8_t old_AB = 3;  // Lookup table index
  static int8_t encval = 0;   // Encoder value  
  static const int8_t enc_states[]  = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Lookup table

  old_AB <<=2;  // Remember previous state

  if (digitalRead(outputA)) old_AB |= 0x02; // Add current state of pin A
  if (digitalRead(outputB)) old_AB |= 0x01; // Add current state of pin B

  encval += enc_states[( old_AB & 0x0f )];

  // Update counter if encoder has rotated a full indent, that is at least 4 steps
  if( encval > 3 ) {        // Four steps forward
    int changevalue = 1;
    if((millis() - _lastIncReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastIncReadTime = millis();
    // counter1 = counter1 + changevalue;              // Update counter
    encoder_change(changevalue);
    encval = 0;
  }
  else if( encval < -3 ) {        // Four steps backward
    int changevalue = -1;
    if((millis() - _lastDecReadTime) < _pauseLength) {
      changevalue = _fastIncrement * changevalue; 
    }
    _lastDecReadTime = millis();
    // counter1 = counter1 + changevalue;              // Update counter
    encoder_change(changevalue);
    encval = 0;
  }
} 

// change of interface logic
void IRAM_ATTR switch_encoder() {

  unsigned long currentMillis = millis();
  if (currentMillis - lastDebounceTime >= 500) {
    lastDebounceTime = currentMillis;
  
 
    Serial.println("ON");
      if (menu_index == 0) {  // digital_scale
        menu_index = 1;
        scale.power_down();
      }
      else if (menu_index == 1) {  // main_menu
        switch(item_selected) {
          case 0:
            oldTime = millis();
            menu_index = 0;
            scale.power_up();
            break;
          case 1:
            menu_index = 2;
            break;
          case 2:
            menu_index = 8;
            break;
          case 3:
            menu_index = 9;
            break;
        }
      }
      else if (menu_index == 2) {  // honey_fill_menu
        switch(selected_item) {
          case 0:
            menu_index = 1;
            break;
          case 1:
            Serial.println("Fill menu 1");
            input_weight = preset1;
            choice = 1;
            menu_index = 10;
            break;
          case 2:
            Serial.println("Fill menu 2");
            input_weight = preset2;
            choice = 1;
            menu_index = 10;
            break;
          case 3:
            Serial.println("Fill menu 3");
            input_weight = preset3;
            choice = 1;
            menu_index = 10;
            break;
          case 4:
            input_weight = 0;
            menu_index = 3;
            break;
          case 5:
            selected_item = 0;
            border_index = 0;
            start_index = 0;
            menu_index = 4;
            break;
        }
      }
      else if (menu_index == 3) {
        if (input_weight == 0) {
          menu_index = 2;
        }
        else {
          choice = 1;
          menu_index = 10;
          Serial.println(input_weight);
        }
      }
      else if (menu_index == 4) {  // honey_fill_menu
        switch(selected_item) {
          case 0:
            menu_index = 2;
            break;
          case 1:
            input_weight = preset1;
            menu_index = 5;
            break;
          case 2:
            input_weight = preset2;
            menu_index = 6;
            break;
          case 3:
            input_weight = preset3;
            menu_index = 7;
            break;
        }
      }
      else if (menu_index == 5) {
        preset1 = input_weight;
        pref.begin("storage", false);
        pref.putInt("preset1", input_weight);
        pref.end();
        menu_index = 4;
      }
      else if (menu_index == 6) {
        preset2 = input_weight;
        pref.begin("storage", false);
        pref.putInt("preset2", input_weight);
        pref.end();
        menu_index = 4;
      }
      else if (menu_index == 7) {
        preset3 = input_weight;
        pref.begin("storage", false);
        pref.putInt("preset3", input_weight);
        pref.end();
        menu_index = 4;
      }
      else if (menu_index == 10) {
        if (choice == 0) {
          Serial.print("Yes - filling amount:");
          Serial.println(input_weight);
          scale.power_up();
          filling = 0;
          menu_index = 11;
        }
        else if (choice == 1) {
          menu_index = 2;
        }
      }
      else if (menu_index == 8) {
        if (calibration == 1) {
          calibration = 2;
        }
        else if (calibration == 3) {
          calibration = 4;
        }
        else if (calibration == 4) {
          menu_index = 1;
          scale.power_down();
        }
      }
      else if (menu_index == 11) {
        if (filling == 0) {
          filling = 1;
        }
        else if (filling == 2) {
          filling = 0;
          menu_index = 2;
          scale.power_down();
        }
      }
  }

} 

void home()
{
  u8g2.clearBuffer();

  unsigned long currentMillis = millis();

  //kazdych 200ms sa vypise hodnota z vahy
  if (currentMillis - oldTime >= 200) {
    oldTime = currentMillis;
  
  //vypisovanie hodnoty z vahy
  Serial.println(scale.get_units(1));
  weight = scale.get_units(1);
  u8g2.setFont(u8g2_font_helvB24_tr);
  }
  char text[8];
  snprintf(text, sizeof(text), "%d g", weight);

  
  int textWidth = u8g2.getStrWidth(text);
  int xPos = 114 - textWidth;
  u8g2.drawStr(xPos, 43, text);

  u8g2.sendBuffer();
}

void main_menu()
{
  item_sel_previous = item_selected - 1;
  if (item_sel_previous < 0) {item_sel_previous = ITEMS_NUM - 1;}
  item_sel_next = item_selected + 1;  
  if (item_sel_next >= ITEMS_NUM) {item_sel_next = 0;}
  if (millis() - oldTime >= dt){
    item_index++;
    oldTime = millis();
    if (item_selected == 0 && item_index > 3){
      item_index = 0;
    }
    else if (item_selected == 1 && item_index > 4){
      item_index = 0;
    }
    else if (item_selected == 2 && item_index > 1){
      item_index = 0;
    }
    else if (item_selected == 3 && item_index > 1){
      item_index = 0;
    }
  }
  u8g2.clearBuffer();
  
  u8g2.drawXBMP(0, 22, 128, 21, epd_bitmap_sel_border);

  u8g2.setFont(u8g2_font_helvR12_tr);
  u8g2.drawStr(25, 15, menu_items[item_sel_previous]);
  u8g2.drawXBMP(4,2,16,16,bitmap_icons[item_sel_previous][0]);

  u8g2.setFont(u8g2_font_helvB12_tr); 
  u8g2.drawStr(25, 15+20+2, menu_items[item_selected]);
  u8g2.drawXBMP(4,24,16,16,bitmap_icons[item_selected][item_index]);

  u8g2.setFont(u8g2_font_helvR12_tr); 
  u8g2.drawStr(25, 15+20+20+2+2, menu_items[item_sel_next]);
  u8g2.drawXBMP(4,46,16,16,bitmap_icons[item_sel_next][0]);

  u8g2.sendBuffer();
  
}


void honey_fill_menu()
{

  u8g2.clearBuffer();

  u8g2.drawXBMP(0, 2+border_index*12, 128, 12, epd_bitmap_border);

  u8g2.setFont(u8g2_font_6x10_tr);

  for (int h = 0; h < 5; h++){
    u8g2.drawStr(15, 11+h*12, honey_menu_items[h + start_index]);
    if (h+start_index == 0) {
      u8g2.setFont(u8g2_font_unifont_t_symbols);
      u8g2.drawUTF8(4, 12+h*12, "←");
      u8g2.setFont(u8g2_font_6x10_tr);
    }
    else 
    if (h+start_index == 1) {
      char text[8];
      snprintf(text, sizeof(text), "%d g", preset1);
      u_int8_t posX = 125 - u8g2.getStrWidth(text);
      u8g2.drawStr(posX, 11+h*12, text);
    }
    else if (h+start_index == 2) {
      char text[8];
      snprintf(text, sizeof(text), "%d g", preset2);
      u_int8_t posX = 125 - u8g2.getStrWidth(text);
      u8g2.drawStr(posX, 11+h*12, text);
    }
    else if (h+start_index == 3) {
      char text[8];
      snprintf(text, sizeof(text), "%d g", preset3);
      u_int8_t posX = 125 - u8g2.getStrWidth(text);
      u8g2.drawStr(posX, 11+h*12, text);
    }
  }

  u8g2.sendBuffer();
  
}

void honey_fill_custom()
{

  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(16 ,15, "Rotate to change");

  u8g2.setFont(u8g2_font_fub20_tr);
  // const char* text = "780 g";
  char text[8];
  snprintf(text, sizeof(text), "%d g", input_weight);

  
  int textWidth = u8g2.getStrWidth(text);
  int xPos = 106 - textWidth;
  u8g2.drawStr(xPos, 42, text);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(41 ,58, "O to exit");

  u8g2.sendBuffer();

}

void honey_preset_menu()
{

  u8g2.clearBuffer();

  u8g2.drawXBMP(0, 2+border_index*12, 128, 12, epd_bitmap_border);

  u8g2.setFont(u8g2_font_6x10_tr);

  for (int h = 0; h < 4; h++){
    u8g2.drawStr(15, 11+h*12, preset_menu_items[h + start_index]);
    if (h+start_index == 0) {
      u8g2.setFont(u8g2_font_unifont_t_symbols);
      u8g2.drawUTF8(4, 12+h*12, "←");
      u8g2.setFont(u8g2_font_6x10_tr);
    }
    else if (h+start_index == 1) {
      char text[8];
      snprintf(text, sizeof(text), "%d g", preset1);
      u_int8_t posX = 125 - u8g2.getStrWidth(text);
      u8g2.drawStr(posX, 11+h*12, text);
    }
    else if (h+start_index == 2) {
      char text[8];
      snprintf(text, sizeof(text), "%d g", preset2);
      u_int8_t posX = 125 - u8g2.getStrWidth(text);
      u8g2.drawStr(posX, 11+h*12, text);
    }
    else if (h+start_index == 3) {
      char text[8];
      snprintf(text, sizeof(text), "%d g", preset3);
      u_int8_t posX = 125 - u8g2.getStrWidth(text);
      u8g2.drawStr(posX, 11+h*12, text);
    }
  }
  u8g2.sendBuffer();
}

void change_presets() {
  
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(16 ,15, "Rotate to change");

  u8g2.setFont(u8g2_font_fub20_tr);
  // const char* text = "780 g";
  char text[8];
  snprintf(text, sizeof(text), "%d g", input_weight);

  
  int textWidth = u8g2.getStrWidth(text);
  int xPos = 106 - textWidth;
  u8g2.drawStr(xPos, 42, text);

  u8g2.setFont(u8g2_font_5x8_tr);
  u8g2.drawStr(32 ,58, "click to save");

  u8g2.sendBuffer();

}

void calibrate()
{
  //start calibration
  if (calibration == 0) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(14 ,15, "Remove all weight");
    u8g2.drawStr(14 ,25, "from the loadcell.");
    u8g2.drawStr(14 ,50, "press to continue");
    u8g2.sendBuffer();

    scale.power_up();
    Serial.println("\n\nCALIBRATION\n===========");
    Serial.println("remove all weight from the loadcell");
    Serial.println("and press button\n");
    calibration = 1;
  }
  //wait for button press
  else if (calibration == 2) {
    Serial.println("Determine zero weight offset");
    scale.tare(5);
    offset = scale.get_offset();

    Serial.print("OFFSET: ");
    Serial.println(offset);
    Serial.println();
    Serial.println("place a weight on the loadcell");
    Serial.println("enter the weight in (whole) grams and press enter");
    input_weight = 0;
    calibration = 3;
  }
  else if (calibration == 3) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(25 ,15, "Place a weight");
    u8g2.drawStr(20 ,25, "on the loadcell");
    u8g2.drawStr(10 ,35, "and set the weight:");
    char text[8];
    snprintf(text, sizeof(text), "%d g", input_weight);
    u_int16_t xPos = 80 - u8g2.getStrWidth(text);
    u8g2.drawStr(xPos ,45, text);
    u8g2.drawStr(14 ,55, "press to continue");
    u8g2.sendBuffer();
  }
  //wait for weight input (button press)
  else if (calibration == 4) {
    Serial.print("WEIGHT: ");
    Serial.println(input_weight);
    
    scale.calibrate_scale(input_weight, 5);
    float myscale = scale.get_scale();

    Serial.print("SCALE:  ");
    Serial.println(myscale, 6);

    Serial.print("\nuse scale.set_offset(");
    Serial.print(offset);
    Serial.print("); and scale.set_scale(");
    Serial.print(myscale, 6);
    Serial.print(");\n");
    Serial.println("in the setup of your project");

    Serial.println("\n\n");
    scale.power_down();
    calibration = 0;
    menu_index = 1;
  }
}

// confirmation tab
void confirm(int input) {
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(25, 15, "Are you sure?");


  char text[8];
  snprintf(text, sizeof(text), "%d g", input_weight);

  int textWidth = u8g2.getStrWidth(text);
  int xPos = 80 - textWidth;
  u8g2.drawStr(xPos, 32, text);

  u8g2.drawStr(25, 50, "Yes");

  u8g2.drawStr(90, 50, "No");

  u8g2.drawFrame(20+62*choice, 40, 27, 13);

  u8g2.sendBuffer();
}

void fill_honey() {
  if (filling == 0) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(26, 15, "Please remove");
    u8g2.drawStr(33, 25, "any weight");
    u8g2.drawStr(15, 50, "press to continue");
    u8g2.sendBuffer();
  }
  else if (filling == 1) {
    delay(200);
    scale.tare(5);
    filling = 2;
  }
  else if (filling == 2) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(19, 15, "To start filling");
    u8g2.drawStr(10, 25, "place the container");
    u8g2.drawStr(30, 35, "on the scale");

    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(32, 50, "press to exit");

    u8g2.sendBuffer();

    if (scale.is_ready()) {
    scale.set_average_mode();
    weight = scale.get_units(3);
    if (weight > 10000) {
      weight = 0;
    }
    Serial.println(weight);
    if (weight > 100)
    {
      delay(1000);
      scale.tare(5);
      filling = 3;
    }
    }
  }
  if (filling == 3) {
    int16_t actual_weight = scale.get_units(2);
    if (actual_weight < 0) {
      actual_weight = 0;
    }
    uint8_t fill_percentage = (actual_weight*100)/input_weight;

    if (fill_percentage > 100) {
      fill_percentage = 100;
    }

    if (fill_percentage < 80) {
      Serial.println("servo at 100 %");
    }
    else if (fill_percentage < 90) {
      Serial.println("servo at 35 %");
    }
    else if (fill_percentage < 100) {
      Serial.println("servo at 10 %");
    }
    else {
      Serial.println("servo at 0 %");
    }

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(44, 15, "Filling");
    char text[8];
    snprintf(text, sizeof(text), "%d g", actual_weight);
    u_int8_t xPos = 78 - u8g2.getStrWidth(text);
    u8g2.drawStr(xPos, 30, text);

    u8g2.drawFrame(13, 40, 104, 10);
    u8g2.drawBox(15, 42, fill_percentage, 6);


    u8g2.setFont(u8g2_font_5x8_tr);
    snprintf(text, sizeof(text), "%d %%", fill_percentage);
    xPos = 75 - u8g2.getStrWidth(text);
    u8g2.drawStr(xPos, 58, text);

    u8g2.sendBuffer();
    if (fill_percentage >= 100) {
      delay(1000);
      filling = 4;
    }
  }
  if (filling == 4) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(18, 15, "Filling complete");
    u8g2.drawStr(12, 50, "take the container");
    u8g2.sendBuffer();

    if (scale.get_units(2) < 50) {
      delay(500);
      filling = 1;
    }
  }
}

void setup() {
  //encoder setup
  pinMode(outputA,INPUT_PULLUP);
  pinMode(outputB,INPUT_PULLUP);
  pinMode(outputSwitch, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(outputA), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), read_encoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputSwitch), switch_encoder, FALLING);

  Serial.begin(9600);
  // aLastState = digitalRead(35);
  // xTaskCreatePinnedToCore(rotary_encoder_driver, "Handle encoder", 4096, NULL, 1, NULL, 1);

  u8g2.begin();
  u8g2.clearBuffer();
  
  u8g2.drawXBMP(36,3,55,58,epd_bitmap_logo_inv);

  u8g2.sendBuffer();
  oldTime = millis();

  pref.begin("storage", false);  // Namespace: "storage"

  preset1 = pref.getInt("preset1", 1000);
  preset2 = pref.getInt("preset2", 700);
  preset3 = pref.getInt("preset3", 350);

  Serial.println(preset1);
  Serial.println(preset2);
  Serial.println(preset3);

  pref.end();

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  Serial.print("Calibration: ");
  Serial.println(scale.get_scale());
  scale.set_scale(0.420000);
  scale.power_down();

  delay(3000);

}
void loop() {
  switch(menu_index) {
    case 0:
      home();
      break;
    case 1:
      main_menu();
      break;
    case 2:
      honey_fill_menu();
      break;
    case 3:
      honey_fill_custom();
      break;
    case 4:
      honey_preset_menu();
      break;
    case 5:
      change_presets();
      break;
    case 6:
      change_presets();
      break;
    case 7:
      change_presets();
      break;
    case 8:
      calibrate();
      break;
    case 9:
      Serial.println(menu_index);
      menu_index = 1;
      break;
    case 10:
      confirm(input_weight);
      break;
    case 11:
      fill_honey();
      break;
  }
}
