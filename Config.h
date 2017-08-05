
#define VERSION 28

//#define DEBUG                        // define to enable serial debug output

// use I2C keypad to free up 8 pins
// additional lib: https://github.com/joeyoung/arduino_keypads/tree/master/Keypad_I2C
// (i used a cheap PCF8574 module from ebay)
//#define USE_I2C_KEYPAD
#define KEYPAD_ADDRESS 0x38     //define I2C address of PCF8574 (see other supported chips at github repo above)

// use SPI TFT 128x160 (instead of 2004 I2C LCD) - not implemented yet
//#define USE_TFT
//#define TFT_DC 9   // digital pin for DC
//#define TFT_CS 10  // digital pin for CS

#define LCD_ADDRESS 0x27        //0x27 is the default address of the LCD with I2C bus module
#define MCP79410_ADDRESS 0x6f   //0x6f is the default address for the MCP79410 Real Time Clock IC
#define MCP342x_ADDRESS 0x68    //0x68 is the default address for the MCP3426 device

#define PIN_TEMP A6             //analog pin used for temperature output from LM35 (was A0 previously but changed)
#define PIN_FAN  3              //digital pin 3 for fan control output (changed to Digital pin 3)
#define PIN_ROTA 2              //digital pin (also interrupt pin) for the A pin of the Rotary Encoder (changed to digital pin 2)
#define PIN_ROTB 4              //digital pin for the B pin of the Rotary Encoder
#define PIN_ROTSW A3            //analog pin A3 used as a digital pin to set cursor position (rotary encoder push button)
#define PIN_ONOFF A1            //analog pin A1 used as a digital pin to set Load ON/OFF
#define PIN_TRIG A2             //analog pin A2 used as a digital pin for trigger pulse input in transient mode

// equivalent to F() macro, also works with lcd.print()
#define FS(x) (__FlashStringHelper*)(x)          // return string from flash
#define FPL(x); lcd.print(FS(x));                // print flash string on LCD
#define FT(x,y);  const char x[] PROGMEM = {y};  // helper for string definitions

// define language
//#define GERMAN
#define ENGLISH


#ifdef GERMAN
// umlauts sometimes differ...
//ü = \365
//ä = \341
//ö = \357
//               01234567890123456789
FT(SPLASH1,     "SCULLCOM");
FT(SPLASH2,     "Hobby Electronics");
FT(SPLASH3,     "DC Electronic Load");
FT(SPLASH4,     "Software Version ");
FT(OVERTEMP,    "Temperatur zu hoch!");
FT(OVERPOW,     "Leistung zu hoch!");
FT(OFF,         "AUS");
FT(ON,          "AN ");
FT(TEMP_IS,     "Temperatur   =");
FT(POWER_LIMIT, "Leistung     =");
FT(USER_SET,    "User Setup  ");
FT(SET_TIME,    "Set Time  = ");
FT(SET_HI,      "Set High I=");
FT(SET_LI,      "Set Low  I=");
FT(SET_I,       "Set I = ");
FT(SET_W,       "Set W = ");
FT(SET_R,       "Set R = ");
FT(CURR_LIMIT,  "Strom Begr.=");
FT(DC_LOAD,     "DC LOAD");
FT(TIME_IS,     "Zeit = ");
FT(MODE_TRANS,  "Transient Mode");
FT(TR_1,        "1 = Dauerhaft");
FT(TR_2,        "2 = Toggle");
FT(TR_3,        "3 = Puls");
FT(TR_4,        "4 = Ende");
FT(BAT,         "BATTERIE");
FT(BAT_CUT1,    "Eingabe Batterie-");
FT(BAT_CUT2,    "Abschaltspannung:");
FT(BAT_SEL0,    "Batterietyp w\341hlen");
FT(BAT_SEL1,    "Batterietyp:");
FT(BAT_SEL2,    "Entladungsende:");
FT(BAT_M1,      "1=LiPo/Li-Ion 2=LiFe");
FT(BAT_M2,      "3=NiCd/NiMH   4=ZiZn");
//               01234567890123456789
FT(BAT_M3,      "5=Spannung    6=Ende");
FT(VOLT,        " Volt");
FT(MSEC,        "ms");
FT(SPACE20,     "                    ");
FT(SPACE16,     "                ");
#endif

#ifdef ENGLISH
//               01234567890123456789
FT(SPLASH1,     "SCULLCOM");
FT(SPLASH2,     "Hobby Electronics");
FT(SPLASH3,     "DC Electronic Load");
FT(SPLASH4,     "Software Version ");
FT(OVERTEMP,    "Over Temperature");
FT(OVERPOW,     "Exceeded Power");
FT(OFF,         "OFF");
FT(ON,          "ON ");
FT(TEMP_IS,     "Temperature  =");
FT(POWER_LIMIT, "Power Limit  =");
FT(USER_SET,    "User Set-Up");
FT(SET_TIME,    "Set Time  = ");
FT(SET_HI,      "Set High I=");
FT(SET_LI,      "Set Low  I=");
FT(SET_I,       "Set I = ");
FT(SET_W,       "Set W = ");
FT(SET_R,       "Set R = ");
FT(CURR_LIMIT,  "Current Limit=");
FT(DC_LOAD,     "DC LOAD");
FT(TIME_IS,     "Time = ");
FT(MODE_TRANS,  "Transient Mode");
FT(TR_1,        "1 = Continuous");
FT(TR_2,        "2 = Toggle");
FT(TR_3,        "3 = Pulse");
FT(TR_4,        "4 = Exit");
FT(BAT,         "BATTERY");
FT(BAT_CUT1,    "Enter Battery");
FT(BAT_CUT2,    "Cut-Off Voltage:");
FT(BAT_SEL0,    "Select Battery Type");
FT(BAT_SEL1,    "Battery Selected");
FT(BAT_SEL2,    "Discharge Cutoff");
FT(BAT_M1,      "1=LiPo/Li-Ion 2=LiFe");
FT(BAT_M2,      "3=NiCd/NiMH   4=ZiZn");
FT(BAT_M3,      "5=Set Voltage 6=Exit");
FT(VOLT,        " Volt");
FT(MSEC,        "mSec");
FT(SPACE20,     "                    ");
FT(SPACE16,     "                ");
#endif


