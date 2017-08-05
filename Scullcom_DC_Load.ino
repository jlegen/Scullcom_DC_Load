//SCULLCOM HOBBY ELECTRONICS
//ELECTRONIC DC LOAD PROJECT
//based on Software Version 27 from
//13th July 2017

#include <SPI.h>                              //include SPI library (Serial Peripheral Interface)
#include <Wire.h>                             //include I2C library

#ifdef USE_TFT                                // not implemented yet
 #include "Adafruit_GFX.h"
 #include "Adafruit_ILI9341.h"                // use 128x160 1.4'' SPI TFT from ebay - check if the TFT is 5V, or if you need level converters
#else
 #include <LiquidCrystal_I2C.h>                // F Malpartida's NewLiquidCrystal library
#endif
                                              // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/NewliquidCrystal_1.3.4.zip
#include <math.h>                             //
#include <Adafruit_MCP4725.h>                 //Adafruit DAC library  https://github.com/adafruit/Adafruit_MCP4725
#include <MCP342x.h>                          //Steve Marple library avaiable from    https://github.com/stevemarple/MCP342x
#include <MCP79410_Timer.h>                   //Scullcom Hobby Electronics library  http://www.scullcom.com/MCP79410Timer-master.zip

#include "Config.h"                           //additional configurations

const byte ROWS = 4;                          //four rows
const byte COLS = 4;                          //four columns

#ifdef USE_I2C_KEYPAD
 #include <Keypad_I2C.h>
 byte rowPins[ROWS] = {4, 5, 6, 7}; //connect to the row pinouts of the keypad
 byte colPins[COLS] = {0, 1, 2, 3}; //connect to the column pinouts of the keypad
#else
 byte rowPins[ROWS] = {9, 10, 11, 12}; //connect to the row pinouts of the keypad
 byte colPins[COLS] = {5, 6, 7, 8};    //connect to the column pinouts of the keypad
#endif
#include <Keypad.h>                           //http://playground.arduino.cc/Code/Keypad

//define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

//initialize an instance of class NewKeypad
#ifdef USE_I2C_KEYPAD
 Keypad_I2C customKeypad ( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS, KEYPAD_ADDRESS, PCF8574 );
#else
 Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);
#endif

char customKey;
char decimalPoint;                            //used to test for more than one press of * key (decimal point)

Adafruit_MCP4725 dac;                         //constructor

MCP342x adc = MCP342x(MCP342x_ADDRESS);

MCP79410_Timer timer = MCP79410_Timer(MCP79410_ADDRESS);

#ifdef USE_TFT
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
 Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
#else
 //Set the pins on the I2C chip used for LCD connections
 //ADDR,EN,R/W,RS,D4,D5,D6,D7
 LiquidCrystal_I2C lcd(LCD_ADDRESS,2,1,0,4,5,6,7);    
#endif

int temp;                                     //
int tempCutOff = 60;                          // was 60
int tempMin = 30;                             //temperature at which to start the fan
int tempMax = 75;                             //maximum temperature when fan speed at 100%
int fanSpeed;

float BatteryLife = 0;                        //
float BatteryLifePrevious = 0;                //
float Seconds = 0;                            //
float BatteryCutoffVolts;                     //used to set battery discharge cut-off voltage
float MaxBatteryCurrent = 1.0;                //maximum battery current allowed for Battery Capacity Testing

int stopSeconds;                              //store for seconds when timer stopped

int CP = 8;                                   //cursor start position

boolean toggle = false;                       //used for toggle of Load On/Off button

unsigned long controlVoltage = 0;             //used for DAC to control MOSFET

long current = 0;                             //variable used by ADC for measuring the current
long voltage = 0;                             //variable used by ADC for measuring the voltage

float reading = 0;                            //variable for Rotary Encoder value divided by 1000

float setCurrent = 0;                         //variable used for the set current of the load
float setPower = 0;                           //variable used for the set power of the load
float setResistance = 0;                      //variable used for the set resistance of the load
float setCurrentCalibrationFactor = 1;        //calibration adjustment - set as required (was 0.997)

float voltageOffset = 0;                      //variable to store voltage reading zero offset adjustment at switch on
float currentOffset = 0;                      //variable to store current reading zero offset adjustment at switch on

float setControlCurrent = 0;                  //variable used to set the temporary store for control current required

int VoltsDecimalPlaces = 3;                   //number of decimal places used for Voltage display on LCD

float ActualVoltage = 0;                      //variable used for Actual Voltage reading of Load
float ActualCurrent = 0;                      //variable used for Actual Current reading of Load
float ActualPower = 0;                        //variable used for Actual Power reading of Load

float PowerCutOff = 50;                       //maximum Power allowed in Watts - then limited to this level CAN BE CHANGED AS REQUIRED
float CurrentCutOff = 4;                      //maximum Current setting allowed in Amps - then limited to this level (was 5)
float ResistorCutOff = 999;                   //maximum Resistor we want to deal with in software
float BatteryCurrent;                         //
float LoadCurrent;                            //

int setReading = 0;                           //

int ControlVolts = 0;                         //used to set output current
float OutputVoltage = 0;                      //

String Mode ="  ";                            //used to identify which mode

int modeSelected = 0;                         //Mode status flag

int lastCount = 50;                           //
volatile float encoderPosition = 0;           //
volatile unsigned long factor= 0;             //number of steps to jump
volatile unsigned long encoderMax = 999000;   //sets maximum Rotary Encoder value allowed CAN BE CHANGED AS REQUIRED (was 50000)

float LiPoCutOffVoltage = 3.0;
float LiFeCutOffVoltage = 2.8;
float NiCdCutOffVoltage = 1.0;
float ZiZnCutOffVoltage = 1.0;
float PbAcCutOffVoltage = 1.75;
String BatteryType ="    ";

byte exitMode = 0;      //used to exit battery selection menu and return to CC Mode

char numbers[20];     // keypad number entry - Plenty to store a representation of a float
byte index = 0;
int z = 0;
float x = 0;
int y = 0;
int r = 0;

float LowCurrent = 0;               //the low current setting
float HighCurrent = 0;              //the high current setting
unsigned long transientPeriod;      //used to store pulse time period 
unsigned long current_time;         //used to store the current time in microseconds
unsigned long last_time;            //used to store the time of the last trasient switch in micro seconds
boolean transient_mode_status;      //used to maintain the state of the trasient mode (false = low current, true = high current)

//--------------------------------Interrupt Routine for Rotary Encoder------------------------
void isr()
{
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if (interruptTime - lastInterruptTime > 5) {  //
    if (digitalRead(PIN_ROTB) == LOW)
    {
      encoderPosition = encoderPosition - factor;
    }else{
      encoderPosition = encoderPosition + factor;
    }
    encoderPosition = min(encoderMax, max(0, encoderPosition));  // sets maximum range of rotary encoder
    lastInterruptTime = interruptTime;
  }
    }
//---------------------------------Initial Set up---------------------------------------
void setup() {
#ifdef DEBUG
  Serial.begin(9600);                                      //used for testing only
  Serial.print("DC Load V");
  Serial.println(VERSION);
#endif

  Wire.begin();                                            //join i2c bus (address optional for master)
  Wire.setClock(400000L);                                  //sets bit rate to 400KHz

  customKeypad.begin( makeKeymap(hexaKeys) );
  
  MCP342x::generalCallReset();                             // Reset devices
  delay(1);                                                //MC342x needs 300us to settle, wait 1ms - (may not be required)

  pinMode (PIN_ROTA, INPUT);
  pinMode (PIN_ROTB, INPUT);

  pinMode (PIN_ROTSW, INPUT_PULLUP);
  pinMode (PIN_ONOFF, INPUT_PULLUP);
 
  pinMode (PIN_TRIG, INPUT_PULLUP);

  pinMode (PIN_FAN, OUTPUT);
  TCCR2B = (TCCR2B & 0b11111000) | 1;                      //change PWM to above hearing (Kenneth Larvsen recommendation)
  pinMode (PIN_TEMP, INPUT);
 
  analogReference(INTERNAL);                               //use Arduino internal reference for tempurature monitoring

  attachInterrupt(digitalPinToInterrupt(PIN_ROTA), isr, LOW);

  dac.begin(0x61);                                         //the DAC I2C address with MCP4725 pin A0 set high
  dac.setVoltage(0,false);                                 //reset DAC to zero for no output current set at Switch On

  lcd.begin(20, 4);                                        //set up the LCD's number of columns and rows 
  lcd.setBacklightPin(3,POSITIVE);                         // BL, BL_POL
  lcd.setBacklight(HIGH);                                  //set LCD backlight on

  lcd.clear();                                             //clear LCD display
  lcd.setCursor(6,0);                                      //set LCD cursor to column 0, row 4
  FPL(SPLASH1);
  lcd.setCursor(1,1);                                      //set LCD cursor to column 0, row 1 (start of second line)
  FPL(SPLASH2);
  lcd.setCursor(1,2);
  FPL(SPLASH3);
  lcd.setCursor(0,3);
  FPL(SPLASH4);
  lcd.print(VERSION); //
  delay(3000);                                             //3000 mSec delay for intro display
  lcd.clear();                                             //clear dislay

  last_time = 0;                                           //set the last_time to 0 at the start (Transicent Mode)
  transient_mode_status = false;                           //set the initial transient mode status (false = low, true = high);
  setCurrent = LowCurrent;                                 //first set the current to the low current value (Transicent Mode)
 
  lcd.setCursor(9,0);
  FPL(OFF);                                        //indicate that LOAD is off at start up
  Current();                                               //sets initial mode to be CC (Constant Current) at Power Up
  
  customKey = customKeypad.getKey();
  
}

//------------------------------------------Main Program Loop---------------------------------
void loop() {
  readKeypadInput();                                     //read Keypad entry
  LoadSwitch();                                          //Load on/off

  transient();                                           //test for Transient Mode
  
  lcd.setCursor(18,3);                                   //sets display of Mode indicator at bottom right of LCD
  lcd.print(Mode);                                       //display mode selected on LCD (CC, CP, CR or BC)

  if(Mode != "TC" && Mode != "TP" && Mode != "TT"){      //if NOT transient mode then Normal Operation
  reading = encoderPosition/1000;                        //read input from rotary encoder 
  maxConstantCurrentSetting();                           //set maxiumum Current allowed in Constant Current Mode (CC)
  powerLevelCutOff();                                    //Check if Power Limit has been exceeded
  
  temperatureCutOff();                                   //check if Maximum Temperature is exceeded
  
  batteryCurrentLimitValue();                            //Battery Discharge Constant Current Limit Value in BC Mode
  displayEncoderReading();                               //display rotary encoder input reading on LCD
  //delay(10);                                           //used to test - may not be required
  lastCount = encoderPosition;                           //store rotary encoder current position
  CursorPosition();                                      //check and change the cursor position if cursor button pressed
    }else{
  transientLoadToggle();                                  //Start Transient Mode
  #ifdef DEBUG
   Serial.print("Current: ");                            //used for testing only
   Serial.println(setCurrent);                           //used for testing only
  #endif
    }

  readVoltageCurrent();                                  //routine for ADC's to read actual Voltage and Current
  ActualReading();                                       //Display actual Voltage, Current readings and Actual Wattage
  
  dacControl();
  dacControlVoltage();                                   //sets the drive voltage to control the MOSFET

  batteryCapacity();                                     //test if Battery Capacity (BC) mode is selected - if so action
  fanControl();                                          //call heatsink fan control
}

//------------------------------------------------Read Keypad Input-----------------------------------------------------
void readKeypadInput (void) {
  customKey = customKeypad.getKey();
  
#ifdef DEBUG
  if (customKey != NO_KEY){                             //only used for testing keypad
   Serial.print("customKey = ");                          //only used for testing keypad
   Serial.println(customKey);                             //only used for testing keypad
  }                                                     //only used for testing keypad
#endif

if(customKey == '#' && digitalRead(PIN_ROTSW) == LOW){    //check if Zero Offset Selected (press * and Cursor Button together){
  toggle = false;                                         //switch Load OFF
  zeroOffset();
}

if(customKey == '*' && digitalRead(PIN_ROTSW) == LOW){    //check if Set-Up Mode Selected (press * and Cursor Button together)
  delay(200);
  toggle = false;                                         //switch Load OFF
  userSetUp();
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 0;
  decimalPoint = (' ');                                   //clear decimal point text character reset
}

if(customKey == 'A' && digitalRead(PIN_ROTSW) == LOW){    //check if Transient Mode Selected (press A and Cursor Button together)
  toggle = false;                                         //switch Load OFF
  transientType();
 
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 0;
  decimalPoint = (' ');                                   //clear decimal point text character reset
  }
               
  if(customKey == 'A'){                                   //check if Constant Current button pressed
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(9,0);
  FPL(OFF);
  Current();                                              //if selected go to Constant Current Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 0;
  decimalPoint = (' ');                                   //clear decimal point test character reset
  }
         
  if(customKey == 'B'){                                   //check if Constant Power button pressed
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(9,0);
  FPL(OFF); 
  Power();                                                //if selected go to Constant Power Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 0;
  decimalPoint = (' ');                                   //clear decimal point test character reset
  }
          
  if(customKey == 'C'){                                   //check if Constant Resistance button pressed  
  toggle = false;                                         //switch Load OFF
  lcd.setCursor(9,0);
  FPL(OFF);  
  Resistance();                                           //if selected go to Constant Resistance Selected routine
  encoderPosition = 0;                                    //reset encoder reading to zero
  index = 0;
  z = 0;
  decimalPoint = (' ');                                   //clear decimal point test character reset
  }

  if(customKey == 'D'){                                   //check if Battery Capacity button pressed
  dac.setVoltage(0,false);                                //Ensures Load is OFF - sets DAC output voltage to 0
  toggle = false;                                         //switch Load OFF
  batteryType();                                          //select battery type
  index = 0;
  z = 0;
  decimalPoint = (' ');                                   //clear decimal point test character reset

    if (exitMode == 1){                                   //if NO battery type selected revert to CC Mode
    lcd.setCursor(9,0);
    FPL(OFF);
    Current();                                            //if selected go to Constant Current Selected routine
    encoderPosition = 0;                                  //reset encoder reading to zero
    customKey = 'A';
    }
    else
    {
    lcd.setCursor(16,2);
    lcd.print(BatteryType);                               //print battery type on LCD 
    lcd.setCursor(9,0);
    FPL(OFF);
    timer.reset();                                        //reset timer
    BatteryLifePrevious = 0;
    BatteryCapacity();                                    //go to Battery Capacity Routine
    }
  }

if (Mode != "BC"){

  if(customKey >= '0' && customKey <= '9' && index < 18){               //check for keypad number input
       numbers[index++] = customKey;
       numbers[index] = '\0';
       lcd.setCursor(z,3);                              
       lcd.print(customKey);                              //show number input on LCD
       z = z+1;
     }
  
  if(customKey == '*'){                                   //check if decimal button key pressed
      if (decimalPoint != ('*')){                         //test if decimal point entered twice - if so skip 
      numbers[index++] = '.';
      numbers[index] = '\0';
      lcd.setCursor(z,3);
      lcd.print(".");
      z = z+1;
      decimalPoint = ('*');                             //used to indicate decimal point has been input
        }
      }

  if(customKey == '#') {                                //check if Load ON/OFF button pressed
    x = atof(numbers);     
         reading = x;
         encoderPosition = reading*1000;
         index = 0;
         numbers[index] = '\0';
         z = 0;
         lcd.setCursor(0,3);
         lcd.print("        ");
         decimalPoint = (' ');                          //clear decimal point test character reset
          }
    }
  
}

//----------------------Limit Maximum Current Setting-----------------------------------------
void maxConstantCurrentSetting (void) {
  if (Mode == "CC" && reading > CurrentCutOff){           //Limit maximum Current Setting
  reading = CurrentCutOff;
  encoderPosition = (CurrentCutOff * 1000);               //keep encoder position value at maximum Current Limit
  lcd.setCursor(0,3);
  lcd.print("                    ");                      //20 spaces to clear last line of LCD 
  }

  if (Mode == "CP" && reading > PowerCutOff) {             //Limit maximum Current Setting
        reading = PowerCutOff;
        encoderPosition = (PowerCutOff * 1000);            //keep encoder position value at maximum Current Limit
        lcd.setCursor(0,3);
        lcd.print("                    ");                   //20 spaces to clear last line of LCD 
    }

   if (Mode == "CR" && reading > ResistorCutOff ) {             //Limit maximum Current Setting
        reading = ResistorCutOff;
        encoderPosition = (ResistorCutOff * 1000);            //keep encoder position value at maximum Current Limit
        lcd.setCursor(0,3);
        lcd.print("                    ");                   //20 spaces to clear last line of LCD 
    }

}

//----------------------Power Level Cutoff Routine-------------------------------------------
void powerLevelCutOff (void) {
  if (ActualPower  > PowerCutOff){                        //Check if Power Limit has been exceed
  reading = 0;
  encoderPosition = 0; 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  FPL(OVERPOW);
  lcd.setCursor(9,0);
  FPL(OFF);
  toggle = false;                                         //switch Load Off
  }
}

//----------------------Constant Current Limit Value------------------------------------------
void batteryCurrentLimitValue (void) {
  if (Mode == "BC" && reading > MaxBatteryCurrent){
  reading = MaxBatteryCurrent;
  encoderPosition = (MaxBatteryCurrent*1000);            //keep encoder position value at 1000mA
  }
}

//----------------------Display Rotary Encoder Input Reading on LCD---------------------------
void displayEncoderReading (void) {

    lcd.setCursor(8,2);                                      //start position of setting entry

    if ( ( Mode == "CP" || Mode == "CR" ) && reading < 100 ) {
        lcd.print("0");
    }
    
    if (reading < 10) {                                      //add a leading zero to display if reading less than 10
        lcd.print("0"); 
    }

    if ( Mode == "CP" || Mode == "CR" ) {
        lcd.print (reading, 2);                              //show input reading from Rotary Encoder on LCD
    } else {
        lcd.print (reading, 3);
    }
    lcd.setCursor (CP, 2);                                   //sets cursor position
    lcd.cursor();                                            //show cursor on LCD
}

//--------------------------Cursor Position-------------------------------------------------------
//Change the position routine
void CursorPosition(void) {

    // Defaults for two digits before decimal and 3 after
    int unitPosition = 9;

    //Power can be 3 digit before decimal but only 2 decimals
    if ( Mode == "CP" || Mode == "CR" ) {
        unitPosition = 10;        
    }

    if (digitalRead(PIN_ROTSW) == LOW) {
    
        delay(200);                                          //simple key bounce delay  
        CP = CP + 1;
        if (CP == unitPosition + 1 ) {
            CP = CP + 1;
        }
    }
    
    if (CP > 13)  { CP = unitPosition; }                     //No point in turning tens and hundreds
    if (CP == unitPosition +4 ) { factor = 1; }
    if (CP == unitPosition +3 ) { factor = 10; }
    if (CP == unitPosition +2 ) { factor = 100; }
    if (CP == unitPosition )    { factor = 1000; }
}

//-----------------------Read Voltage and Current---------------------------------------------
void readVoltageCurrent (void) {
  
  MCP342x::Config status;
// Initiate a conversion; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain1,                         //"gain1" means we have select the input amp of the ADC to x1
           1000000, voltage, status);
  
// Initiate a conversion; convertAndRead() will wait until it can be read
  adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
           MCP342x::resolution16, MCP342x::gain4,                         //"gain4" means we have select the input amp of the ADC to x4
           1000000, current, status);
  
}

//----------------------Calculate Actual Voltage and Current and display on LCD---------------------
void ActualReading(void) {
  ActualCurrent = ((((current - currentOffset)*2.048)/32767) * 2.5);        //calculate load current
  ActualVoltage = ((((voltage - voltageOffset)*2.048)/32767) * 50.4);       //calculate load voltage upto 100v (was 50)
  ActualPower = ActualVoltage*ActualCurrent;

  if (ActualPower <=0){
    ActualPower = 0;
  }

 if (ActualVoltage <=0.0){                              //added to prevent negative readings on LCD due to error
  ActualVoltage = 0.0;
 }
 if (ActualCurrent <= 0.0){                             //added to prevent negative readings on LCD due to error
  ActualCurrent = 0.0;
 }
  
 lcd.setCursor(0,1);
    
    if ( ActualCurrent < 10.0 ) {
        lcd.print(ActualCurrent,3);
    } else {
        lcd.print(ActualCurrent,2);
    }
    
    lcd.print("A ");
    
    if (ActualVoltage < 10.0) {
        lcd.print(ActualVoltage, 3);
    } else {
        lcd.print(ActualVoltage, 2);
    }    

    lcd.print("V ");
     
    if (ActualPower < 100 ) {
        lcd.print(ActualPower,2);
    } else {
        lcd.print(ActualPower,1);
    }
    lcd.print("W ");
}

//-----------------------DAC Control Voltage for Mosfet---------------------------------------
void dacControlVoltage (void) {
  if (Mode == "CC"){
  setCurrent = reading*1000;                                //set current is equal to input value in Amps
  setReading = setCurrent;                                  //show the set current reading being used
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

  if (Mode == "CP"){
  setPower = reading*1000;                                  //in Watts
  setReading = setPower;
  setCurrent = setPower/ActualVoltage;
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent;                       //
  }

  if (Mode == "CR"){
  setResistance = reading;                                  //in ohms
  setReading = setResistance;
  setCurrent = (ActualVoltage)/setResistance*1000;
  setControlCurrent = setCurrent * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

if (Mode == "TC" || Mode == "TP" || Mode == "TT"){                            //Transient Mode - Continuous
  setControlCurrent = (setCurrent * 1000) * setCurrentCalibrationFactor;
  controlVoltage = setControlCurrent; 
  }

}

//-----------------------Battery Capacity Discharge Routine-----------------------------------
void batteryCapacity (void) {
  if (Mode == "BC"){
    setCurrent = reading*1000;                             //set current is equal to input value in Amps
    setReading = setCurrent;                               //show the set current reading being used
    setControlCurrent = setCurrent * setCurrentCalibrationFactor;
    controlVoltage = setControlCurrent;

    lcd.setCursor(0,3);
    lcd.print (timer.getTime());                           //start clock and print clock time

    Seconds = timer.getTotalSeconds();                     //get totals seconds
  
    LoadCurrent = ActualCurrent;                           //if timer still running use present Actual Current reading
    if (timer.status() == 2){                              //if timer is halted then use last Actual Current reading before timer stopped
      LoadCurrent = BatteryCurrent;
      }
 
    BatteryLife = (LoadCurrent*1000)*(Seconds/3600);       //calculate battery capacity in mAh
    lcd.setCursor(9,3);
    BatteryLife = round(BatteryLife);

    if(BatteryLife >= BatteryLifePrevious){                //only update LCD (mAh) if BatteryLife has increased
  
      if (BatteryLife < 10) {                              //add a 3 leading zero to display if reading less than 10
      lcd.print("000");
      }

      if (BatteryLife >= 10 && BatteryLife <100){          //add a 2 leading zero to display
      lcd.print("00");  
      }

      if (BatteryLife >= 100 && BatteryLife <1000){        //add a 1 leading zero to display
      lcd.print("0"); 
      }
  
    lcd.print(BatteryLife,0);
    lcd.setCursor(13,3);
    lcd.print("mAh");
    BatteryLifePrevious = BatteryLife;                      //update displayed battery capacity on LCD
    } 
  }


  if (Mode == "BC" && ActualVoltage <= BatteryCutoffVolts){ //stops clock if battery reached cutoff level and switch load off

  BatteryCurrent = ActualCurrent;
  dac.setVoltage(0,false);                                  //reset DAC to zero for no output current set at switch on                                             
  toggle = false;                                           //Load is toggled OFF
  lcd.setCursor(9,0);
  FPL(OFF);                                         //indicate that LOAD is off at start up
  timer.stop();
  }
}

//-----------------------Fan Control----------------------------------------------------------
void fanControl (void) {
    temp = analogRead(PIN_TEMP);
    temp = temp * 0.107421875;                           // convert to Celsius

    //Serial.print("Temp = ");                          //used for testing only
    //Serial.println(temp);                             //used for testing only
  

    if (temp < tempMin) {                                    //is temperature lower than really cold always turn off fan
        fanSpeed = 0;
        digitalWrite(PIN_FAN, LOW);                          //then fan turned off
    }

    if (temp >= tempMin && temp < 32 && fanSpeed !=0 ) {      //Hysteresis to avoid fan starting and stopping at 30 deg C
        fanSpeed = 131;
        analogWrite(PIN_FAN, fanSpeed);
    }


    if (temp >= 32  && temp < 40 ) {                     //Below 40 we run fan fixed at minimum
        fanSpeed = 131;
        analogWrite(PIN_FAN, fanSpeed);
    }
    

    if ((temp >= 40) && (temp < 50)){                    //OK we need the fan but let us keep it quiet if we can
        fanSpeed = map(temp, 40, 50, 131, 200);
        analogWrite(PIN_FAN, fanSpeed);
   }

    if ((temp >= 50) && (temp <= tempMax)){
        fanSpeed = map(temp, 50, 75, 131, 255);           //OK we need a jet stream and hearing protection
        analogWrite(PIN_FAN, fanSpeed);
    }

    if (temp > tempMax) {                                     //OK we need fan at full steam. Ready for take off
        fanSpeed = 255;
        digitalWrite(PIN_FAN, HIGH);
    }

    lcd.setCursor(16,0);
    lcd.print(temp);
    lcd.print((char)0xDF);
    lcd.print("C");

    //Serial.print("Fan Speed ");                      //used for testing only
    //Serial.println(fanSpeed);                        //used for testing only
}
  
//-----------------------Toggle Current Load ON or OFF------------------------------
void LoadSwitch(void) {
if (digitalRead(PIN_ONOFF) == LOW) {
    
  delay(200);                                              //simple key bounce delay 
 
    if(toggle)
    {
      lcd.setCursor(9,0);
      FPL(OFF);
      //Load = 0;
      toggle = !toggle;        
    }
    else
    {
      lcd.setCursor(9,0);
      FPL(ON);
      lcd.setCursor(0,3);
      lcd.print("                    ");                 //clear bottom line of LCD
      //Load = 1;
      toggle = !toggle;
    }
  //delay(100);                                          //simple delay for key debounce (commented out if not required)
}
}
//-----------------------Select Constant Current LCD set up--------------------------------
void Current(void) {
  Mode = ("CC");
  lcd.setCursor(0,0);
  FPL(DC_LOAD);  
  lcd.setCursor(0,2);
  FPL(SPACE16);
  lcd.setCursor(0,2);
  FPL(SET_I);
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  FPL(SPACE20);                                         //20 spaces so as to allow for Load ON/OFF to still show
  CP = 9;                                               //sets cursor starting position to units.
}

//----------------------Select Constant Power LCD set up------------------------------------
void Power(void) {
  Mode = ("CP");
  lcd.setCursor(0,0);
  FPL(DC_LOAD);
  lcd.setCursor(0,2);
  FPL(SPACE16);
  lcd.setCursor(0,2);
  FPL(SET_W);
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print("W");
  lcd.setCursor(0,3);                               //clear last line of time info
  FPL(SPACE20);                                     //20 spaces so as to allow for Load ON/OFF to still show
  CP = 10;                                          //sets cursor starting position to units.
}

//----------------------- Select Constant Resistance LCD set up---------------------------------------
void Resistance(void) {
  Mode = ("CR");
  lcd.setCursor(0,0);
  FPL(DC_LOAD);  
  lcd.setCursor(0,2);
  FPL(SPACE16);
  lcd.setCursor(0,2);
  FPL(SET_R);
  lcd.setCursor(16,2);
  lcd.print("    ");
  lcd.setCursor(14,2);
  lcd.print((char)0xF4);
  lcd.setCursor(0,3);                                   //clear last line of time info
  FPL(SPACE20);                    //20 spaces so as to allow for Load ON/OFF to still show
  CP = 10;                                               //sets cursor starting position to units.
}

//----------------------- Select Battery Capacity Testing LCD set up---------------------------------------
void BatteryCapacity(void) {
  Mode = ("BC");
  lcd.setCursor(0,0);
  FPL(BAT);
  lcd.setCursor(0,2);
  FPL(SPACE16);
  lcd.setCursor(0,2);
  FPL(SET_I);
  lcd.setCursor(14,2);
  lcd.print("A");
  lcd.setCursor(0,3);                                   //clear last line of time info
  FPL(SPACE20);                    //20 spaces so as to allow for Load ON/OFF to still show
}

//----------------------Battery Type Selection Routine------------------------------------------------
void batteryType (void) {
  exitMode = 0;                                         //reset EXIT mode
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(0,0);
  FPL(BAT_SEL0);  
  lcd.setCursor(0,1);
  FPL(BAT_M1);
  lcd.setCursor(0,2);
  FPL(BAT_M2);
  lcd.setCursor(0,3);                                   //clear last line of time info
  FPL(BAT_M3);                    

  customKey = customKeypad.waitForKey();                //stop everything till the user press a key.

  if (customKey == '1'){
  BatteryCutoffVolts = LiPoCutOffVoltage;
  BatteryType = ("LiPo");
    }

  if (customKey == '2'){
  BatteryCutoffVolts = LiFeCutOffVoltage;
  BatteryType = ("LiFe");
    }

  if (customKey == '3'){
  BatteryCutoffVolts = NiCdCutOffVoltage;
  BatteryType = ("NiCd");  
    }

  if (customKey == '4'){
  BatteryCutoffVolts = ZiZnCutOffVoltage;
  BatteryType = ("ZiZn"); 
    }

  if (customKey == '5'){ 
  BatteryType = ("SetV");
    }

  if (customKey == '6'){                                  //Exit selection screen
  exitMode = 1;
    }

  if (customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == 'A' || customKey == 'B' || customKey == 'C' || customKey == 'D' || customKey == '*' || customKey == '#'){
  batteryType();                                                      //ignore other keys
    }

  if(BatteryType == "SetV" && exitMode != 1){
  setBatteryCutOff();
    }

  batteryTypeSelected();                                    //briefly display battery type selected and discharge cut off voltage

  lcd.clear();

}

//--------------------------Zero Setting Offset Routine--------------------------------------------
void zeroOffset (void) {

  delay(200);                                            //simple key bounce delay 
  readVoltageCurrent();                                  //routine for ADC to read actual Voltage and Current
  voltageOffset = voltage;
  currentOffset = current;

#ifdef DEBUG
 Serial.print("voltageOffset = ");                    //used for testing only
 Serial.println(voltageOffset);                       //used for testing only
 Serial.print("currentOffset = ");                    //used for testing only
 Serial.println(currentOffset);                       //used for testing only
#endif
}
//--------------------------Set DAC Voltage--------------------------------------------
void dacControl (void) {
  if (!toggle){
  dac.setVoltage(0,false);                                 //set DAC output voltage to 0 if Load Off selected
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer.status() == 1){
    timer.stop();
    }
  
  }else{
  //Serial.println("Control Voltage");                    //used for testing only
  //Serial.println(controlVoltage);                       //used for testing only
  dac.setVoltage(controlVoltage,false);                   //set DAC output voltage for Range selected
    if(Mode == "BC" && ActualVoltage >= BatteryCutoffVolts && timer.status() != 1){
    timer.start();
    }
  }
}

//--------------------------Battery Selected Information--------------------------------------------
void batteryTypeSelected (void) {
  if (exitMode !=1){                                      //if battery selection was EXIT then skip this routine
  lcd.clear();
  lcd.setCursor(2,0);
  FPL(BAT_SEL1);
  lcd.setCursor(8,1);
  lcd.print(BatteryType);                                 //display battery type selected
  lcd.setCursor(2,2);
  FPL(BAT_SEL2);
  lcd.setCursor(6,3);
  lcd.print(BatteryCutoffVolts);                          //display battery discharge cut off voltage
  FPL(VOLT);                                    
  delay(3000);
  }
}

//--------------------------Set Battery Cut-Off Voltage--------------------------------------------
void setBatteryCutOff (void) {

  lcd.clear();
  lcd.setCursor(1,0);
  FPL(BAT_CUT1);
  lcd.setCursor(1,1);
  FPL(BAT_CUT2);
  y = 8;
  z = 8;
  r = 2;

  inputValue();
  BatteryCutoffVolts = x;

  lcd.clear();
}

//------------------------Key input used for Battery Cut-Off and Transient Mode------------------------
void inputValue (void){

 while(customKey != '#'){
  
  customKey = customKeypad.getKey();
  if(customKey >= '0' && customKey <= '9'){               //check for keypad number input
       numbers[index++] = customKey;
       numbers[index] = '\0';
       lcd.setCursor(z,r);                              
       lcd.print(customKey);                              //show number input on LCD
       z = z+1;
     }
  
  if(customKey == '*'){                                   //check if ZERO READING key pressed
      if (decimalPoint != ('*')){                         //test if decimal point entered twice - if so ski
      numbers[index++] = '.';
      numbers[index] = '\0';
      lcd.setCursor(z,r);
      lcd.print(".");
      z = z+1;
      decimalPoint = ('*');                               //used to indicate decimal point has been input
        }
      }

 if(customKey == 'C'){                                    //clear entry
    index = 0;
    z = y;
    lcd.setCursor(y,r);
    lcd.print("     ");
    numbers[index] = '\0';                                //
    decimalPoint = (' ');                                 //clear decimal point test character reset
  }

 }
 
  if(customKey == '#') {                                  //check if Load ON/OFF button pressed
    x = atof(numbers);     
    index = 0;
    numbers[index] = '\0';
    decimalPoint = (' ');                                 //clear decimal point test character reset
  }
}

//----------------------------------------Transient Mode--------------------------------------------
void transientMode (void) {

  y = 11;
  z = 11;
  
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(3,0);
  FPL(MODE_TRANS);  
  lcd.setCursor(0,1);
  FPL(SET_LI);
  lcd.setCursor(19,1);
  lcd.print("A");
  r = 1;
  inputValue();
  
  if(x >= CurrentCutOff){
    LowCurrent = CurrentCutOff;
  }else{
    LowCurrent = x;
  }
  lcd.setCursor(11,r);
  lcd.print(LowCurrent,3);
  
  customKey = '0';

  z = 11;

  lcd.setCursor(0,2);
  FPL(SET_HI);
  lcd.setCursor(19,2);
  lcd.print("A");
  r = 2;
  inputValue();
  if(x >= CurrentCutOff){
    HighCurrent = CurrentCutOff;
  }else{
    HighCurrent = x;
  }
  lcd.setCursor(11,r);
  lcd.print(HighCurrent,3);

  customKey = '0';

  if(Mode == "TC" || Mode == "TP"){
  z = 11;

  lcd.setCursor(0,3);
  FPL(SET_TIME);
  lcd.setCursor(16,3);
  FPL(MSEC);
  r = 3;
  inputValue();
  transientPeriod = x;
  lcd.setCursor(11,r);
  lcd.print(transientPeriod);
  }else{
  lcd.setCursor(0,3);
  lcd.print("                    ");
  }

lcd.clear();

toggle = false;                                           //switch Load OFF
lcd.setCursor(9,0);
FPL(OFF);                                         //print on display OFF

}

//----------------------------------------Transient Type Selection--------------------------------------------
void transientType (void) {
  toggle = false;                                         //switch Load OFF
  exitMode = 0;                                           //reset EXIT mode
  lcd.noCursor();                                         //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(3,0);
  FPL(MODE_TRANS);  
  lcd.setCursor(0,1);
  FPL(TR_1);
  lcd.setCursor(0,2);
  FPL(TR_2);
  lcd.setCursor(11,2);                                    //
  FPL(TR_3);
  lcd.setCursor(0,3);                                     //
  FPL(TR_4);

  customKey = customKeypad.waitForKey();                  //stop everything till the user press a key.

  if (customKey == '1'){
  Mode = ("TC"); 
    }

  if (customKey == '2'){
   Mode = ("TT");
    }

  if (customKey == '3'){
  Mode = ("TP");  
    }

  if (customKey == '4'){                                  //Exit selection screen
  exitMode = 1;
    }

  if (customKey == '5' || customKey == '6' || customKey == '7' || customKey == '8' || customKey == '9' || customKey == '0' || customKey == 'A' || customKey == 'B' || customKey == 'C' || customKey == 'D' || customKey == '*' || customKey == '#'){
  transientType();                                                      //ignore other keys
  
    }
lcd.clear();

if (exitMode == 1){                                       //if NO Transient Mode type selected revert to CC Mode
    lcd.setCursor(9,0);
    FPL(OFF);
    Current();                                            //if selected go to Constant Current Selected routine
    encoderPosition = 0;                                  //reset encoder reading to zero
    customKey = 'A';
      }else{
    transientMode();
      }
 }
 
//----------------------------------------Transient--------------------------------------------
void transient (void) {
  if(Mode == "TC" || Mode == "TP" || Mode == "TT"){
  lcd.noCursor();                                         //switch Cursor OFF for this menu 
  lcd.setCursor(0,0);
  FPL(DC_LOAD);
  lcd.setCursor(0,2);
  lcd.print("Lo=");
  lcd.setCursor(3,2);
  lcd.print(LowCurrent,3);
  lcd.setCursor(8,2);
  lcd.print("A");
  lcd.setCursor(11,2);
  lcd.print("Hi=");
  lcd.setCursor(14,2);                                    //
  lcd.print(HighCurrent,3);
  lcd.setCursor(19,2);
  lcd.print("A");

  if(Mode == "TC" || Mode == "TP"){
  lcd.setCursor(0,3);                                     //
  FPL(TIME_IS);
  lcd.setCursor(7,3);
  lcd.print(transientPeriod);
  lcd.setCursor(12,3);                                    //
  FPL(MSEC);
    }else{
     lcd.setCursor(0,3);
     lcd.print("  ");
    }
  }
delay(1);
}

//-------------------------------------Transcient Load Toggel-------------------------------------------
void transientLoadToggle(){

  if(Mode == "TC"){
  current_time = micros();                              //get the current time in micro seconds()
  if (last_time == 0){
    last_time = current_time;
  } else {
    switch (transient_mode_status){
      case (false):
          // we are in the low current setting
          if ((current_time - last_time) >= (transientPeriod * 1000.0)){
             transientSwitch(LowCurrent, true);  
          }
        break;
      case (true):
          // we are in the high current setting 
          if ((current_time - last_time) >= (transientPeriod * 1000.0)){
            transientSwitch(HighCurrent, true);            
          }
        break; 
      } 
     }
    }


  if(Mode == "TP"){
    current_time = micros();                            //get the current time in micro seconds()
    if (last_time == 0){
        last_time = current_time;
        transientSwitch(LowCurrent, true);
    }
    if (digitalRead(PIN_TRIG) == LOW){
      // a trigger pluse is received
      // set to the high current
      transientSwitch(HighCurrent, true); 
    } else {
        if ((current_time - last_time) >= (transientPeriod * 1000.0)){
            transientSwitch(LowCurrent, true);            
        }
      }  
  }


 if(Mode == "TT"){
// this function will toggle between high and low current when the trigger pin is taken low
  if (digitalRead(PIN_TRIG) == LOW){
    switch (transient_mode_status){
      case (false):
        transientSwitch(LowCurrent, true); 
        break;
      case (true):
        transientSwitch(HighCurrent, true);   
        break;
    }
  }
}

}

//-------------------------------------Transcient Switch-------------------------------------------
void transientSwitch(float current_setting, boolean toggle_status){
  if (toggle_status){
  transient_mode_status = !transient_mode_status;
  }
  setCurrent = current_setting;
  last_time = current_time;
}

//-------------------------------------User set up-------------------------------------------------
void userSetUp (void) {
y = 14;
z = 14;
  
  lcd.noCursor();                                       //switch Cursor OFF for this menu               
  lcd.clear();
  lcd.setCursor(4,0);
  FPL(USER_SET);
  lcd.setCursor(0,1);
  FPL(CURR_LIMIT);
  lcd.setCursor(19,1);
  lcd.print("A");
  r = 1;
  inputValue();
  CurrentCutOff = x;
  lcd.setCursor(14,r);
  lcd.print(CurrentCutOff,3);
  
  customKey = '0';

  z = 14;

  lcd.setCursor(0,2);
  FPL(POWER_LIMIT);
  lcd.setCursor(19,2);
  lcd.print("W");
  r = 2;
  inputValue();
  PowerCutOff = x;
  lcd.setCursor(14,r);
  lcd.print(PowerCutOff,2);

customKey = '0';

z = 14;

  lcd.setCursor(0,3);
  FPL(TEMP_IS);
  lcd.setCursor(18,3);
  lcd.print((char)0xDF);
  lcd.print("C");
  r = 3;
  inputValue();
  tempCutOff = x;
  lcd.setCursor(14,r);
  lcd.print(tempCutOff);

    //delay(500);                                             //used in testing only
    lcd.clear();

    lcd.setCursor(9,0);
    FPL(OFF);
    Current();                                            //if selected go to Constant Current Selected routine
    encoderPosition = 0;                                  //reset encoder reading to zero
    customKey = 'A';
}

//------------------------------------------High Temperature Cut-Off--------------------------------------------------------------
void temperatureCutOff (void){
  if (temp >= tempCutOff){                                 //if Maximum temperature is exceeded
  reading = 0;
  encoderPosition = 0; 
  lcd.setCursor(0,3);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  FPL(OVERTEMP);
  lcd.setCursor(9,0);
  FPL(OFF);
  toggle = false;                                         //switch Load Off
  }
}
//------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------

