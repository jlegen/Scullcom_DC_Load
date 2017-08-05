# Scullcom_DC_Load
This is a modified software for the nice DC-Load project from http://www.scullcom.uk
It also has battery measurements, transient modes and more.

The relevant changes of this software version - based on Scullcom's V27 - are:
* moved strings into flash memory - this reduces RAM usage down to ~ 40%
* as a side effect, it is now easy to add more languages - i added german :-)
* added an I2C PCF8574 expander module for the keypad - this frees up 8 GPIO pins; for this, i added the library
https://github.com/joeyoung/arduino_keypads/tree/master/Keypad_I2C which seemlessly adds to the used kaypad-lib
* moved some parameters and defines to a new file 'config.h'

The keypad simply has to be reconnected to the I2C expander module, which i got from ebay. 
To make use of it, change the define in config.h.

The software should be fully backwards compatible.

My intention is to see how to add a nicer colored TFT to this project - therefore the freed pins... :-)

