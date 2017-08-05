# Scullcom_DC_Load
This is a modified software for the nice DC-Load project from http://www.scullcom.uk
It also has battery measurements, transient modes and more.

The relevant changes of this software version - based on Scullcom's V27 - are:
* moved strings into flash memory - this frees up RAM to a usage of down to 40%
* as a side effect, it is now easy to add more languages - i added german :-)
* added an I2C PCF8574 expander module for the keypad - this frees up 8 GPIO pins 
* moved some parameters and defines to a new file 'config.h'

The keypad simply has to be reconnected to the I2C expander module, which i got from ebay. 
To make use of it, change the define in config.h.

The software should be fully backwards compatible.

My intention is to try this out with a nicer colored TFT - therefore the freed pins... :-)

