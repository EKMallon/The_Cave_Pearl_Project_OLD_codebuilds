Cave Pearl Datalogger script for DRIP SENSOR by Edward Mallon
	 

This version is configured by default as DRIPSENSOR only, and assumes the following hardware

MCU: Rocket scream ultra -or- Sparkfun Pro Mini -or- Generic 3.3v ProMini Clone
-Voltage divider:  RAW-10MO-A0-(10MO+0.1 µF)-GND  for the readExternalVcc function
-Ds3231 RTC with AT24c32eeprom, Adxl345 I2C accelerometer, 3 color LED on pins 4-6 with a massive 10k limit resistor so I leave the led on during some sleeps.
-Pololu Power Switch LV which will cut the power if you drive pin 9 high, Without the switch the code will simply sleep on errors.
-A raspberry pi uSD card adapter connected to pins 10-13, with 50k pullup on unused connections (see more at the Cave Pearl Project website: http://edwardmallon.wordpress.com/category/drip-sensor-development/ for wiring conections) 

AND Because everything is wrapped in  #ifdef--#endif  statements anyway I have left in the script support for the following sensors even though the drip sensor does not use any of them:
HMC5883 Compass, DS18B20 Temp, TMP102 Temp, BMA180 Acc, BMA250 Acc, Ms5803-02 & 05 Pressure sensors. Simply UNCOMMENT defines at the beginning of the script to enaable each sensor you have connected - the rest of the code should adjust automatically...more or less... You will still have to edit some of the Pstring functions to make sure you are getting the right data saved to your SD cards.

ALSO, I adopted the Rocket Screem Ultra sleep library, to turn off the BOD fuse while sleeping however I left in my older routine for non-RS watch dog time sleeping, as I am still not certain that turning off BOD is a good idea for data loggers.
