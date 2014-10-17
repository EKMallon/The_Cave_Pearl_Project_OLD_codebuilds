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

==========================================================================================

This script outputs a CSV format file like this:

Unit#Drip sensor prototype	 Data platform #sensor0X with pololu power control		Rocket Ultra	ADXL345 (DRIP) 24Hz	 								
The sample interval for this series is: 15 min.  0 Sec.  											
YYYY/MM/DD HH:MM	 RTC Temp(C)	 Vcc(mV)	 Drip Count 	Vcc after Newfile Created														
					ACCx=	5	ACCy=	4	ACCz=	229	10/12/2014 23:45
10/12/2014 9:46	23	4623	0	9999	                    					
10/12/2014 10:00	22	4623	229	9999	                  				
10/12/2014 10:15	22	4623	0	9999	                    				
10/12/2014 10:30	22	4623	9	9999	                    				
10/12/2014 10:45	22	4623	1	9999	                    				
10/12/2014 11:00	22	4623	0	9999	                    				
10/12/2014 11:15	22	4623	0	9999	                    				
10/12/2014 11:30	22	4623	14	9999	                   				
10/12/2014 11:45	22	4623	0	9999	                    				
...etc.

In the first file on the SD card, Vcc after new file created will remain 9999, as a new file has not been created yet. The new file creation event is the biggest hit to the power supply (usually dropping it by 20-50mv), and I am using that as a test of the battery performance. Most likely it will be that event that foces the unit to go into low power sleep.

Once per day the accelerometer is read, but that line of text gets dumped to the sd file a little out of sync with the data written from the eeprom buffer. I have it shifted out of the way, so it should be pretty easy to just sort those entries out of the way in Excel. I just use that information to make sure the unit has not fallen over.

============================================================================================