A major overhaul of the Cave Pearl software in this release:

----------------------------------------------------------------------------------------------------


Common I2C functions have been now been extracted to their own subroutines and, drawing on the elegant  code over at the multiwii project, I have wrapped large sections with #ifdef/#endif statements, to get the sketch to adapt automatically to whatever combination of different sensors is attached for a given deployment. Simply uncomment the defines at the beginning of the script for each sensor you have connected - the rest of the code should adjust automatically...mostly...still some tweaks needed for the SD card writing.

"#define SamplesPerCycle XX "is limited to 42 if you have the compass or the pressure sensor attached, as that pushes the page writes to 3 per sample cycle, and the AT25C32 eeprom only has 4k of storage. I know I have very inefficient use of the eeprom here, but I wont bother optimizing the data handling till I have done my final sensor selections, as then I will know what the data actually looks like...

----------------------------------------------------------------------------------------------------


Added: scheme to replace power wasting delays (while waiting for high bit depth sensor conversions) with variable watchdog timer sleeps. 

Added: a facility to have sampling intervals less than one minute. I only use this when debugging, so under normal operation make sure that the "#define SampleIntSeconds 20" is commented out, and you have a number greater than 0 for "#define SampleIntervalMinutes 0". Still not happy about the RTC library, and its requirement to load SPI.h just to run without errors, but will find a better lib to use later. I have linked some alternates there in the code.

Added: A smoothing routine for the spiky accelerometer data based on Paul Badger's Digital Smooth http://playground.arduino.cc/Main/DigitalSmooth - but keep a close eye on the freemem if you push "#define filterSamples 7" much beyond 7 you could have problems, especially if you are trying to run several other sensors at the same time.

Added: a voltage check AFTER the SD card saves. Since the SD is the largest current drawing event, I figured that checking for unsafe voltage events then made sense. Low volt errors now go to permanent sleep state, rather than a while(1) loop. 


----------------------------------------------------------------------------------------------------


Electronic components for the current datalogger platform:
**********************************************************
TinyDuino Processor board (unregulated), TinyShield Proto, TinySheild microSD, 
TinyShield Ambient Light sensor (hacked) to use vregulated I2C lines, DS3232 Rtc, AT24C32 I2C eeprom


Sensors support included in this build:
***************************************
HMC5883 Compass, DS18B20 Temp, TMP102 Temp, BMA180 Acc, BMA250 Acc, Ms5803-02 & 05 Pressure sensors* 

*Note: I was using a derivative of the OPENROV code by Walt Holm to drive the MS5803's, but Luke Miller spotted that the float math was generating errors. I have mangled his libs here, because with the rest of the parts bolted together here I don't have enough ram to load even one of his libs directly, and just barely have enough ram for the first order calculations for debugging. So I am only recording the D1 & D2 values in the logged data.

-----------------------------------------------------------------------------------------------------


P.S. Looking back at the previous Cave Pearl scripts, I can see where I borked the fomatting so badly that it probably made any real coders out there wish they were blind. But hey, I have only ever taken one programming course in my life...in high school...and that was well before the days of screens & keyboards...just bubble cards and printouts several days later.  Just something to consider before using any of this code on your own project...


