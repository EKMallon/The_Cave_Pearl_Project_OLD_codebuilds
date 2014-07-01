/*      Cave Pearl Datalogger script (v19) by Edward Mallon
* 	 
*       This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
*       without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
* 	Licensed under the GPL v3 license. See http://www.gnu.org/licenses/ for details on reuse and redistribution.
* 	
* 	This script incorporates the work of many other people for the libraries and some sensor functions. I have done my best 
*       to site these sources directly in the comments with each section, and I make no claims whatsoever on those sections of code.
*       That being said, the overall integration of these components into a full datalogging system is a work that I have spent 
*       many weeks of hard work on, and the functions related to overall data handling, and coordinated operation, are my own creations. 
*       I would appreciate credit for that if you use this software directly, or as the basis for your own similar datalogging project. 
*       I am happy for everyone to take this work as a starting point, and develop it in any way they see fit with one exception: 
*       The GNU General Public License does not permit incorporating this program into proprietary programs.
*
*       Electronic components for the datalogger platform:
*       TinyDuino Processor board (unregulated), TinyShield Proto, TinySheild microSD, 
*       TinyShield Ambient Light sensor (hacked) for vregulated I2C lines, DS3232 Rtc, AT24C32 I2C eeprom
*
*       Sensors supported included in this build:
*       HMC5883 Compass, DS18B20 Temp, TMP102 Temp, BMA180 Acc, BMA250 Acc, Ms5803-02 & 05 Pressure sensors
*       *uncomment defines at the beginning of the script for each sensor you have connected - the rest of the code should
*       adjust automatically...more or less...
*
*/
 

#include <Wire.h>       // I2C lib needs 128 byte Serial buffer
#include <SPI.h>        // not used here, but needed to prevent a RTClib compile error....grrr!
#include <RTClib.h>     // Date, Time and Alarm functions by https://github.com/MrAlvin/RTClib based largely on https://github.com/jcw
#include <avr/sleep.h> 
#include <avr/wdt.h>
#include <PString.h>    // from  http://arduiniana.org/
#include <SdFat.h>      // needs 512 byte buffer,  from  https://code.google.com/p/sdfatlib/downloads/list
SdFat sd; /*Create the objects to talk to the SD card*/
SdFile file;
const byte chipSelect = 10; //sd card chip select pin

#define SampleIntervalMinutes 15   // power-down time in minutes before interupt triggers the next sample
//#define SampleIntSeconds 20 // this is ONLY used for DEBUGGING! otherwise SET to 0! Must Set minutes to zero for sub minute alarms to occur!

#define SamplesPerCycle 42  // MAX # of sample cycles to buffer in eeprom before sd card write 
//The AT25C32 is internally organized into (32,768 bits)=4096 bytes - beyond 4096 characters it rewrites over top of the old data
//(128 x 32byte writes fill entire 4096 byte block) so MAX 64 wout compass (2 pgwrites/cycle) but max=42 if compass is installed! (3 pgwrites/cycle)
unsigned int countLogs = 0;      // # records have been written to each file so far
unsigned int fileInterval = 2800; // #of log records before new logfile is made usually 2880
/* count each time a log is written into each file.  Must be less than 65,535 counts per file.  If the sampleinterval is 15min,
 and fileInterval is 2880 seconds, then 96samples/day * 30days/month = 30 day intervals */
char FileName[] = "LOG00000.CSV";  //the first file name
byte Cycle=0; 

#define filterSamples   7  // #of ACCELEROMETER samples for filtering- odd number, no smaller than 3 - works great at 13 & good at 7-9 samples
// too many sampels and you will run out of free ram!

//Comment out the sensors you are not using  (uncomment all to verify code though)

//****** AND ****** dont forget to edit the DATA file header in SETUP so that the config is written to the SD card file

//#define BMA180_ADDRESS 0x40
//#define BMA250_ADDRESS 0x18
#define TS_TMP102  INSTALLED
//#define TS_DS18B20 INSTALLED
//#define HMC5883_ADDRESS 0x1E
//#define MS5803_02_ISON 0x76//(CSB pin 3 tied to Vdd)
//#define MS5803_05_ISON 0x76//(CSB pin 3 tied to Vdd)

#define unregulatedMCU 1  //tiny duino can read vcc directly
//#define vRegulatedMCU 1  //need to use a voltage divider and analog pin
int Vcc; //the supply voltage (via 1.1 internal band gap OR analog read)
int Vcc2; //the post SD card write voltage
//int Vdelta;   //change in supply voltage after each SD write cycle...to track battery conditioin
const float referenceVolts = 3.3;
// R1 = 10000 from a pin to input voltage // R2 = 10000 from analog pin to ground // - max measurable input voltage of 6.6v
const float resistorFactor = 511; // = 1023.0 * (R2/(R1 + R2));  


#define ECHO_TO_SERIAL   // for debugging
#ifdef ECHO_TO_SERIAL
//#define WAIT_TO_START
/* Wait for serial input in setup(), only if serial is enabled.  You don't want to define WAIT_TO_START unless ECHO_TO_SERIAL is defined, 
 because it would wait forever to start if you aren't using the serial monitor! */
#endif


// 3 color indicator LED pin connections- if there is only one led set all the defines to the same pin
#define RED_PIN 3  //will probably need this interrupt capable pin later...
#define GREEN_PIN 4
#define BLUE_PIN 5

//Global variables
//******************
byte bytebuffer =0;
//int intbuffer=0;  //not used yet

//AT24C32 I2C eeprom 
//******************
#define EEPROM_ADDRESS 0x57               // I2C Buss address of AT24C32 32K EEPROM (first block)
#define EEPromPageSize 32                 // 32 bytes is page size for the AT24C32
unsigned int CurrentPageStartAddress = 0; // set to zero at the start of each cycle
char EEPROMBuffer[28];                    // this buffer contains a string of ascii because I am using the pstring function to load it
uint8_t BytesWrittentoSD = 0;

//DS3231 RTC
//**********
#define DS3231_ADDRESS 104
RTC_DS3231 RTC;                            //DS3231 will function with a VCC ranging from 2.3V to 5.5V
byte Alarmhour = 1;
byte Alarmminute = 1;
byte Alarmday = 1;                         //only used for sub second alarms
byte Alarmsecond = 1;                      //only used for sub second alarms
byte INTERRUPT_PIN = 2;                    // SQW is soldered to this pin on the arduino
volatile boolean clockInterrupt = false;
char CycleTimeStamp[ ]= "0000/00/00,00:00"; //16 characters without seconds!

// ACC common variables
// ********************
// this is alot of memory to eat up just for acclerometer smoothing
#if defined(BMA250_ADDRESS) || defined(BMA180_ADDRESS)
int rawACCx [filterSamples];   // raw sensor values for x 
int rawACCy [filterSamples];   // raw sensor values for y 
int rawACCz [filterSamples];   // raw sensor values for z 
#endif
int smoothACCx = 0;  // smoothed x data
int smoothACCy = 0;  // smoothed y data
int smoothACCz = 0;  // smoothed z data

// Temperature sensor common variables
// ***********************************
int TEMP_Raw = 0;
float TEMP_degC = 0.0;
uint8_t wholeTemp=0;
uint8_t fracTemp=0;

#ifdef TS_TMP102
int TMP102_ADDRESS = 0x48;  // 72=base address (3 others possible)
byte errorflag=0; //used in tmp102 functions... might not be needed after error process is integrated
#endif

#ifdef TS_DS18B20    //variables for DS18B20 temperature sensor
#include <OneWire.h>    // from  http://www.pjrc.com/teensy/td_libs_OneWire.html only need if DS18B20 is connected!
// Also see Dallas Temperature library by Miles Burton: http://milesburton.com/Dallas_Temperature_Control_Library
const byte DS18B20_PIN=4;  // DS18B20 Data on digital pin 4 will have to move this with 3 color led!
OneWire ds(DS18B20_PIN);
byte addr[8];  
#endif

// Compass
// *******
int CompassX = 0;
int CompassY = 0;
int CompassZ = 0;

// MS5803-0X pressure sensors
// **************************

// D1 and D2 need to be unsigned 32-bit integers (long 0-4294967295)
uint32_t  D1 = 0;    // Store D1 value = uncompensated pressure value
uint32_t  D2 = 0;    // Store D2 value = uncompensated temperature value

#if defined(MS5803_02_ISON) || defined(MS5803_05_ISON)

//variables for MS5803 functions
#define MS5803_I2C_ADDRESS    0x76 // or 0x77
#define CMD_RESET	0x1E	// ADC reset command
#define CMD_ADC_READ	0x00	// ADC read command
#define CMD_ADC_CONV	0x40	// ADC conversion command
#define CMD_ADC_D1	0x00	// ADC D1 conversion
#define CMD_ADC_D2	0x10	// ADC D2 conversion
#define CMD_ADC_256	0x00	// ADC resolution=256
#define CMD_ADC_512	0x02	// ADC resolution=512  //default after reset
#define CMD_ADC_1024	0x04	// ADC resolution=1024
#define CMD_ADC_2048	0x06	// ADC resolution=2048
#define CMD_ADC_4096	0x08	// ADC resolution=4096
//resolutions at these oversample ratios are  0.084 / 0.054 / 0.036 / 0.024 mbar //
uint8_t _Resolution = 512;  //oversampling rate
// Create array to hold the 8 sensor calibration coefficients
unsigned int sensorCoeffs[8]; // unsigned 16-bit integer (0-65535)
// bytes to hold the results from I2C communications with the sensor
byte HighByte;
byte MidByte;
byte LowByte;  

float mbar; // Store pressure in mbar. 
float tempC; // Store temperature in degrees Celsius

int wholembar=0;
int fracmbar=0;

//    float tempF; // Store temperature in degrees Fahrenheit
//    float psiAbs; // Store pressure in pounds per square inch, absolute
//    float psiGauge; // Store gauge pressure in pounds per square inch (psi)
//    float inHgPress;	// Store pressure in inches of mercury
//    float mmHgPress;	// Store pressure in mm of mercury

// These three variables are used for the conversion steps (which takes 3K more memory than I have left)
// generally I only use these during debugging...
// They should be signed 32-bit integer initially 
// i.e. signed long from -2147483648 to 2147483647
int32_t mbarInt; // pressure in mbar, initially as a signed long integer
int32_t	dT = 0;
// These values need to be signed 64 bit integers 
// (long long = int64_t)
int64_t TEMP = 0;  //I could not get the MS5803-02 lib working unless this was 64
int64_t	Offset = 0;
int64_t	Sensitivity  = 0;
int64_t	T2 = 0;
int64_t	OFF2 = 0;
int64_t	Sens2 = 0; 

#endif


/**********************************************
 *  *   *   *   *   *   SETUP   *   *   *   *   *
 ***********************************************/
// errors in setup will always call error routine that halts the system
void setup () {

  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, HIGH);//pull up the interrupt pin

  pinMode(RED_PIN, OUTPUT);
  digitalWrite(RED_PIN, LOW);     // error state
  pinMode(GREEN_PIN, OUTPUT);
  digitalWrite(GREEN_PIN, HIGH);  // sensor reads
  pinMode(BLUE_PIN, OUTPUT);
  digitalWrite(BLUE_PIN, LOW);    // eeprom & SD card writes

  Serial.begin(9600);
  Wire.begin();
  RTC.begin();

  // check RTC
  //**********
  clearClockTrigger(); //stops RTC from holding the interrupt low if system reset just occured
  RTC.turnOffAlarm(1);
  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);  
  if (now.unixtime() < compiled.unixtime()) { //checks if the RTC is not set yet
    Serial.println(F("RTC is older than compile time! Updating"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
    Serial.println(F("Clock updated...."));
  }

  // startup delays if needed
  //*************************
#ifdef ECHO_TO_SERIAL
  serial_boilerplate();
#endif

  delay(8000); // delay here just to prevent power stutters from writing multiple headers to the sd card

#ifdef WAIT_TO_START  // only triggered if WAIT_TO_START is defined at beging of code
  Serial.println(F("Type any character to start"));
  while (!Serial.available());
#endif  

  //Temp sensor(s) init
  //**********************
#ifdef TS_TMP102
  initTMP102();
#endif

#ifdef TS_DS18B20
  if ( !ds.search(addr)) 
  {
    Serial.println(F("---> ERROR: Did not find the DS18B20 Temperature Sensor!"));
    return;
  }
  else 
  {
    Serial.print(F("DS18B20 found at ROM address ="));
    for(byte i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    Serial.println();
  }
#endif

  //Accelerometer init
  //**********************
#ifdef BMA250_ADDRESS
  initBMA250(); 
#endif

#ifdef BMA180_ADDRESS 
  initBMA180();  
#endif

  //Compass init
  //**********************
#ifdef HMC5883_ADDRESS
  initHMC5883(); 
#endif


#if defined(MS5803_I2C_ADDRESS)
  // Initialize the MS5803 sensor. This will report the
  // conversion coefficients to the Serial terminal if present.
  // If you don't want all the coefficients printed out, 
  // set sensor.initializeMS_5803(false) otherwise send true

  if (initializeMS_5803(true)) {
    Serial.println( "MS5803 CRC check OK." );
  } 
  else {
    Serial.println( "MS5803 CRC check FAILED!" );
  }
#endif

  //get the SD card ready
  //**********************
  pinMode(chipSelect, OUTPUT);  //make sure that the default chip select pin is set to output, even if you don't use it 

#ifdef ECHO_TO_SERIAL
  Serial.print(F("Initializing SD card...")); 
#endif

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library. // change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println(F("Cound not Initialize Sd Card"));
    error();
  }

#ifdef ECHO_TO_SERIAL
  Serial.println(F("SD card initialized."));
  Serial.print(F("The sample interval for this series is: ")); 
  Serial.print(SampleIntervalMinutes); 
  Serial.print(F(" minutes"));
#ifdef SampleIntSeconds
  Serial.print(F(" ")); 
  Serial.print(SampleIntervalMinutes);
  Serial.print(F(" Seconds ")); 
#endif
  Serial.println(F(" ")); 

  Serial.println(F("YYYY/MM/DD HH:MM, Vcc(mV), accX = , accY = , accZ = , Raw Temp, Temp (C), CompassX, CompassY, CompassZ,D1(Raw MS5803 Pressure), D2(Raw MS5803 Temperature), "));
  delay(50); //short delay to clear serial com lines 
#endif

  // open the file for write at end like the Native SD library  
  // see http://forum.arduino.cc/index.php?topic=49649.0
  // O_CREAT = create the file if it does not exist
  if (!file.open(FileName, O_RDWR | O_CREAT | O_AT_END)) {
    Serial.println(F("1st open LOG.CSV fail"));
    error();
  }
  file.println(F("Unit#:x, Data platform:Y, MCU: TinyDuino, TempSensor:TMP102, ACC: Bma180, Compass:HMC5883, MS5803"));// edit this line with specifics before each upload
  file.println(F(" "));

#if defined(MS5803_I2C_ADDRESS) //write cal constants to the sd card

  file.println(F("Ms5803-2A Pressure sensor Calibration constants are:"));
  file.print(F("C1="));
  file.print(sensorCoeffs[1]);
  file.println(F(" =SENS(t1)"));
  file.print(F("C2="));
  file.print(sensorCoeffs[2]);
  file.println(F(" =OFF(t1)"));
  file.print(F("C3="));
  file.print(sensorCoeffs[3]);
  file.println(F(" =TCS"));
  file.print(F("C4="));
  file.print(sensorCoeffs[4]);
  file.println(F(" =TCO"));
  file.print(F("C5="));
  file.print(sensorCoeffs[5]);
  file.println(F(" =RTreff"));
  file.print(F("C6="));
  file.print(sensorCoeffs[6]);
  file.println(F(" =TEMPSENS"));
  file.println(F(" "));

#endif  

  file.print(F("The sample interval for this series is: "));
  file.print(SampleIntervalMinutes);
  file.print(F(" minutes"));
#ifdef SampleIntSeconds
  Serial.print(F(" ")); 
  Serial.print(SampleIntSeconds);
  Serial.print(F(" Seconds ")); 
#endif
  file.println(F(" "));
  file.println(F("YYYY/MM/DD HH:MM, Vcc(mV), accX = , accY = , accZ = , Raw Temp , Temp (C), CompassX, CompassY, CompassZ, "));
  file.println(F(" "));
  delay(100); //short delay to clear coms
  file.close();

  digitalWrite(GREEN_PIN, LOW);

  DIDR0 = 0x0F; // disable the digital inputs on analog 0..3
}

/***********************************************
 *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
 ***********************************************/
// errors during main loop will only call error routine & halt the system if ECHO_TO_SERIAL is defined (ie: we are in debug mode)
// that way if one sensor dies in the field we can still get data from the others
void loop () 
{ 
  // keep track of how many lines have been written to a file
  // after so many lines, start a new file
  if(countLogs >= fileInterval){
    digitalWrite(BLUE_PIN, HIGH);
    delay(1);
    digitalWrite(BLUE_PIN, LOW);
    createLogFile();   // create a new file
    countLogs = 0;     // reset our counter to zero
    digitalWrite(BLUE_PIN, HIGH);
    delay(1);
    digitalWrite(BLUE_PIN, LOW);
  }

  CurrentPageStartAddress = 0;  //yes I know I am burning out the first block of the eeprom

  for (int Cycle = 0; Cycle < SamplesPerCycle; Cycle++) { //this counts from 0 to (SamplesPerCycle-1)

    if (clockInterrupt) {  // old location of this call.... why was it here before?
      clearClockTrigger(); 
      digitalWrite(INTERRUPT_PIN, HIGH);//set weak internal pull up the interrupt pin
    }

    digitalWrite(GREEN_PIN, HIGH); 
    delay(1); 
    digitalWrite(GREEN_PIN, LOW);  //sample heartbeat pip

    DateTime now = RTC.now();  // Read the time and date from the RTC

    //sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    //Time read always occurs <1 sec after interrupt, so seconds data is always "00" - so I dont record it any more
    sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute()); 

    Vcc = (readVcc());   
    if (Vcc < 2800){
      Serial.println(F("Supply voltage TOO LOW!"));
      error();
    }

#ifdef vRegulatedMCU  //3400mv is minimum allowed input to the voltage regulator
    if ((Vcc-700) < 2800){
      Serial.println(F("Supply voltage TOO LOW!"));
      error();
    }
#endif

    //Read the accelerometer:
#ifdef BMA180_ADDRESS
    readBMA180();
#endif  
    //or
#ifdef BMA250_ADDRESS
    readBMA250(); 
#endif

    //Read the compass:
#ifdef HMC5883_ADDRESS
    readHMC5883();
#endif

#if defined(MS5803_I2C_ADDRESS)
    // Use readSensor() function to get pressure and temperature reading. 
    readSensor();
#endif

    //Read the temperature:
#ifdef TS_TMP102   
    TEMP_Raw =readTMP102();
    TEMP_degC =TEMP_Raw*0.0625;
    wholeTemp = (int)TEMP_degC; 
    fracTemp= (TEMP_degC - wholeTemp) * 1000; // split float into 2 intergers
#endif
    // or
#ifdef TS_DS18B20
    // Must do this reading last due to 1 second of sleeping while waiting for sensor! 
    TEMP_Raw =readDS18B20Temp();
    TEMP_degC =TEMP_Raw/16.00; 
    wholeTemp = (int)TEMP_degC; 
    fracTemp= (TEMP_degC - wholeTemp) * 1000; // Float split into 2 intergers
#endif



    //serial output for debugging - comment out ECHO_TO_SERIAL to eliminate
#ifdef ECHO_TO_SERIAL
    Serial.print(CycleTimeStamp); 
    Serial.print(F(" Cycle: ")); 
    Serial.print(Cycle);
    Serial.print(F(", Vcc= ")); 
    Serial.println(Vcc); 
    Serial.print(F(", ACCx= ")); 
    Serial.print(smoothACCx); 
    Serial.print(F(", ACCy= ")); 
    Serial.print(smoothACCy); 
    Serial.print(F(", ACCz= "));
    Serial.println(smoothACCz); 

#if defined(MS5803_I2C_ADDRESS)
    Serial.print(F(" D1 = "));
    Serial.print(D1);
    Serial.print(F(" D2 = "));
    Serial.print(D2);
    Serial.print(F(" MS5803 Pressure = "));
    wholembar = (int)mbar; 
    fracmbar = (mbar - wholembar) * 100;
    Serial.print(wholembar);
    Serial.print(F("."));
    Serial.print(fracmbar);
    //Serial.print(mbar); // it takes one whole K to print floats! dont do it!
    Serial.print(F(" mbar   "));
    wholeTemp = (int)tempC; 
    fracTemp= (tempC - wholeTemp) * 1000;
    Serial.print(F(" MS5803 Temp = "));
    //Serial.print(tempC);
    Serial.print(wholeTemp);
    Serial.print(F("."));
    Serial.print(fracTemp); //avoid sending floats to serial print - it eats sram! 
    Serial.print(F("C"));
#endif


    Serial.print(F(", Temp C: "));
    Serial.print(wholeTemp);
    Serial.print(F("."));
    Serial.print(fracTemp); //avoid sending floats to serial print - it eats sram! 
    Serial.print(F(", Free Ram: "));
    Serial.print(freeRam()); //only use this for debugging
    Serial.println(F(" ")); //terminate line

#ifdef HMC5883_ADDRESS
    Serial.print(F(", CompX: ")); 
    Serial.print(CompassX); 
    Serial.print(F(" CompY: ")); 
    Serial.print(CompassY);
    Serial.print(F(" CompZ: ")); 
    Serial.println(CompassZ);
#endif 

    delay(50); //short delay to clear serial com lines
#endif

    //Construct first char string of 28 bytes - end of buffer is filled with blank spaces flexibly with pstring
    //but could contruct the buffer with sprintf if I wasn't changing my sensors so often!

    PString str(EEPROMBuffer, sizeof(EEPROMBuffer)); 
    str = CycleTimeStamp;           //17 / 16 characters without seconds plus comma
    str.print(F(","));
    str.print(Vcc);                 // 5 / 4 char typical: 4096,
    str.print(F(","));
    str.print(smoothACCx);          // 6  5 or 6char:   +_1023 from BMA250  OR (+- 16,384) from BMA180
    str.print(F("                      "));  // just filler spaces for the end of the string to accomodate changes in data size
    //typical: 0000/00/00,00:00,4500,-1200

    digitalWrite(BLUE_PIN, HIGH); 
    delay(1);
    digitalWrite(BLUE_PIN, LOW);
    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer); // whole page is written at once here
    CurrentPageStartAddress += EEPromPageSize;

    //Construct second char string of 28 bytes to complete the record
    str = ","; 
    str.print(smoothACCy);
    str.print(F(","));
    str.print(smoothACCz);
    str.print(F(","));
    //str.print(TEMP_Raw);  //4 digits both 12 bit sensors, so 4095 is largest value 
    //str.print(F(",")); 
    str.print(wholeTemp);  //two digits positive
    str.print(F("."));
    str.print(fracTemp);   //3 digits, because *1000
    str.print(F(","));
    str.print(D1);
    str.print(F("                  "));
    //typical: ,-1255,-255,498,21.56
    //can calculate TMP celcius later with deg_c = TMP102_raw*0.0625;   

    //24 bit raw ADC value for D1 & D2  max = 16777216 = 8 characters! 9 with comma for each value!

    digitalWrite(BLUE_PIN, HIGH);
    delay(1);
    digitalWrite(BLUE_PIN, LOW);
    Write_i2c_eeprom_page(EEPROM_ADDRESS, 
    CurrentPageStartAddress, EEPROMBuffer); // 28 bytes/page is max whole page is written at once here
    CurrentPageStartAddress += EEPromPageSize;

#if defined(MS5803_I2C_ADDRESS) || defined(HMC5883_ADDRESS)  // if needed construct a third char string
    //Construct third char string of 28 bytes to complete the record
    str = ",";
    str.print(D2); // 8 characters
    str.print(F(","));

    // if(CompassX==0) then print the first order MS5803 stuff..
    str.print(CompassX);  //max 18 characters from compass
    str.print(F(","));
    str.print(CompassY);
    str.print(F(","));
    str.print(CompassZ);
    str.print(F(","));
#ifdef SampleIntSeconds
    str.print(F("V2,"));
    str.print(Vcc2); //just recording this in when I am in debugging mode
    str.print(F(","));
#endif
    str.print(F("                                          ")); 

    digitalWrite(BLUE_PIN, HIGH);
    delay(1);
    digitalWrite(BLUE_PIN, LOW);
    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer); // 28 bytes/page is max whole page is written at once here
    CurrentPageStartAddress += EEPromPageSize;

#endif

    // IF full set of sample cycles is complete, run a loop to dump data to the sd card
    // BUT only if Vcc is above 2.85 volts so we have enough juice!
    if (Cycle==(SamplesPerCycle-1) && Vcc >= 2850){ 
      digitalWrite(RED_PIN, HIGH);

      // this for debugging only
#ifdef ECHO_TO_SERIAL
      Serial.println(F(" --Writing to SDcard --")); 
      delay (10);
      //Serial.print(F("Current eeprom page address:"));Serial.print(CurrentPageStartAddress ); delay (10);
      //Serial.print(F("-now set to zero  & Cycle:"));Serial.println(Cycle); delay (10);
#endif

      file.open(FileName, O_RDWR | O_AT_END);
      // open the file for write at end like the Native SD library
      // if (!file.open(FileName, O_RDWR | O_AT_END)) {
      // error("L open file fail");
      //}
      // file.print(F("EEPROM page address AT start:"));file.println(CurrentPageStartAddress ); // this for debuggin only

      CurrentPageStartAddress=0;  //reset the page counter back to the beginning of the eeprom stack
      for (int i = 0; i < SamplesPerCycle; i++) {  //loop to read from I2C ee and write to SD card 

        Read_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer, sizeof(EEPROMBuffer) );  //there will be a few blank spaces
        CurrentPageStartAddress += EEPromPageSize; 
        file.write(EEPROMBuffer,sizeof(EEPROMBuffer));
        BytesWrittentoSD = BytesWrittentoSD + sizeof(EEPROMBuffer);

        Read_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer, sizeof(EEPROMBuffer) );
        CurrentPageStartAddress += EEPromPageSize;
        file.write(EEPROMBuffer,sizeof(EEPROMBuffer));
        BytesWrittentoSD = BytesWrittentoSD + sizeof(EEPROMBuffer);

#if defined(MS5803_I2C_ADDRESS)  || defined(HMC5883_ADDRESS)  //the compass or pressure sensor adds a third data line to each cycle
        Read_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer, sizeof(EEPROMBuffer) );
        CurrentPageStartAddress += EEPromPageSize;
        file.write(EEPROMBuffer,sizeof(EEPROMBuffer));
        BytesWrittentoSD = BytesWrittentoSD + sizeof(EEPROMBuffer);
#endif

        file.println(F(" "));  //add a carridge return to the file
        BytesWrittentoSD = BytesWrittentoSD + 2;  //why did I add this little bit?

        countLogs++;
        // An application which writes to a file using print(), println() or write() must call sync()
        // at the appropriate time to force data and directory information to be written to the SD Card. approximately 512 bytes
        if(BytesWrittentoSD > 440) {
          syncTheFile; 
          BytesWrittentoSD=0;
        } //might need to lower this
      }

      file.close(); 
      CurrentPageStartAddress=0;

      Vcc2 = (readVcc());
      if (Vcc2 < 2800){
        Serial.println(F("Supply voltage TOO LOW!"));  //2.7 volts is listed as the minimum safe write voltage for Sandisk mem cards
        delay(10); //wait till the SD card is back in sleep mode (5ms) then go to error state
        error();
        // Vdelta = Vcc-Vcc2; //also could use Vdelta to track the power left in the batteries?
      }

#ifdef vRegulatedMCU  //3400mv is minimum allowed input on the voltage regulated system
      if ((Vcc2-700) < 2800){
        Serial.println(F("Supply voltage TOO LOW!"));
        delay(10); 
        error();
      }
#endif


      digitalWrite(RED_PIN, LOW);
    }

    // setNextAlarmTime();
    Alarmhour = now.hour(); 
    Alarmminute = now.minute()+SampleIntervalMinutes;
    Alarmday = now.day();
    //Alarmsecond = now.second()+SampleIntSeconds;
    if (SampleIntervalMinutes > 0) //then our alarm is in (SampleInterval) minutes
    {
      if (Alarmminute > 59) {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
        Alarmminute =0; 
        Alarmhour = Alarmhour+1; 
        if (Alarmhour > 23) {
          Alarmhour =0;
        }
      }
      RTC.setAlarm1Simple(Alarmhour, Alarmminute);
    }
    else
    {  // for testing and debug I sometimes want the alarms more frequent than 1 per minute.
      Alarmsecond = now.second()+SampleIntSeconds;
      if (Alarmsecond >59){
        Alarmsecond =0;
        Alarmminute = Alarmminute+1;  
        if (Alarmminute > 59) 
        {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
          Alarmminute =0; 
          Alarmhour = Alarmhour+1; 
          if (Alarmhour > 23) { //uhoh a day rollover, but we dont know the month..so we dont know the next day number?
            Alarmhour =0;
            delay(327670);
            delay(327670);
            delay(327670); //Just wait for a minute..
            DateTime now = RTC.now();  //now start the numbers again with the new day already rolled over
            Alarmday = now.day();
            Alarmhour = now.hour(); 
            Alarmminute = now.minute();
            Alarmsecond = now.second()+SampleIntSeconds; 
          }
        }
      }
      RTC.setA1Time(Alarmday, Alarmhour, Alarmminute, Alarmsecond, 0b00001000, false, false, false);  
      //The variables ALRM1_SET bits and ALRM2_SET are 0b1000 and 0b111 respectively.
      //setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM)
    }
    RTC.turnOnAlarm(1);

#ifdef ECHO_TO_SERIAL
    Serial.println(F(" "));
    Serial.print(F("  Alarm Set:")); 
    Serial.print(now.hour(), DEC); 
    Serial.print(':'); 
    Serial.print(now.minute(), DEC);
    Serial.print(F(" Sleep:")); 
    Serial.print(SampleIntervalMinutes);
    Serial.print(F(" min."));
#ifdef SampleIntSeconds
    Serial.print(SampleIntSeconds);
    Serial.print(F(" sec.")); 
# endif   
    Serial.println(F(" "));
    delay(50); //a delay long enought to boot out the serial coms 
#endif

    sleepNwait4RTC();  //the sleep call is inside the main cycle counter loop

  }   //samples per cycle loop terminator 
}   //the main void loop terminator



// ************************************************************************************************************
//  *  *   *   *  *  COMMON FUNCTIONS  *   *   *   *   *
// ************************************************************************************************************

/**********************************************
 * SD card funtions : Create file & SYNC
 ***********************************************/

// from http://forums.adafruit.com/viewtopic.php?f=31&t=17964

void createLogFile(void) {
  // create a new file, up to 100,000 files allowed
  // we will create a new file every time this routine is called
  // If we are creating another file after fileInterval, then we must
  // close the open file first.
  if (file.isOpen()) {
    file.close();
  }
  for (uint16_t i = 0; i < 100000; i++) {
    FileName[3] = i/10000 + '0';
    FileName[4] = i/1000  + '0';
    FileName[5] = i/100   + '0';
    FileName[6] = i/10    + '0';
    FileName[7] = i%10    + '0';
    // O_CREAT - create the file if it does not exist
    // O_EXCL - fail if the file exists  O_WRITE - open for write
    if (file.open(FileName, O_CREAT | O_EXCL | O_WRITE)) break;  
    //if you can open a file with the new name, break out of the loop
  }

  // clear the writeError flags generated when we broke the new name loop
  file.writeError = 0;

  if (!file.isOpen()) { 
    Serial.println(F(" Disk Full Error"));
    error();
  }
  Serial.print(F("Logging to: "));
  Serial.println(FileName); 

  // fetch the time   //is this what takes away 5 minutes from our clock?
  DateTime now = RTC.now();
  // set creation date time
  if (!file.timestamp(T_CREATE,now.year(),now.month(),now.day(),now.hour(),
  now.minute(),now.second() )) {
    Serial.println(F("Can't timestamp the new file"));
    error();
  }
  // set write/modification date time
  if (!file.timestamp(T_WRITE,now.year(),now.month(),now.day(),now.hour(),
  now.minute(),now.second() )) {
    Serial.println(F("Can't write to the new file"));
    error();
  }
  // set access date
  if (!file.timestamp(T_ACCESS,now.year(),now.month(),now.day(),now.hour(),
  now.minute(),now.second() )) {
    Serial.println(F("Can't set new file access date"));
    error();
  }
  file.sync();
  //file.close();
  //file.open(FileName, O_RDWR | O_AT_END);

  // write the file as a header:
  file.print(F("The sample interval for this series is:"));
  Serial.print(SampleIntervalMinutes);
  Serial.println(F(" minutes"));
  Serial.println(F(" "));
  file.println(F("YYYY/MM/DD HH:MM, Vcc(mV), accX = , accY = , accZ = , Raw Temp, Temp (C), CompassX, CompassY, CompassZ, "));
  Serial.println(F(" "));
  file.close();

#ifdef ECHO_TO_SERIAL
  Serial.println(F("New log file created on the SD card!"));
  delay(100); //short delay to clear serial com lines
#endif  // ECHO_TO_SERIAL

  // write out the header to the file, only upon creating a new file
  if (file.writeError) {
    // check if error writing
    Serial.println(F(" Can't write new file header"));
    error();
  } 

  //    if (!file.sync()) {
  // check if error writing
  //    Serial.println(F("File Syncing Error"));error();
  //  } 

}

// ***********************************************/

void syncTheFile(void) 
{
  /* don't sync too often - requires 2048 bytes of I/O to SD card. 512 bytes of I/O if using Fat16 library */

  digitalWrite(RED_PIN, HIGH);
  delay(1);
  digitalWrite(RED_PIN, LOW);  /* blink LED to show we are syncing data to the card & updating FAT!*/

  if (!file.sync()) { 
    Serial.println(F("File Sync error"));
    error(); //terminal error if you cant save data to sd card
  }  // 15-20 ms for syncing

  digitalWrite(RED_PIN, HIGH);
  delay(1);
  digitalWrite(RED_PIN, LOW); 
}

/*****************************************************************************************************************
 * i2c EEPROM    READ & WRITE PAGEs
 *******************************************************************************************************************/

// Address is a page address
// But data can be maximum of 28 bytes, because the Wire library has a buffer of 32 bytes
void Write_i2c_eeprom_page( int deviceaddress, unsigned int eeaddress, char* data) 
{
  unsigned char i=0;
  unsigned int  address;
  address=eeaddress;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)((address) >> 8));   // MSB
  Wire.write((int)((address) & 0xFF)); // LSB
  do{ 
    Wire.write((byte) data[i]);
    i++;
  } 
  while(data[i]);  
  Wire.endTransmission(); 
  delay(10);  // data sheet says 5ms for page write
}

void Read_i2c_eeprom_page( int deviceaddress, unsigned int eeaddress,char* data, unsigned int num_chars) 
{
  unsigned char i=0;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,(num_chars-1));
  while(Wire.available()) data[i++] = Wire.read();
}

/**********************************************
 * SLEEP and wait for RTC
 ***********************************************/
//defs needed for stopping the ADC during sleep mode
#ifndef cbi 
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void sleepNwait4RTC() 
{
  cbi(ADCSRA,ADEN); // Switch ADC OFF: worth 334 µA during sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(0,clockTrigger, LOW);
  sleep_mode();
  //HERE AFTER WAKING UP
  sleep_disable();
  detachInterrupt(0);
  sbi(ADCSRA,ADEN);  // Switch ADC converter back ON
}
/**********************************************
 * CLOCK TRIGGER FLAG
 ***********************************************/
void clockTrigger() {
  clockInterrupt = true; //do something quick, flip a flag, and handle in loop();
}
/**********************************************
 * CLEAR CLOCK TRIGGER
 ***********************************************/
// clear alarm interupt from http://forum.arduino.cc/index.php?topic=109062.0
void clearClockTrigger()
{
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231
  Wire.write(0x0F);               //Tell the device which address we want to read or write
  Wire.endTransmission();         //Before you can write to and clear the alarm flag you have to read the flag first!
  Wire.requestFrom(0x68,1);       // Read one byte
  bytebuffer=Wire.read();      // In this example we are not interest in actually using the bye
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231 
  Wire.write(0x0F);               //Tell the device which address we want to read or write
  Wire.write(0b00000000);         //Write the byte.  The last 0 bit resets Alarm 1
  Wire.endTransmission();
  clockInterrupt=false;           //Finally clear the flag we use to indicate the trigger occurred
}

/**********************************************
 * SLEEP and wait for watchdog timer
 ***********************************************/
// you must use one of the  setWTD_125ms() functions before you call this.. otherwise you sleep forever!
void sleepNwait4WDT() {   //uses the wdt's internal int..so no need for attaching int to a pin here
  ADCSRA |= (0<<ADEN);                     // disable ADC
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  sleep_cpu ();     // sleeping ... 
  // first action after leaving WDT Interrupt Vector:
  sleep_disable();                       // disable sleep  
  //ADCSRA |= (1<<ADEN);                   // switch ADC on
  ADCSRA |= (1<<ADEN);                  // enable ADC
  delay (3); //  if you want to communicate (I2C,SPI,UART,etc) you should delay for 2 - 5 mS after waking up or else you could lose bits         
}

/**********************************************
 * WDT INTERRUPT HANDLER
 ***********************************************/
ISR (WDT_vect) 
{
  wdt_disable();  // disable watchdog
} 

// *********************************************************
// Set WDT delays  (to save power during sensor conversions)  
// ********************************************************* 
//#define WDPS_16MS   (0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(0<<WDP0) 
#define WDPS_32MS   (0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0) 
//#define WDPS_64MS   (0<<WDP3)|(0<<WDP2)|(1<<WDP1)|(0<<WDP0) 
#define WDPS_125MS  (0<<WDP3)|(0<<WDP2)|(1<<WDP1)|(1<<WDP0) 
//#define WDPS_250MS  (0<<WDP3)|(1<<WDP2)|(0<<WDP1)|(0<<WDP0) 
//#define WDPS_500MS  (0<<WDP3)|(1<<WDP2)|(0<<WDP1)|(1<<WDP0) 
//#define WDPS_1S     (0<<WDP3)|(1<<WDP2)|(1<<WDP1)|(0<<WDP0) 
//#define WDPS_2S     (0<<WDP3)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0) 
//#define WDPS_4S     (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(0<<WDP0) 
//#define WDPS_8S     (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0)

void setWTD_125ms()   //the acclerometer reads use this one
{  
  cli();         // disable interrupts
  MCUSR = 0;     // clear various "reset" flags
  WDTCSR = (1<<WDCE)|(1<<WDE); // Set Change Enable bit and Enable Watchdog System Reset Mode. 
  WDTCSR = (1<<WDIE)|(1<<WDIF)|(1<<WDE )| WDPS_125MS;   // Set Watchdog prescalar as per your define above
  wdt_reset();   // pat the dog
  sei();         // Enable global interrupts  
} 

void setWTD_32ms()   // Tmp102 sensor read uses this one
{ 
  cli();         // disable interrupts
  MCUSR = 0;     // clear various "reset" flags
  WDTCSR = (1<<WDCE)|(1<<WDE); // Set Change Enable bit and Enable Watchdog System Reset Mode. 
  WDTCSR = (1<<WDIE)|(1<<WDIF)|(1<<WDE )| WDPS_32MS;   // Set Watchdog prescalar as per your define above
  wdt_reset();   // pat the dog
  sei();         // Enable global interrupts  
} 

/*******************************************************
 * READ VCC          using internal 1.1 v  OR analog pin
 ********************************************************/
// from http://forum.arduino.cc/index.php/topic,15629.0.html and http://forum.arduino.cc/index.php?topic=88935.0

long readVcc() 
{ 

  // Need to switch between analog pin read & vcc trick depending on which board is used
  long result;  

#ifdef unregulatedMCU // then re can use internal vcc trick
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(3); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
#endif

  //10 bit resolution, returning integers from 0 to 1023
  // we have a simple 2x 10 Kohm resistor divider supplying this pin so it can read Vraw above Vcc 
#ifdef vRegulatedMCU   //the supply voltage via analog read from a 10k ohm resistor divider
  int avrgVraw = 0; 
  int ActualRaw_mV = 0;
  int vRaw = 0;  
  analogRead(A0);        //ignore the first reading
  delay(1);              //settling time
  vRaw=analogRead(A0);   // read the value from the A0 pin
  avrgVraw = avrgVraw + vRaw;
  delay(1);
  vRaw=analogRead(A0);
  avrgVraw = avrgVraw + vRaw;
  delay(1);
  vRaw=analogRead(A0);
  avrgVraw = avrgVraw + vRaw;
  avrgVraw = avrgVraw/3;  //avrg of 3readings
  float volts = (avrgVraw / resistorFactor) * referenceVolts ; // calculate the ratio
  result = (volts*1000);// conv to millivolts
#endif

  return result;
}

// ************************************************************************************************************
// I2C   REGISTER   FUNCTIONS
// ************************************************************************************************************

/* Writes newValue to address register bits on device using the bitmask to protect the other bits */

byte i2c_writeRegBits(int DEVICE, byte address, byte newValue, byte mask) {
  byte result,current;  
  current = i2c_readRegByte(DEVICE, address);  //load the existing register contents
  result = i2c_writeRegByte(DEVICE, address, (current & ~mask) | (newValue & mask));  //eliminate old bits, and OR in the new ones
  return result;
}

// based on https://github.com/makerbot/BMA180-Datalogger/blob/master/bma180-datalogger-shield/bma180-logger/bma180.ino

byte i2c_readRegByte(int dev_address, byte reg_address)  //MUST be interger for the i2c address
{
  byte temp;

  Wire.beginTransmission(dev_address); //set destination target
  Wire.write(reg_address);
  Wire.endTransmission();        

  Wire.beginTransmission(dev_address); //get data from device
  Wire.requestFrom(dev_address, 1);
  while(Wire.available())
  {
    temp = Wire.read();
  }

  return temp;
}


byte i2c_writeRegByte(int dev_address, byte reg_address, byte data)  
{
  byte result;

  Wire.beginTransmission(dev_address);
  Wire.write(reg_address);
  Wire.write(data);
  result = Wire.endTransmission();

  //should I do some error checking??
  if(result > 0)
  {
    Serial.print(F("FAIL in I2C reg write! Result code is "));
    Serial.println(result);

#ifdef ECHO_TO_SERIAL   //only call halt on error if in debug mode
    error();
#endif
  }
  //else
  //{
  //  Serial.println(" ");
  //}
  delay(10);  //sensors often need some settling time after register writing (BMA180 does)

  return result;
} 

/**********************************************
 * BOILERPLATE
 ***********************************************/
void serial_boilerplate() 
{
  Serial.println(F("The Cave Pearl: An Open Source Hydrometric Pendulum"));
  Serial.println(F("==================================================="));
  Serial.println();
  Serial.println(F("Developed by Edward Mallon: http://edwardmallon.wordpress.com/"));
  Serial.println();
}

/**********************************************
 * FREE RAM AVAILIABLE
 ***********************************************/
// from: http://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


/**********************************************
 * ERROR HANDLER
 ***********************************************/
// I should really figure out how to turn this into a perminant sleep function
// more advanced debugging: http://forum.arduino.cc/index.php?topic=203282.0
void error() 
{
  digitalWrite(RED_PIN, HIGH);
  sleepNwait4WDT(); //since we have gone to sleep without setting the wdt, this is sleep forever!
  //while (1);  //or could start an endless loop,
}

// ************************************************************************************************************
//   *  *   *  *  *  *  *  *  *  *  SENSOR FUNCTIONS  *  *  *  *  *  *  *  *  *  *  *  *  * 
// ************************************************************************************************************

// ************************************************************************************************************
// I2C TMP102 TEMPERATURE SENSOR
// ************************************************************************************************************
//  writes 2 bytes to registers, instead of one, so not itegrated with I2C write byte functions yet!

#if defined(TS_TMP102)

#define TMP102_CFG_default_byte1 B01100001  // 12 bit rez WITH ShutDown bit turned ON
#define TMP102_CFG_default_byte2 B10100000  // just the defaults from pg 7
#define TMP102_OneShotBit B10000000         // one-shot by ORing D7 of CFG byte 1 to 1
// err1-byte pointer to write to tmp102 BEFORE reading back 2 bytes of data from that register
#define TMP102_TEMP_REG_pointer 0x00  // temperature register, read only, 16bits
#define TMP102_CONF_REG_pointer 0x01  // config register, read/write, 16 bits

void initTMP102()
{
#ifdef ECHO_TO_SERIAL
  Serial.println(F("Initializing TMP102 Temperature sensor..."));
  delay(10); 
#endif

  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(TMP102_CONF_REG_pointer);
  Wire.write(TMP102_CFG_default_byte1);  //Sets to 12bit, sd mode on
  Wire.write(TMP102_CFG_default_byte2);  //none of these settings matter in one shot mode, at my temperature range, but set them to defaults anyway
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("Initial control reg writing on TMP102 failed!"));
    error();
    errorflag=0;
  }

  // set one-shot bit to "1" - starts a single conversion then sleeps the sensor
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(TMP102_CONF_REG_pointer); // Select control register.
  Wire.write(TMP102_CFG_default_byte1 | TMP102_OneShotBit); // Start one-shot conversion 40μA during conv
  //Wire.write(TMP102_CFG_default_byte2);  //dont need to bother writing the second byte
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("OOPS! Problem setting OneSHOT bit on TMP102! "));
    error();
    errorflag=0;
  }   
  TEMP_Raw=readTMP102();

#ifdef ECHO_TO_SERIAL 
  Serial.print(F("Success: TMP102 has been initialized: First Raw read="));
  Serial.print(TEMP_Raw);
  Serial.println(F(" "));
#endif

}

int readTMP102() 
{
  //float deg_c; 
  errorflag=0; 
  // start by resetting the one shot bit back to zero
  Wire.beginTransmission(TMP102_ADDRESS);
  Wire.write(TMP102_CONF_REG_pointer); 
  Wire.write(TMP102_CFG_default_byte1);  //Sets to 12bit, sd mode on 
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("TMP102 clearing OS bit in CFG reg failed..."));

#ifdef ECHO_TO_SERIAL  //if echo is on, we are in debug mode, and errors force a halt. 
    error();
#endif

    errorflag=0;
  } 
  // now seting the one-shot bit to "1" will start a single conversion
  Wire.beginTransmission(TMP102_ADDRESS); 
  Wire.write(TMP102_CONF_REG_pointer); // point at the control register.
  Wire.write(TMP102_CFG_default_byte1 | TMP102_OneShotBit); // ORing the bits together
  //Wire.write(TMP102_CFG_default_byte2);  //I don't need to bother writing the second byte?
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("OOPS! problem setting OneSHOT bit on TMP102"));

#ifdef ECHO_TO_SERIAL  //if echo is on, we are in debug mode, and errors force a halt. 
    error();
#endif

    errorflag=0;
  }

  //delay(28);  OR:
  setWTD_32ms();
  sleepNwait4WDT();
  // conversion: 26ms according to the sheet
  // during the conversion the OS bit will temporarily read "0", then revert to "1" after the conversion so you could check for that

  Wire.beginTransmission(TMP102_ADDRESS); //now read the temp
  Wire.write(TMP102_TEMP_REG_pointer); // Select temperature register.
  errorflag = Wire.endTransmission();
  if ( errorflag != 0) {
    Serial.print (F("Can't set temp reg pointer on TMP102"));

#ifdef ECHO_TO_SERIAL  //if echo is on, we are in debug mode, and errors force a halt. 
    error();
#endif

    errorflag=0;
  }
  Wire.requestFrom(TMP102_ADDRESS, 2);
  const byte TempByte1 = Wire.read(); // MSByte, should be signed whole degrees C.
  const byte TempByte2 = Wire.read(); // unsigned because I am not reading any negative temps
  const int Temp16 = (TempByte1 << 4) | (TempByte2 >> 4);    // builds 12-bit value
  //TEMP_degC = Temp16*0.0625;
  return Temp16;
}
#endif

// ************************************************************************************************************
// DS18B20  ONE WIRE TEMPERATURE
// ************************************************************************************************************
// this returns the temperature from one DS18S20 using 12 bit conversion
// also see Dallas Temperature Control library by Miles Burton: http://milesburton.com/Dallas_Temperature_Control_Library

#if defined(TS_DS18B20)

int readDS18B20Temp()
{  
  //byte data[12];
  byte data[2];
  ds.reset();
  ds.select(addr);
  ds.write(0x44); // start conversion, read temperature and store it in the scratchpad 

  //this next bit creates a 1 second WDT delay for the DS18b20 temp conversion 
  //The time needed between the CONVERT_T command and the READ_SCRATCHPAD command has to be at least 
  //750 millisecs (can be shorter if using a D18B20 type with resolutions < 12 bits)
  //if you start getting "85" all the time you did not wait long enough
  // power saving during sleep from http://www.gammon.com.au/forum/?id=11497

  MCUSR = 0;   // clear various "reset" flags  
  WDTCSR = bit (WDCE) | bit (WDE); // allow changes, disable reset
  // set interrupt mode and an interval 
  WDTCSR = bit (WDIE) | bit (WDP2) | bit (WDP1);    //this creates a 1 sec wdt delay for the temp conversion
  wdt_reset();  // pat the dog 
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  sleep_cpu ();    
  // resume here after wdt interrupt happens - cancel sleep as a precaution
  sleep_disable();
  delay(3);

  byte present = ds.reset(); 
  ds.select(addr);  
  ds.write(0xBE); // Read Scratchpad
  for (int i = 0; i < 2; i++) 
  { // we read 9 bytes? but you only use two of them?
    data[i] = ds.read();
  }
  byte MSB = data[1];
  byte LSB = data[0];
  int tempRaw = ((MSB << 8) | LSB); //using two's compliment
  //TEMP_degC = tempRaw / 16;
  return tempRaw;
}

#endif

// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
// sensor output ranges:  -2048 to 2047
// Sensor Field Range ± 1.3 Ga (default)     Gain: 1090 LSb/Gauss (default)     Digital resolution: 0.92 mG/LSb

#if defined(HMC5883_ADDRESS)

void initHMC5883()
{
  //Put the HMC5883 into operating mode
  Wire.beginTransmission(HMC5883_ADDRESS);
  Wire.write(0x02);     // Mode register
  Wire.write(0x00);     // Continuous measurement mode
  Wire.endTransmission();
#ifdef ECHO_TO_SERIAL 
  Serial.println(F(" HMC5883 compass has been initialized"));
#endif 
}

void readHMC5883()
{
  uint8_t ReadBuff[6];

  // Read the 6 data bytes from the HMC5883 - probably should add smoothing to this too?
  Wire.beginTransmission(HMC5883_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883_ADDRESS,6);

  for(int i = 0; i < 6;i++)
  {
    ReadBuff[i] = Wire.read();
  }

  CompassX = ReadBuff[0] << 8;
  CompassX |= ReadBuff[1];

  CompassY = ReadBuff[4] << 8;
  CompassY |= ReadBuff[5];

  CompassZ = ReadBuff[2] << 8;
  CompassZ |= ReadBuff[3];
}

#endif


// ************************************************************************************************************
// I2C Accelerometer BMA180   14bit  (+- 16,384)
// ************************************************************************************************************
/*0x40 = address of the accelerometer with SDO pulled up to VCC (0x41 if SDO pulled down to GND)
 BMA180 default values:  see page 26 of datasheet: DEF_PMODE =0, DEF_SCALE =250,  bandwidth default =20 HZ */

#if defined(BMA180_ADDRESS)

#define BMA180_CMD_BW_TCS  0x20 
//bits 4-7 of this register are the filtering bandwidth bits, 0-3 are the temp compensation bits (page 46)
#define cmd_bandwidth_MASK B11110000
#define BMA180_BW_10HZ     0x00  //to filter down to 10hz, the sensor reads 253 samples, so it takes quite a while!
#define BMA180_RANGEnSMP 0X35  //7.7.1 address of combined Offx, range & smp_skp register (53 decimal) - 2G default sensitivity
//and the three sensitivity range bits to put into that register are:
#define range_MASK          B00001110
#define BMA180_RANGE_1G     0x00
/* the set of Data Register addresses to read out */
#define BMA180_CMD_CHIP_ID          0x00
//#define BMA180_CMD_VERSION          0x01
#define BMA180_CMD_ACC_X_LSB        0x02  /* First of 6 registers of accel data */
#define BMA180_CMD_ACC_X_MSB        0x03
#define BMA180_CMD_ACC_Y_LSB        0x04
#define BMA180_CMD_ACC_Y_MSB        0x05
#define BMA180_CMD_ACC_Z_LSB        0x06
#define BMA180_CMD_ACC_Z_MSB        0x07
//#define BMA180_CMD_TEMP             0x08
/* the device status registers - info about alert/interrupt & if data is availiable to read out*/
#define BMA180_CMD_STATUS_REG1      0x09  //1st tap, alert, slope signs, offset, ee_write
//#define BMA180_CMD_STATUS_REG2      0x0A  //tap and slope sense bits
//#define BMA180_CMD_STATUS_REG3      0x0B  //interrupt statuses from the tap sensing
//#define BMA180_CMD_STATUS_REG4      0x0C  //high signs, x&y&z tapsense
#define BMA180_CMD_RESET            0x10 
/* reset register: set to 0 for soft reset -all register values will reset*/
#define BMA180_CMD_CTRL_REG0        0x0D // EE_W enable write control is bit 4 of this register address!
#define BMA180_CMD_CTRL_REG1        0x0E // contains the offsets for x, y,z
#define BMA180_CMD_CTRL_REG2        0x0F // unlocking eeprom register
//#define BMA180_CMD_CTRL_REG3        0x21 // configure different interrupt trigger bits in this register
//#define BMA180_CMD_CTRL_REG4        0x22 // low_hy, mot_cd_r, ff_cd_tr, offset_finetuning
/* CTRL_REGISTER 0 BIT MASKS - the really important ones! see page 26 for defaults*/
#define ctrl_reg0_dis_wake_up_MASK  B00000001  
/* BIT(0) set to 1 and unit sleeps automatically for wake_up_dur (7.7.9) then takes readings, 
 set this register bit to 0 to disable the automatic sleep&wake-up mode */
#define ctrl_reg0_sleep_MASK        B00000010  
/* BIT(1) chip will sleep if this bit set to 1 and wake when set to 0 
 Sleep bit should not be set to "1", when wake up mask is set to "1",  wait 10ms before any eeprom operation on wake from sleep*/
#define ctrl_reg0_ee_w_MASK         B00010000  
/* BIT(4) set this to 1 to Unlock writing to addr from 0x20 to 0x50, (default)=0 on reset
 7.10.3 ee_w  This bit must be written 1 to be able to write anything into image registers, resetting ee_w to 0 will prevent any register updates*/
// No serial transaction should occur within minimum 10 us after soft_reset command
/* BMA180_CTRL_REG3 MASK */
//#define ctrl_reg3_new_data_int_MASK  0x01  /* BIT(1) Set to 1, for Intrupt to occur when new accel data is ready in all three channels */


void initBMA180() {  

#ifdef ECHO_TO_SERIAL
  Serial.println(F("Initializing BMA180 accelerometer..."));
  delay(10);
#endif

  i2c_writeRegByte(BMA180_ADDRESS,BMA180_CMD_RESET,0xB6);  //B6 (hex) forces the reset - can be done before the ee bit is set?
  Serial.print(F("Soft Reset "));
  delay(10); //delay serial comms after reset  
  //Control, status & image registers are reset to values stored in the EEprom. 
  //puts the BMA in wake-up mode & default low noise mode "00": BW=1200 Noise 150 ug/rt pg28

  int id = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_CHIP_ID);
  if(id == 0x03)
  {
#ifdef ECHO_TO_SERIAL
    Serial.print(F("BMA180 Chip found at: ")); 
    Serial.print(id);
    delay(10); 
#endif
    if (i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG0,1, ctrl_reg0_ee_w_MASK) == 0) //enable register writing
    {
#ifdef ECHO_TO_SERIAL
      Serial.print(F("  BMA180 Write Init Pass"));
      delay(10);
#endif

      // disable wakeup mode because we will be sleeping the sensor manually
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG0,0, ctrl_reg0_dis_wake_up_MASK);  
#ifdef ECHO_TO_SERIAL
      Serial.print(F("  Wake up mode disabled, "));
#endif

      // Connect to the bw_tcs register and set the BW filtering level to 10Hz, Only bits 4-7 of the register hold this data
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_BW_TCS,BMA180_BW_10HZ,cmd_bandwidth_MASK);
#ifdef ECHO_TO_SERIAL
      Serial.print(F("  Filtering level set to 10HZ, "));
      delay(10);
#endif

      // Connect to the offset_lsb1 register and set the range
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_RANGEnSMP,BMA180_RANGE_1G,range_MASK);

#ifdef ECHO_TO_SERIAL
      Serial.print(F("  Range set to 1G,"));
      delay(10);
#endif
      // since this is a tilt sensing application, I am using the 1g range, which is factory calibrated
      // To enable the factory calibrated offset registers to be used by the sensors DAC
      // en_offset_x, en_offset_y, en_offset_z control bits are set to 1 
      // p49: to regulate all axis, it is necessary to enable the en_offset bits sequentially

      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG1,0,B10000000); //en_offset_x
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG1,0,B01000000); //en_offset_y
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG1,0,B00100000); //en_offset_z
#ifdef ECHO_TO_SERIAL
      Serial.println(F("  Factory offsets enabled,"));
      delay(30);
#endif
      //BMAtemperature = i2c_readReg(BMA180, BMA180_CMD_TEMP);
      //Serial.print("Temperature =  ");Serial.println(BMAtemperature);

      //final step in setup is to disable register writing
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG0,0, ctrl_reg0_ee_w_MASK);
      delay(15);

    }
    else
    {  
      Serial.println(F(" BMA180 Write Init Fail! "));
      delay(10); 
      error();
    }
  }
  else
  {
    Serial.println(F(" BMA180 Chip Detect Fail! "));
    delay(10); 
    error();
  }

  readBMA180();  //just to get the sensor arrays loaded

#ifdef ECHO_TO_SERIAL
  Serial.println(F("...BMA180 has been initialized"));
  delay(10);
#endif 
}


void readBMA180() 
{ 
  // read in the 3 axis data, each one is 14 bits = +- 16,383 for integer values
  // note negative values in the directions of the arrows printed on the breakout board!

  for (int thisReading = 0; thisReading < filterSamples; thisReading++){  //fill the smoothing arrays

    bytebuffer = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_X_MSB);
    rawACCx[thisReading] = bytebuffer << 8; // puts the most sig bits on the corect side - I am reading 14 bits total
    bytebuffer = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_X_LSB); 
    rawACCx[thisReading] |= bytebuffer >> 2; //this shift gets rid of two non-value bits in LSB register

    bytebuffer = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Y_MSB);
    rawACCy[thisReading] = bytebuffer << 8;
    bytebuffer = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Y_LSB);
    rawACCy[thisReading] |= bytebuffer >> 2; // what about adding the offset here?

    bytebuffer = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Z_MSB);
    rawACCz[thisReading] = bytebuffer << 8;
    bytebuffer = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Z_LSB);
    rawACCz[thisReading] |= bytebuffer >> 2;

    // we have the internal BMA bandwith filter set to 10 Hz so its not going to give new data without some time!
    setWTD_125ms();
    sleepNwait4WDT();

  }
  // Now send those readings out to the digital smoothing function
  smoothACCx = digitalSmooth(rawACCx);
  smoothACCy = digitalSmooth(rawACCy);
  smoothACCz = digitalSmooth(rawACCz);
}

#endif


// ************************************************************************************************************
// I2C Accelerometer BMA 250    10 bit (+- 1023)
// ************************************************************************************************************
// see http://www.bosch-sensortec.com/de/homepage/products_3/3_axis_sensors/acceleration_sensors/bma250_1/bma250
// for more registers http://asf.atmel.com/docs/latest/xmegaau/html/bma250_8h.html

#if defined(BMA250_ADDRESS)
//#define BMA250_FIXED_DEVID		0x03
//#define BMA250_REG_DEVID		0x00
//#define BMA250_REG_OFSX		0x16
//#define BMA250_REG_OFSX_HIGH		0x1A
#define BMA250_REG_BW_RATE		0x10
//#define BMA250_BW_MASK		0x1f
//#define BMA250_BW_200HZ		0x0d
//#define BMA250_BW_100HZ		0x0c
//#define BMA250_BW_50HZ		0x0b
//#define BMA250_BW_25HZ		0x0a
#define BMA250_BW_8HZ                   0x08 
//#define BMA250_REG_POWER_CTL		0x11		
#define BMA250_REG_DATA_FORMAT		0x0f
//#define BMA250_RANGE_MASK		0x0f
#define BMA250_RANGE_2G			0x03
//#define BMA250_RANGE_4G		0x05
//#define BMA250_RANGE_8G		0x08
//#define BMA250_RANGE_16G		0x0C
//#define BMA250_REG_DATAXLOW		0x02
//#define BMA250_REG_DATA_RESOLUTION	0x14
//#define BMA250_MEASURE_MODE		0x80	
//#define BMA250_SELF_TEST           	0x32
//#define BMA250_SELF_TEST_AXIS_X	0x01
//#define BMA250_SELF_TEST_AXIS_Y	0x02
//#define BMA250_SELF_TEST_AXIS_Z	0x03
//#define BMA250_SELF_TEST_POSITIVE	0x00
//#define BMA250_SELF_TEST_NEGATIVE	0x04
//#define BMA250_INT_REG_1           	0x16
//#define BMA250_INT_REG_2          	0x17
#define BMA250_CMD_ACC_X_LSB        0x02  /* First of 6 registers of accel data */
#define BMA250_CMD_ACC_X_MSB        0x03
#define BMA250_CMD_ACC_Y_LSB        0x04
#define BMA250_CMD_ACC_Y_MSB        0x05
#define BMA250_CMD_ACC_Z_LSB        0x06
#define BMA250_CMD_ACC_Z_MSB        0x07

byte initBMA250()
{
#ifdef ECHO_TO_SERIAL
  Serial.println(F("Init BMA250..."));
#endif 

  i2c_writeRegByte(BMA250_ADDRESS,BMA250_REG_DATA_FORMAT,BMA250_RANGE_2G);  //select 2G range
  i2c_writeRegByte(BMA250_ADDRESS,BMA250_REG_BW_RATE,BMA250_BW_8HZ);  //actually 7.81Hz bandwith, 64ms update

#ifdef ECHO_TO_SERIAL
  Serial.println(F("...BMA250 accelerometer has been initialized"));
#endif 

  return(0);
}

// from BMA250_I2C_Sketch.pde -BMA250 Accelerometer using I2C from www.dsscircuits.com/accelerometer-bma250.html
byte readBMA250()
{

  for (int thisReading = 0; thisReading < filterSamples; thisReading++){  //fill the smoothing arrays

    bytebuffer = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_X_MSB);
    rawACCx[thisReading] = bytebuffer << 8; // puts the most sig bits on the corect side
    bytebuffer = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_X_LSB); 
    rawACCx[thisReading] |= bytebuffer >> 6; //this shift gets rid of 6 non-value bits in LSB register

    bytebuffer = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_Y_MSB);
    rawACCy[thisReading] = bytebuffer << 8;
    bytebuffer = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_Y_LSB);
    rawACCy[thisReading] |= bytebuffer >> 6; // what about adding the offset here?

    bytebuffer = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_Z_MSB);
    rawACCz[thisReading] = bytebuffer << 8;
    bytebuffer = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_Z_LSB);
    rawACCz[thisReading] |= bytebuffer >> 6;

    //delay(135); // we have the internal BMA bandwith filter set to 7.81 Hz so its not going to give new data without some time!
    setWTD_125ms();  //might need a touch more time...
    sleepNwait4WDT();
  }
  // Now send those readings out to the digital smoothing function - probably dont need to smooth as much as BMA180
  smoothACCx = digitalSmooth(rawACCx);
  smoothACCy = digitalSmooth(rawACCy);
  smoothACCz = digitalSmooth(rawACCz);
}
#endif

// ************************************************************************************************************
// Accelerometer SMOOTHING FUNCTION
// ************************************************************************************************************
// this smoothing function based on Paul Badger's  http://playground.arduino.cc/Main/DigitalSmooth
// "int *inputArray" passes an array to the function - the asterisk indicates the array name is a pointer
int digitalSmooth(int *inputArray){     
  int j, k, temp, top, bottom;
  long total;
  static int i;
  boolean done;

  done = 0;                // flag to know when we're done sorting              
  while(done != 1){        // simple swap sort, sorts numbers from lowest to highest
    done = 1;
    for (j = 0; j < (filterSamples - 1); j++){
      if (inputArray[j] > inputArray[j + 1]){     // numbers are out of order - swap
        temp = inputArray[j + 1];
        inputArray [j+1] =  inputArray[j] ;
        inputArray [j] = temp;
        done = 0;
      }
    }
  }

#ifdef ECHO_TO_SERIAL
  Serial.println(F("Sorted Raw ACC readings:"));
  for (j = 0; j < (filterSamples); j++){    // print the array for debugging
    Serial.print(inputArray[j]); 
    Serial.print(F("   ")); 
  }
  Serial.println(F(" "));
#endif

  // throw out top and bottom 15% of samples - limit to throw out at least one from top and bottom
  bottom = max(((filterSamples * 15)  / 100), 1); 
  top = min((((filterSamples * 85) / 100) + 1  ), (filterSamples - 1));   // the + 1 is to make up for asymmetry caused by integer rounding
  k = 0;
  total = 0;
  for ( j = bottom; j< top; j++){
    total += inputArray[j];  // total remaining indices
    k++; 
    // Serial.print(sensSmoothArray[j]);Serial.print("   "); //more debugging
  }
  //  Serial.println(); Serial.print("average = "); Serial.println(total/k); //more debugging
  return total/k;    // divide by number of samples

}

// ************************************************************************************************************
// MS5803 functions   sensor spec http://www.meas-spec.com/downloads/MS5803-02BA.pdf
// ************************************************************************************************************
//
// ALL of these MS5803 functions are based on Luke Millers libraries at  https://github.com/millerlp?tab=repositories
// The only reason I re-juggled his excellent 2Bar and 5Bar sensor libraries into the combined code here (in June 2014), is because I did not have enough free ram left in my 
// script to load even one of original the libraries. Using his code here means that I must release this version of the Cave Pearl codebase under 
// GPL license, https://github.com/millerlp/MS5803_02/blob/master/LICENSE.md  and I am cool with that.  

#if defined(MS5803_I2C_ADDRESS)

void sendCommandMS5803(byte command){
  Wire.beginTransmission(0x76); //MS5803-02 i2c address, same for all?
  Wire.write(command);
  Wire.endTransmission();
}


//=======================================================

boolean initializeMS_5803(boolean Verbose) {
  Wire.begin();
  // Reset the sensor during startup
  resetSensor(); 

  if (Verbose) {
    // Display the oversampling resolution or an error message
    Serial.print(F("Oversampling setting: "));
    Serial.println(_Resolution);    		
  }

  // Read sensor coefficients
  for (int i = 0; i < 8; i++ ){
    // The PROM starts at address 0xA0
    Wire.beginTransmission(MS5803_I2C_ADDRESS);
    Wire.write(0xA0 + (i * 2));
    Wire.endTransmission();
    Wire.requestFrom(MS5803_I2C_ADDRESS, 2);
    while(Wire.available()) {
      HighByte = Wire.read();
      LowByte = Wire.read();
    }
    sensorCoeffs[i] = (((unsigned int)HighByte << 8) + LowByte);
    if (Verbose){
      // Print out coefficients 
      Serial.print("C");
      Serial.print(i);
      Serial.print(" = ");
      Serial.println(sensorCoeffs[i]);
      delay(10);
    }
  }
  // The last 4 bits of the 7th coefficient form a CRC error checking code.
  unsigned char p_crc = sensorCoeffs[7];
  // Use a function to calculate the CRC value
  unsigned char n_crc = MS_5803_CRC(sensorCoeffs); 

  if (Verbose) {
    Serial.print(F("p_crc: "));
    Serial.println(p_crc);
    Serial.print(F("n_crc: "));
    Serial.println(n_crc);
  }
  // If the CRC value doesn't match the sensor's CRC value, then the 
  // connection can't be trusted. Check your wiring. 
  if (p_crc != n_crc) {
    return false;
  }
  // Otherwise, return true when everything checks out OK. 
  return true;
}

//------------------------------------------------------------------
void readSensor() {
  // Choose from CMD_ADC_256, 512, 1024, 2048, 4096 for mbar resolutions
  // of 1, 0.6, 0.4, 0.3, 0.2 respectively. Higher resolutions take longer
  // to read.
  if (_Resolution == 256){
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_256); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_256); // read raw temperature	
  } 
  else if (_Resolution == 512) {
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_512); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_512); // read raw temperature		
  } 
  else if (_Resolution == 1024) {
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_1024); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_1024); // read raw temperature
  } 
  else if (_Resolution == 2048) {
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_2048); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_2048); // read raw temperature
  } 
  else if (_Resolution == 4096) {
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_4096); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_4096); // read raw temperature
  }

  // Calculate 1st order temperature, dT is a long integer
  // D2 is originally cast as an uint32_t, but can fit in a int32_t, so we'll
  // cast both parts of the equation below as signed values so that we can
  // get a negative answer if needed

  dT = (int32_t)D2 - ( (int32_t)sensorCoeffs[5] * 256 );

  // Use integer division to calculate TEMP. It is necessary to cast
  // one of the operands as a signed 64-bit integer (int64_t) so there's no 
  // rollover issues in the numerator.

  TEMP = 2000 + ((int64_t)dT * sensorCoeffs[6]) / 8388608LL;   //this line would not compile until I declared TEMP to be int64_t at start
  // Recast TEMP as a signed 32-bit integer
  TEMP = (int32_t)TEMP;

  /***********************************************************************************************
   * //  NOTE: I do not have enough ram left in this CODEBUILD for ANY second order calculations! (Needs >6K! free)
   * //  Just including them here for completeness since I will need to do these later in excel...
   *************************************************************************************************/

#if defined(MS5803_02_ISON)  //calculations specific to the 2 bar sensor

  /* 
   for the MS5803-02 sensor:
   // All operations from here down are done as integer math until we make the final calculation of pressure in mbar.     
   // 2nd order temperature compensation (see pg 9 of MS5803 data sheet)
   // I have tried to insert the fixed values wherever possible (i.e. 2^31 is hard coded as 2147483648).
   if (TEMP < 2000) {
   // For 2 bar model
   T2 = ((uint64_t)dT * dT) / 2147483648 ; // 2^31 = 2147483648
   T2 = (int32_t)T2; // recast as signed 32bit integer
   OFF2 = (61 * ((TEMP-2000) * (TEMP-2000))) / 16 ;
   Sens2 = 2 * ((TEMP-2000) * (TEMP-2000)) ;
   } 
   else { // if TEMP is > 2000 (20.0C)
   // For 2 bar model
   T2 = 0;
   OFF2 = 0;
   Sens2 = 0;
   }
   // Additional compensation for very low temperatures (< -15C)
   if (TEMP < -1500) {
   // For 2 bar model
   OFF2 = OFF2 + 20 * ((TEMP+1500)*(TEMP+1500));
   Sens2 = Sens2 + 12 * ((TEMP+1500)*(TEMP+1500));
   }  
   */

  // Calculate initial Offset and Sensitivity For 2 bar sensor
  // THESE CALCS TAKE 1K OF PROG MEM TO PERFORM!
  Offset = (int64_t)sensorCoeffs[2] * 131072 + (sensorCoeffs[4] * (int64_t)dT) / 64; 
  Sensitivity = (int64_t)sensorCoeffs[1] * 65536 + (sensorCoeffs[3] * (int64_t)dT) / 128;

#endif

#if defined(MS5803_05_ISON)  //calculations specific to the 5 bar sensor
  /*    // Do 2nd order temperature compensation (see pg 9 of MS5803-05 data sheet)
   if (TEMP < 2000) {
   		// For 5 bar model
   		// If temperature is below 20.0C
   		T2 = 3 * ((uint64_t)dT * dT)  / POW_2_33 ; // 2^33 = 8589934592
   		T2 = (int32_t)T2; // recast as signed 32bit integer
   		OFF2 = 3 * ((TEMP-2000) * (TEMP-2000)) / 8 ;
   		Sens2 = 7 * ((TEMP-2000) * (TEMP-2000)) / 8 ;
   } else { // if TEMP is > 2000 (20.0C)
   		// For 5 bar sensor
   		T2 = 0;
   		OFF2 = 0;
   		Sens2 = 0;
   }
   // Additional compensation for very low temperatures (< -15C)
   if (TEMP < -1500) {
   		// For 5 bar sensor
   		// Leave OFF2 alone in this case
   		Sens2 = Sens2 + 3 * ((TEMP+1500)*(TEMP+1500));
   }
   */
  // Calculate initial Offset and Sensitivity For 5 bar sensor - these two lines take 1K of prog space!

  Offset = (int64_t)sensorCoeffs[2] * 262144 + (sensorCoeffs[4] * (int64_t)dT) / 32;
  Sensitivity = (int64_t)sensorCoeffs[1] * 131072 + (sensorCoeffs[3] * (int64_t)dT) / 128;

#endif 

  /* 
   // Adjust TEMP, Offset, Sensitivity values based on the 2nd order 
   // temperature correction above. (But Since I did not calculate any of the 2nd order values...skiping this)
   TEMP = TEMP - T2; // both should be int32_t
   Offset = Offset - OFF2; // both should be int64_t
   Sensitivity = Sensitivity - Sens2; // both should be int64_t
   */

  // Final compensated pressure calculation. We first calculate the pressure
  // as a signed 32-bit integer (mbarInt), then convert that value to a
  // float (mbar). [same calc for the 2 & 5 bar sensor...]

  // THESE FIRST ORDER CALCS TAKE 1K OF PROG MEM TO PERFORM!
  mbarInt = ((D1 * Sensitivity) / 2097152 - Offset) / 32768;
  mbar = (float)mbarInt / 100;    
  tempC  = (float)TEMP / 100; 

  /* // Start other temperature conversions by converting mbar to psi absolute
   //    psiAbs = mbar * 0.0145038;
   //    // Convert psi absolute to inches of mercury
   //    inHgPress = psiAbs * 2.03625;
   //    // Convert psi absolute to gauge pressure
   //    psiGauge = psiAbs - 14.7;
   //    // Convert mbar to mm Hg
   //    mmHgPress = mbar * 0.7500617;
   //    // Convert temperature to Fahrenheit
   //    tempF = (tempC * 1.8) + 32;
   */
}

//------------------------------------------------------------------
// Function to check the CRC value provided by the sensor against the 
// calculated CRC value from the rest of the coefficients. 
// Based on code from Measurement Specialties application note AN520
// http://www.meas-spec.com/downloads/C-Code_Example_for_MS56xx,_MS57xx_%28except_analog_sensor%29_and_MS58xx_Series_Pressure_Sensors.pdf
unsigned char MS_5803_CRC(unsigned int n_prom[]) {
  int cnt;				// simple counter
  unsigned int n_rem;		// crc reminder
  unsigned int crc_read;	// original value of the CRC
  unsigned char  n_bit;
  n_rem = 0x00;
  crc_read = sensorCoeffs[7];		// save read CRC
  sensorCoeffs[7] = (0xFF00 & (sensorCoeffs[7])); // CRC byte replaced with 0
  for (cnt = 0; cnt < 16; cnt++)
  { // choose LSB or MSB
    if (cnt%2 == 1) {
      n_rem ^= (unsigned short)((sensorCoeffs[cnt>>1]) & 0x00FF);
    }
    else {
      n_rem ^= (unsigned short)(sensorCoeffs[cnt>>1] >> 8);
    }
    for (n_bit = 8; n_bit > 0; n_bit--)
    {
      if (n_rem & (0x8000))
      {
        n_rem = (n_rem << 1) ^ 0x3000;
      }
      else {
        n_rem = (n_rem << 1);
      }
    }
  }
  n_rem = (0x000F & (n_rem >> 12));// // final 4-bit reminder is CRC code
  sensorCoeffs[7] = crc_read; // restore the crc_read to its original place
  // Return n_rem so it can be compared to the sensor's CRC value
  return (n_rem ^ 0x00); 
}

//-----------------------------------------------------------------
// Send commands and read the temperature and pressure from the sensor
unsigned long MS_5803_ADC(char commandADC) {
  // D1 and D2 will come back as 24-bit values, and so they must be stored in 
  // a long integer on 8-bit Arduinos.
  long result = 0;
  // Send the command to do the ADC conversion on the chip
  Wire.beginTransmission(MS5803_I2C_ADDRESS);
  Wire.write(CMD_ADC_CONV + commandADC);
  Wire.endTransmission();
  // Wait a specified period of time for the ADC conversion to happen
  // See table on page 1 of the MS5803 data sheet showing response times of
  // 0.5, 1.1, 2.1, 4.1, 8.22 ms for each accuracy level. 
  switch (commandADC & 0x0F) 
  {
  case CMD_ADC_256 :
    delay(1); // 1 ms
    break;
  case CMD_ADC_512 :
    delay(3); // 3 ms
    break;
  case CMD_ADC_1024:
    delay(4);
    break;
  case CMD_ADC_2048:
    delay(6);
    break;
  case CMD_ADC_4096:
    delay(10);
    break;
  }
  // Now send the read command to the MS5803 
  Wire.beginTransmission(MS5803_I2C_ADDRESS);
  Wire.write((byte)CMD_ADC_READ);
  Wire.endTransmission();
  // Then request the results. This should be a 24-bit result (3 bytes)
  Wire.requestFrom(MS5803_I2C_ADDRESS, 3);
  while(Wire.available()) {
    HighByte = Wire.read();
    MidByte = Wire.read();
    LowByte = Wire.read();
  }
  // Combine the bytes into one integer
  result = ((long)HighByte << 16) + ((long)MidByte << 8) + (long)LowByte;
  return result;
}

//----------------------------------------------------------------
// Sends a power on reset command to the MS5803 sensor.
void resetSensor() {
  Wire.beginTransmission(MS5803_I2C_ADDRESS);
  Wire.write(CMD_RESET);
  Wire.endTransmission();
  delay(10);
}

#endif


/*****************************************************
 * RETIRED FUNCTIONS
 ******************************************************/
 
 /* the ds3231 RTC temp reading routine is now retired from the main codebuild, just including it here in case someone else wants it
//the following variables should be defined at start for this function
//byte tMSB, tLSB; float RTCTempfloat; int temp3231; uint8_t wRTCtemp,fRTCtemp;
// also see https://github.com/mizraith/RTClib  or https://github.com/akafugu/ds_rtc_lib for more DS3231 specific libs
 could also use RTC.getTemperature() from the library here as in:
 RTC.convertTemperature();             //convert current temperature into registers
 Serial.print(RTC.getTemperature());   //read registers and display the temperature */
// get temp from  http://forum.arduino.cc/index.php/topic,22301.0.html
//RTCTempfloat= get3231Temp(); wRTCtemp = (int)RTCTempfloat; fRTCtemp= (RTCTempfloat - wRTCtemp) * 100; // Float split into 2 intergers

/*float get3231Temp()
 {
 //temp registers (11h-12h) get updated automatically every 64s
 Wire.beginTransmission(DS3231_ADDRESS);
 Wire.write(0x11);
 Wire.endTransmission();
 Wire.requestFrom(DS3231_ADDRESS, 2);
 
 if(Wire.available()) {
 tMSB = Wire.read(); //2's complement int portion
 tLSB = Wire.read(); //fraction portion
 
 temp3231 = ((((short)tMSB << 8 | (short)tLSB) >> 6) / 4.0); 
 // Allows for readings below freezing - Thanks to Coding Badly
 //temp3231 = (temp3231 * 1.8 + 32.0); // Convert Celcius to Fahrenheit
 return temp3231;
 
 }
 else {
 temp3231 = 255.0;  //Use a value of 255 as error flag
 }
 
 return temp3231;
 }
 */


/**********************************************
 * old ERROR HANDLER
 **********************************************
 * // always write error messages to the serial monitor but this routine wastes
 * // everything passed to the string from the original call is in sram!
 * 
 * void error(char *str) 
 * {
 * Serial.print(F("error in: "));
 * Serial.println(str);
/* this next statement will start an endless loop, basically stopping all
 * operation upon any error.  Change this behavior if you want. 
 * digitalWrite(RED_PIN, HIGH);
 * while (1);
 * }
 */

//************************************ 
 //MS5803-05 -OR- MS5803-02 init
 //************************************ 
/*  THIS is based on the OPENROV project code by WALT HOLM
// https://github.com/OpenROV/openrov-software/blob/master/arduino/Test%20Code/Walts_Depth_Sensor_Rev1.ino

 #if defined(MS5803_02_ADR) || defined(MS5803_05_ADR)
 
 // Reset the device and check for device presence  // left at default 512 oversampling after reset
 sendCommandMS5803(Reset);
 delay(1000); //how long does this delay have to be???
 setWTD_125ms(); //use the sleep instead of delays
 sleepNwait4WDT();
 Serial.println(F("MS5803 is reset"));
 
 // Get the calibration constants and store in array  (note this code is from 02 bar sensor!)
 for (byte i = 0; i < 8; i++)
 {
 sendCommandMS5803(PromBaseAddress + (2*i));
 Wire.requestFrom(0x76, 2); //dev address is 0x76
 while(Wire.available()){
 ByteHigh = Wire.read();
 ByteLow = Wire.read();
 }
 CalConstant[i] = (((unsigned int)ByteHigh << 8) + ByteLow);
 } 
 
 #ifdef ECHO_TO_SERIAL
 Serial.println(F("Ms5803-2 Pressure sensor Calibration constants are:"));
 for (byte i=0; i < 8; i++){ 
 Serial.print(F("C"));
 Serial.print(i);
 Serial.print(F(":,"));
 Serial.println(CalConstant[i]);
 }
 Serial.println(F(" "));
 #endif
 
 // could do some crc checking here
 
 #endif
 */

/*
 //Read Pressure Sensor: Based on the OPEN ROV code
 #if defined(MS5803_02_ADR) || defined(MS5803_05_ADR)
 
 // Read the Ms5803 for the RAW ADC Temperature and ADC Pressure values, just read the raw D1 &D2 values & Calculate later in Excel
 
 sendCommandMS5803(D1_512); //start D1 conversion
 digitalWrite(GREEN_PIN, HIGH);
 delay(10);// conversion delay for each oversample ratio is  1.1 / 2.1 / 4.1 / 9 ms 
 digitalWrite(GREEN_PIN, LOW);
 sendCommandMS5803(AdcRead); 
 Wire.requestFrom(0x76, 3);
 while(Wire.available())
 {
 ByteHigh = Wire.read();
 ByteMiddle = Wire.read();
 ByteLow = Wire.read();
 }
 rawMS5803pressure = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow; 
 //24 bit ADC value for D1 & D2  max = 16777216 
 
 #ifdef ECHO_TO_SERIAL
 Serial.print(F("D1(Raw Pressure): ")); 
 Serial.print(rawMS5803pressure);
 #endif
 
 sendCommandMS5803(D2_512); //start D2 conversion
 delay(10); 
 sendCommandMS5803(AdcRead); 
 Wire.requestFrom(0x76, 3);
 while(Wire.available())
 {
 ByteHigh = Wire.read(); 
 ByteMiddle = Wire.read(); 
 ByteLow = Wire.read();
 }
 rawMS5803temp = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow;
 
 #ifdef ECHO_TO_SERIAL
 Serial.print(F(" D2(Raw Temp): ")); 
 Serial.println(rawMS5803temp);
 #endif
 
 #endif
 */










