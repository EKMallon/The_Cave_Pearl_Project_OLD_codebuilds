/*      Cave Pearl Datalogger script by Edward Mallon
 * 	 
 *       This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 *       without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 * 	 Licensed under the GPL v3 license. See http://www.gnu.org/licenses/ for details on reuse and redistribution.
 * 	
 * 	 This script incorporates the work of many other people for the libraries and some sensor functions. I have done my best 
 *       to site these sources directly in the comments with each section, and I make no claims whatsoever on those sections of code.
 *       That being said, the integration of these components into a functional datalogging system is a work that I have spent 
 *       MANY weeks of hard work on, expeciallly the data handling and overall coordinated operation, which are my contributions. 
 *       I would appreciate credit for that if you use this software directly, or as the basis for your own datalogging project. 
 *       I am happy for everyone to take this work as a starting point, and develop it in any way they see fit with one exception: 
 *       The GNU General Public License does not permit incorporating this program into proprietary programs.
 *
 *       This version is configured by default as DRIPSENSOR code, and assumes the following hardware
 *       MCU: Rocket scream ultra -or- Sparkfun Pro Mini-or- Generic 3.3v ProMini Clone
 *       Voltage divider:  RAW-10MΩ-A0-(10MΩ+0.1 µF)-GND  for the readExternalVcc function
 *       Ds3231 RTC with AT24c32eeprom, Adxl345 I2C accelerometer, 3 color LED on pins 4-6 with a massive 10k limit resistor
 *       Pololu Power Switch LV which will cut the power if you drive pin 9 high, without the switch the code will simply sleep on error
 *       A raspberry pi uSD card adapter connected to pins 10-13, with 50k pullup on unused connections
 *       (see more at the Cave Pearl Project website: http://edwardmallon.wordpress.com/category/drip-sensor-development/ for wiring conections) 
 * 
 *       AND Because everything is wrapped in  #ifdef--#endif  statements anyway
 *       I have left in the script support for the following sensors even though the drip sensor does not use any of them:
 *       HMC5883 Compass, DS18B20 Temp, TMP102 Temp, BMA180 Acc, BMA250 Acc, Ms5803-02 & 05 Pressure sensors
 *       simply UNCOMMENT defines at the beginning of the script to enaable each sensor you have connected - the rest of the code should
 *       adjust automatically...more or less... You will still have to edit some of the Pstring functions to make sure you are
 *       getting the right data saved to your SD cards.
 *
 *       also, I adopted the Rocket Screem Ultra sleep library, to turn off the BOD fuse while sleeping
 *       however I left in my older routine for non-RS watch dog time sleeping, as I am still not certain that turning off BOD is a good idea
 */




#include <Wire.h>       // I2C lib needs 128 byte Serial buffer
#include <SPI.h>        // not used here, but needed to prevent a RTClib compile error....grrr!
#include <RTClib.h>     // Date, Time and Alarm functions by https://github.com/MrAlvin/RTClib based largely on https://github.com/jcw
#include <LowPower.h>   // https://github.com/rocketscream/Low-Power
#include <avr/sleep.h> 
//#include <avr/power.h>// Power Reduction Register (PRR) settings
//#include <avr/wdt.h>  // re-enable this if you switch away from the RS sleep libraries
#include <PString.h>    // from  http://arduiniana.org/
#include <SdFat.h>      // needs 512 byte buffer,  from https://github.com/greiman/SdFat
SdFat sd; /*Create the objects to talk to the SD card*/
SdFile file;
const byte chipSelect = 10; //sd card chip select

//#define ECHO_TO_SERIAL   // for debugging  this takes about the same memory as doing the first order pressure calculations!
#define SampleIntervalMinutes 15   // power-down time in minutes before RTC interrupt triggers the next cycle
#define SampleIntSeconds 0 // this is ONLY used for DEBUGGING! otherwise SET to 0! Must Set minutes to zero for sub minute alarms to occur!

#define SamplesPerCycle 64  // MAX # of sample cycles to buffer in eeprom before sd card write 
//The AT25C32 is internally organized into (32,768 bits)=4096 bytes - beyond 4096 characters it rewrites over top of the data
//(128 x 32byte writes fill entire 4096 byte block) so MAX 64 wout compass (2 pgwrites/cycle) but max 42 if compass is installed! (3 pgwrites/cycle)
unsigned int countLogs = 0;      // # records have been written to each file so far
unsigned int fileInterval = 2880; // #of log records before new logfile is made usually 2880
/* count each time a log is written into each file.  Must be less than 65,535 counts per file.  If the sampleinterval is 15min,
 and fileInterval is 2880 seconds, then 96samples/day * 30days/month = 30 day intervals */
char FileName[] = "LOG00000.CSV";  //the first file name
byte Cycle=0; 

//****** AND ****** dont forget to edit the DATA file header in SETUP so that the config is written to the SD card file

//#define TS_TMP102  INSTALLED
//#define TS_DS18B20 INSTALLED

//#define BMA180_ADDRESS 0x40
//#define BMA250_ADDRESS 0x18

#define filterSamples   5  // #of ACCELEROMETER samples for filtering- odd number, no smaller than 3 - works great at 13 & good at 7-9 samples
// too many samples and you will run out of free ram!

//#define HMC5883_ADDRESS 0x1E

//#define MS5803_02_ISON 0x76  //(CSB pin 3 tied to Vdd)
//#define MS5803_05_ISON 0x76  //(CSB pin 3 tied to Vdd)
//#define MS5805_02_ISON 0x76  //drip sensor only
//#define firstOrderCalculation   // it adds 3k! -only define this if you have enough memory! otherwise just save raw D1 and D2 from the pressure sensors

#define ADXL345_ISON 0x53    //drip sensor is a special case without other sensors attached!
#define tapSensitivity 2  //might have to reduce this - some adxl's are twitchier than others

//#define unregulatedMCU 1  //Tinyduino can read vcc directly
#define vRegulatedMCU 1  //Pro Mini needs to use a voltage divider on analog pin 0
int Vcc = 9999; //the supply voltage (via 1.1 internal band gap OR analog read)
int Vcc2 = 9999; //the post SD card writing supply voltage, 9999 until first card write cycle
//int Vdelta;   //change in supply voltage after each SD write cycle...to track battery conditioin
#ifdef vRegulatedMCU
const float referenceVolts = 3.3;
const float resistorFactor = 511; // = 1023.0 * (R2/(R1 + R2));  if R1 and R2 are the same - max measurable input voltage of 6.6v
#endif

#define POWERDOWN_PIN 9 //which pin is driving the pololu powerswitch LV - if you are not using it then comment out this define
int CutoffVoltage = 3400;  //most of the regulated boards go down to 3400 mV but set this for the board you are using
int BlueVWarning = CutoffVoltage+300; //I have READSENSOR_PIN's led pip changing from Green -> Blue -> Red based on these numbers
int RedVWarning = CutoffVoltage+150;  //to warn you when the batteries are running low

char unitNumber[] = "DS03";
char loggerNumber[] = "DSDL03";

#ifdef ECHO_TO_SERIAL
//#define WAIT_TO_START
/* Wait for serial input in setup(), only if serial is enabled.  You don't want to define WAIT_TO_START unless ECHO_TO_SERIAL is defined, 
 because it would wait forever to start if you aren't using the serial monitor! */
#endif

// 3 color indicator LED pin connections- if there is only one led set all the defines to the same pin number
#define RED_PIN 4  
#define GREEN_PIN 5
#define BLUE_PIN 6

byte READSENSOR_PIN = GREEN_PIN;

//ds04, 01 02: 456

//Global variables
//******************
byte bytebuffer1 =0;
byte bytebuffer2 =0;
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
#define DS3231_ADDRESS 0x68                 //=104 dec
#define DS3231_STATUS_REG 0x0f
#define DS3231_CTRL_REG 0x0e
#define Bit0_MASK         B00000001        //Bit 0=Alarm 1 Flag (A1F)
#define Bit1_MASK         B00000010        //Bit 1 = Alarm 2 Flag (A2F)
#define Bit2_MASK         B00000100
#define Bit3_MASK         B00001000        //Bit 3: Enable/disable 32kHz Output (EN32kHz) - has no effect on sleep current
#define Bit4_MASK         B00010000        //Bit 4: Bits 4&5 of status reg adjust Time Between Temperature Updates  see http://www.maximintegrated.com/en/app-notes/index.mvp/id/3644
#define Bit5_MASK         B00100000        //Bit 5:
#define Bit6_MASK         B01000000
#define Bit7_MASK         B10000000
#define RTCPOWER_PIN 7                      //When the arduino is awake, power the rtc from this pin (draws ~70uA), when arduino sleeps pin set low & rtc runs on battery at <3uA
                                            // SEE http://www.gammon.com.au/forum/?id=11497 for an example powering ds1307 from pin, alarms still work!
RTC_DS3231 RTC;                            //DS3231 will function with a VCC ranging from 2.3V to 5.5V
byte Alarmhour = 1;
byte Alarmminute = 1;
byte Alarmday = 1;                         //only used for sub second alarms
byte Alarmsecond = 1;                      //only used for sub second alarms
byte INTERRUPT_PIN = 2;                    // SQW is soldered to this pin on the arduino
volatile boolean clockInterrupt = false;
char CycleTimeStamp[ ]= "0000/00/00,00:00"; //16 characters without seconds!
byte Startday;

// ACC common variables
// ********************
#if defined(BMA250_ADDRESS) || defined(BMA180_ADDRESS)
#define accelIsAttached //had to move this out for adxl
#endif

// this is alot of memory to eat up just for acclerometer smoothing
#if defined(BMA250_ADDRESS) || defined(BMA180_ADDRESS) || defined(ADXL345_ISON)  //***now ADXL uses these variables once a day too
int rawACCx [filterSamples];   // raw sensor values for x 
int rawACCy [filterSamples];   // raw sensor values for y 
int rawACCz [filterSamples];   // raw sensor values for z 

int smoothACCx = 0;  // smoothed x data
int smoothACCy = 0;  // smoothed y data
int smoothACCz = 0;  // smoothed z data
#endif

//ADXL345 I2C ACCelerometer used as a drip counter only!
//******************************************************
#ifdef ADXL345_ISON
#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_ACTIVIT_BIT 0x04
#define ADXL345_INT_SING_TAP_BIT 0x06
#define adxl345intPin 3 // The arduino pin (3) connected to INT 1 on ADXL345
int ADXL345_ADDRESS=0x53;
volatile boolean accInterrupt = false;
bool bitBack;      //byte bytebuffer1 =0; IN MAIN CODE
int intSource;  // I really only trac activity now, so could comment this out...
unsigned long tapcount=0; 
//"int" stores values -32768 to + 32767 and "unsigned int" 0 to +65535, so 65536 would require a "long"
// uint gives us a max of 72 counts per second if we are binning to 15 minute sample intervals
// longer intervals, say of an hour, might require us to go to "unsigned long" so the counter could go up to 4,294,967,295

//int x, y, z;
byte _buff[6] ;    //6 bytes buffer for x,y,z, read from the device using the OLD code - can be removed when you update it to standard register reads
//char str[512]; // Turns data into a string to send to Serial Port "BUFFER"

#endif

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
#ifdef HMC5883_ADDRESS
int CompassX = 0;
int CompassY = 0;
int CompassZ = 0;
#endif

// MS580X-0Y pressure sensors
// **************************

// D1 and D2 need to be unsigned 32-bit integers (long 0-4294967295)
uint32_t  D1 = 0;    // Store D1 value = uncompensated pressure value
uint32_t  D2 = 0;    // Store D2 value = uncompensated temperature value
int _ms580Xresolution = 512;  //MS580X sensor oversampling rate
//#define _ms580Xresolution = 512;  //oversampling rate  does not work as define?

#if defined(MS5803_02_ISON) || defined(MS5803_05_ISON) || defined(MS5805_02_ISON)

//variables for MS580X pressure sensor functions
#define MS580X_I2C_ADDRESS    0x76 // or 0x77 ( only 0x76 for MS5805)
#define CMD_RESET	0x1E	// ADC reset command
#define CMD_ADC_READ	0x00	// ADC read command
// The base address for conversion commands:
#define CMD_ADC_CONV	0x40	
//what to add to this base addresses for d1&d2 
#define CMD_ADC_D1	0x00	
#define CMD_ADC_D2	0x10	
//2,4,6,8 to add to the base address to change the resolution
#define CMD_ADC_256	0x00	// ADC resolution=256
#define CMD_ADC_512	0x02	// ADC resolution=512  //default after reset
#define CMD_ADC_1024	0x04	// ADC resolution=1024
#define CMD_ADC_2048	0x06	// ADC resolution=2048
#define CMD_ADC_4096	0x08	// ADC resolution=4096
//resolutions at these oversample ratios are  0.084 / 0.054 / 0.036 / 0.024 mbar //
// Create array to hold the 8 sensor calibration coefficients
unsigned int sensorCoeffs[8]; // unsigned 16-bit integer (0-65535) -ERROR only need 7 here?
// bytes to hold the results from I2C communications with the sensor
byte HighByte;
byte MidByte;
byte LowByte;

uint8_t wholeTempP=0;  //for breaking the decimal value into integers for Pstring print functions
uint8_t fracTempP=0;

bool verbose = true;

#ifdef firstOrderCalculation    // MIGHT NOT HAVE ENOUGH RAM FOR THESE CALCULATIONS DEPENDING ON HOW MANY SENSORS ATTACHED....
float mbar; // Store pressure in mbar. 
float tempC; // Store temperature in degrees Celsius
int wholembar=0;
int fracmbar=0;
//    float tempF; // Store temperature in degrees Fahrenheit
//    float psiAbs; // Store pressure in pounds per square inch, absolute
//    float psiGauge; // Store gauge pressure in pounds per square inch (psi)
//    float inHgPress;	// Store pressure in inches of mercury
//    float mmHgPress;	// Store pressure in mm of mercury

// These three variables are used for the conversion steps (which takes 3K more memory than I have left) generally I only use these during debugging...
// They should be signed 32-bit integer initially  i.e. signed long from -2147483648 to 2147483647
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

#endif


/**********************************************
 *  *   *   *   *   *   SETUP   *   *   *   *   *
 ***********************************************/
// errors in setup will always call error routine that halts the system
void setup () {
  pinMode(POWERDOWN_PIN, OUTPUT);
  digitalWrite(POWERDOWN_PIN, LOW);// driving this pin high shuts down the system
  pinMode(RTCPOWER_PIN, HIGH);
  digitalWrite(RTCPOWER_PIN, OUTPUT);// driving this high supplies power to the RTC Vcc pin while arduino is awake
  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, HIGH);//RTC is on pin (2) pull up the interrupt pin
#ifdef ADXL345_ISON  
  pinMode(adxl345intPin, INPUT);
  digitalWrite(adxl345intPin, HIGH);// ASDXL345 is on pin (3) pull up the interrupt pin
#endif
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
  i2c_writeRegBits(DS3231_ADDRESS,DS3231_STATUS_REG,0,Bit3_MASK); // disable the 32khz output  pg14-17 of datasheet  //This does not reduce the sleep current
  i2c_writeRegBits(DS3231_ADDRESS,DS3231_STATUS_REG,1,Bit4_MASK); // see APPLICATION NOTE 3644 - this might only work on the DS3234?
  i2c_writeRegBits(DS3231_ADDRESS,DS3231_STATUS_REG,1,Bit5_MASK); // setting bits 4&5 to 1, extends the time between RTC temp updates to 512seconds (from default of 64s)
  DateTime now = RTC.now();
  Startday = now.day();
  DateTime compiled = DateTime(__DATE__, __TIME__);  
  if (now.unixtime() < compiled.unixtime()) { //checks if the RTC is not set yet
    Serial.println(F("RTC is older than compile time! Updating"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
    Serial.println(F("Clock updated...."));
    DateTime now = RTC.now();
    Startday = now.day(); //get the current day for the error routine
  }

  // startup delays if needed
  //*************************
#ifdef ECHO_TO_SERIAL
  serial_boilerplate();
#endif

  delay(10000); // delay here just to prevent power stutters from writing multiple headers to the sd card

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
    Serial.println(F("ERROR: Did not find the DS18B20 Temp Sensor!"));
    return;
  }
  else 
  {
    Serial.print(F("DS18B20 found @ ROM addr:"));
    for(byte i = 0; i < 8; i++) {
      Serial.write(' ');
      Serial.print(addr[i], HEX);
    }
    Serial.println(F(" "));
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

#ifdef ADXL345_ISON 
  initADXL345();  
#endif

  //Compass init
  //**********************
#ifdef HMC5883_ADDRESS
  initHMC5883(); 
#endif


#if defined(MS580X_I2C_ADDRESS)
  // Initialize the MS5803 sensor. This will report the
  // conversion coefficients to the Serial terminal if present.
  // If you don't want all the coefficients printed out, 
  // set sensor.initializeMS_5803(false) otherwise send true

  if (initializeMS_5803(true)) {
    Serial.println(F("MS5803 CRC  OK." ));
  } 
  else {
    Serial.println(F("MS5803 CRC FAILED!") );
  }
#endif
    digitalWrite(GREEN_PIN, LOW);
  //get the SD card ready
  //**********************
  pinMode(chipSelect, OUTPUT);  //make sure that the default chip select pin is set to output, even if you don't use it 

#ifdef ECHO_TO_SERIAL
  Serial.print(F("Init SD card:")); 
#endif

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library. // change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
    Serial.println(F("Cound NOT initialize SD Card"));Serial.flush();
    error();
  }

#ifdef ECHO_TO_SERIAL
  Serial.println(F("card initialized."));
  Serial.print(F("The sample interval is: ")); 
  Serial.print(SampleIntervalMinutes); 
  Serial.print(F(" min."));
#ifdef SampleIntSeconds
  Serial.print(F(" ")); 
  Serial.print(SampleIntSeconds);
  Serial.print(F(" Sec.")); 
#endif
  Serial.println(F(" ")); 

#ifdef accelIsAttached
  Serial.println(F("YYYY/MM/DD HH:MM, Temp(C), Vcc(mV),  AccX = , AccY = , AccZ = , Vcc2 , Compass X, Compass Y, Compass Z,"));
#endif
#ifdef MS580X_I2C_ADDRESS  //if we are pressure sensing, the unit is stationary, and we dont have the acc or comass running
  Serial.println(F("YYYY/MM/DD HH:MM, Temp(C), Vcc(mV), D1(Raw MS580x Press), D2(Raw MS580x Temp), Vcc2 , 1st order Pres (mbar), 1st ord. Temp(C)"));
#endif
#ifdef ADXL345_ISON
  Serial.println(F("YYYY/MM/DD HH:MM, RTC Temp(C),External Vcc(mV), Drip Count ,Vcc(2) after Newfile Created"));
#endif

  Serial.flush();//delay(50); //short delay to clear serial com lines 
#endif

  // open the file for write at end like the Native SD library  
  // see http://forum.arduino.cc/index.php?topic=49649.0
  // O_CREAT = create the file if it does not exist
  if (!file.open(FileName, O_RDWR | O_CREAT | O_AT_END)) {
    Serial.println(F("1st open LOG.CSV fail"));
    error();
  }
// HEADER INFO
file.print(F("Unit#Drip sensor prototype 2"));
file.print(unitNumber);
file.print(F(", Data platform #"));
file.println(loggerNumber);

#ifdef unregulatedMCU
file.print(F(",TinyDuino,"));
#endif

#ifdef vRegulatedMCU
file.print(F(",ProMini,"));
#endif

#ifdef BMA180_ADDRESS
file.print(F("Bma180,"));
#endif
#ifdef HMC5883_ADDRESS
file.print(F("Bma250,"));
#endif

#ifdef TS_TMP102
file.print(F("TMP102,"));
#endif

#ifdef TS_DS18B20
file.print(F("DS18B20,"));
#endif

#ifdef HMC5883_ADDRESS
file.print(F("HMC5883,"));
#endif

#ifdef MS5803_02_ISON
file.print(F("MS5803_02,"));
#endif

#ifdef MS5803_05_ISON
file.print(F("MS5803_05,"));
#endif

#ifdef MS5805_02_ISON
file.print(F("MS5805_02,"));
#endif

#ifdef ADXL345_ISON
file.print(F("ADXL345 (DRIP) "));file.print(int(getRate()/2));file.print(F("Hz,"));
#endif

file.println(F(" "));

#if defined(MS580X_I2C_ADDRESS) //If you have a pressure sensor on, write cal constants to the sd card

  file.println(F("Ms580X Pressure sensor Calibration constants are:"));
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
  file.print(F(" min. "));
#ifdef SampleIntSeconds
  file.print(F(" ")); 
  file.print(SampleIntSeconds);
  file.print(F(" Sec. ")); 
#endif
  file.println(F(" "));

#ifdef accelIsAttached
  file.println(F("YYYY/MM/DD HH:MM, Temp(C), Vcc(mV), AccX = , AccY = , AccZ = , Vcc2 , Compass X, Compass Y, Compass Z,"));
#endif
#ifdef MS580X_I2C_ADDRESS  //if we are pressure sensing, the unit is stationary, and we dont have the acc or comass running
  file.println(F("YYYY/MM/DD HH:MM, Temp(C), Vcc(mV), D1(Raw MS580x Press), D2(Raw MS580x Temp), Vcc2 , 1st order Pres (mbar), 1st ord. Temp(C)"));
#endif
#ifdef ADXL345_ISON
  file.println(F("YYYY/MM/DD HH:MM, RTC Temp(C), Vcc(mV), Drip Count ,Vcc after Newfile Created"));
#endif

  file.println(F(" "));
  file.close();
  delay(100); //short delay to clear coms cant sleep here though...
  
  digitalWrite(GREEN_PIN, LOW);

  DIDR0 = 0x0F; // disable the digital inputs on analog 0..3 so we can use analog sensors  (analog 4&5 being used by I2C!) 
 
 //set any unused digital pins, that are NOT connected to ANYTHING to input HIGH so they dont float during sleep
 #ifndef ECHO_TO_SERIAL
 pinMode(0, INPUT); digitalWrite(0, HIGH);  //if we are on usb then these pins are RX & TX 
 pinMode(1, INPUT); digitalWrite(1, HIGH);
 #endif
 //2&3 are used as interrupts
 //4,5,6 used for RGB led
 pinMode(7, INPUT); digitalWrite(7, HIGH);
 pinMode(8, INPUT); digitalWrite(8, HIGH);
// pinMode(9, INPUT); digitalWrite(9, HIGH) dont set this if pin 9 is being used to trigger pololu power switch!
 //10,11,12,13 connected to the Sd card, SPI mode
 
 Vcc = readExternalVcc(); //first check of the system voltage
 if (Vcc < BlueVWarning){
      READSENSOR_PIN = BLUE_PIN;  //change sensor pip to blue as an early indicator that batteries have fallen low
      }
      if (Vcc < RedVWarning){
      READSENSOR_PIN = RED_PIN;  //If the sensor pip is now red, it is time to change the batteries.
      }
      
      if (Vcc < CutoffVoltage){  //The pro mini board vregulators need 3.35 v minimum for stable output
      #ifdef ECHO_TO_SERIAL
      Serial.println(F("Voltage too low for SD card writing!"));
      #endif
      error();
      }
 
}
/*******************************************************************************************************************************************
 *******************************************************************************************************************************************
 *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
 *******************************************************************************************************************************************
/******************************************************************************************************************************************/

// sensor errors during main loop should only call error routine & halt the system if ECHO_TO_SERIAL is defined (ie: we are in debug mode)
// that way if one sensor dies in the field we can still get data from the others
void loop () 
{ 
  
  // countLogs track how many lines have been written to the file
  // if it goes above your file interval set at the beginning, start a new file
  if(countLogs >= fileInterval){
    digitalWrite(RED_PIN, HIGH);
    createLogFile();   // create a new file this is the largest power using event!
    countLogs = 0;     // reset our counter to zero
    digitalWrite(RED_PIN, LOW);
  }

  CurrentPageStartAddress = 0; 

//this for loop wraps the "cycle of samples" repeating to the point where the eeprom is full
//--------------------------------------------------------------------------------------------------

  for (int Cycle = 0; Cycle < SamplesPerCycle; Cycle++) { //this counts from 0 to (SamplesPerCycle-1)

    if (clockInterrupt) {  // I know this is redundant...just put it here to make sure the RTC alarm is off
      clearClockTrigger(); 
      digitalWrite(INTERRUPT_PIN, HIGH);//set weak internal pull up the interrupt pin
    }

    digitalWrite(READSENSOR_PIN, HIGH);  //heartbeat pip to say we are taking a sample now
    DateTime now = RTC.now();  // Read the time and date from the RTC
    //old: sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    //Time read always occurs <1 sec after RTC interrupt, so seconds data was always "00" - so I dont record it any more

    sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute()); 

    //Vcc = readExternalVcc();  // now I am only reading this now before SD write operations - as there is a power wasting 3ms delay in this code
    //Vcc2 = readInternalVcc();  // 5ms delay -field runs only reading VCC2 after new file creation event
    digitalWrite(READSENSOR_PIN, LOW);  // end of heartbeat pip - might need to leave it on longer...
    
    if (Vcc2 < 3350){  // this check catches if the chip is seeing a voltage too low for the SD car writing
      #ifdef ECHO_TO_SERIAL
      Serial.println(F("Supply V TOO LOW!"));Serial.flush();
      #endif 
      error();
    }

// Now we read the sensors that are attached:
//--------------------------------------------------------------------------------------------------
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

#if defined(MS580X_I2C_ADDRESS)
    // Use readPressureSensor() function to get pressure and temperature reading. 
    readPressureSensor();
#endif

    //Read the temperature:
#ifdef TS_TMP102   
    TEMP_Raw =readTMP102();
    TEMP_degC =TEMP_Raw*0.0625;
#endif
    // or
#ifdef TS_DS18B20
    // Must read DS18B20 last due to 1 second of sleeping while waiting for it's high bit depth conversion! 
    TEMP_Raw =readDS18B20Temp();
    TEMP_degC =TEMP_Raw/16.00; 
#endif
    // If no other temperature sensor is attached, the RTC is the temperature sensor of last resort, its pretty crummy tho
#ifndef TS_TMP102 && TS_DS18B20 //&& MS580X_I2C_ADDRESS
    TEMP_degC = getRTCTemp(); 
#endif

    wholeTemp = (int)TEMP_degC; 
    fracTemp= (TEMP_degC - wholeTemp) * 100; // Float split into 2 intergers so print funtions dont eat memory

// This first section is serial output for debugging only  - comment out ECHO_TO_SERIAL to eliminate
//--------------------------------------------------------------------------------------------------
#ifdef ECHO_TO_SERIAL
    Serial.print(CycleTimeStamp); 
    Serial.print(F(" Cycle: ")); 
    Serial.print(Cycle);
#ifdef ADXL345_ISON  //If we are counting taps, the ACCelerometer is not making other readings, so we can sub in here.
    Serial.print(F(", DripCount:"));
    Serial.print(tapcount); 
#endif
    Serial.print(F(", Vcc= ")); 
    Serial.println(Vcc); 
#ifdef accelIsAttached
    Serial.print(F(", ACCx= ")); 
    Serial.print(smoothACCx); 
    Serial.print(F(", ACCy= ")); 
    Serial.print(smoothACCy); 
    Serial.print(F(", ACCz= "));
    Serial.println(smoothACCz); 
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

#if defined(MS580X_I2C_ADDRESS)
    Serial.print(F(" D1 = "));
    Serial.print(D1);
    Serial.print(F(" D2 = "));
    Serial.print(D2);
#ifdef firstOrderCalculation
    Serial.print(F(" 1stOrder Pressure = "));
    wholembar = (int)mbar; 
    fracmbar = (mbar - wholembar) * 100;
    Serial.print(wholembar);
    Serial.print(F("."));
    Serial.print(fracmbar);
    //Serial.print(mbar); // it takes one whole K to print floats! dont do it! - it eats sram! 
    Serial.print(F(" mbar,"));
    wholeTempP = (int)tempC; 
    fracTempP= (tempC - wholeTempP) * 1000;
    Serial.print(F(" & Temp: "));
    Serial.print(wholeTempP);
    Serial.print(F("."));
    Serial.println(fracTempP);
    //Serial.print(tempC);
#endif
#endif

    //delay(100); //short delay to clear serial com lines
#endif

//Construct first char string of 28 bytes - end of buffer is filled with blank spaces flexibly with pstring
//but could contruct the buffer with sprintf if I wasn't changing my sensors so often!
//--------------------------------------------------------------------------------------------------

    PString str(EEPROMBuffer, sizeof(EEPROMBuffer)); 
    str = CycleTimeStamp;           //17 / 16 characters without seconds plus comma
    str.print(F(","));
    //str.print(TEMP_Raw);  //4 digits both 12 bit sensors, so 4095 is largest value 
    str.print(wholeTemp);  //two digits positive
    str.print(F("."));
    str.print(fracTemp);   //3 digits, because *1000, but the third digit is just junk data
    str.print(F(",")); 
    str.print(Vcc);                 // 5 / 4 char typical: 4096,
    str.print(F("         "));  // just filler spaces for the end of the string to accomodate changes in data size
    //typical: 0000/00/00,00:00,4500,24.135

    digitalWrite(BLUE_PIN, HIGH); 
    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer); // whole page is written at once here
    CurrentPageStartAddress += EEPromPageSize;
    digitalWrite(BLUE_PIN, LOW);

//Construct second char string of 28 bytes to complete the record
//--------------------------------------------------------------------------------------------------
    str = ","; 

#ifdef accelIsAttached  // 20 characters of data incl signs commas for bma180, less for bma250
    str.print(smoothACCx);          // 6  5 or 6char:   +_1023 from BMA250  OR (+- 16,384) from BMA180
    str.print(F(","));
    str.print(smoothACCy);
    str.print(F(","));
    str.print(smoothACCz);
    str.print(F(","));
#endif

    //generally, you either have the acclerometer OR the pressure sensor, but not both at the same time
    //24 bit raw ADC value for D1 & D2  max = 16777216 = 8 characters! 9 with comma for each value!
#if defined(MS580X_I2C_ADDRESS)  // 18 characters of data incl commas
    str.print(D1); // 8 characters
    str.print(F(","));
    str.print(D2); 
    str.print(F(","));
#endif

#ifdef ADXL345_ISON  //If we are counting drips, the other ACCelerometer readings will be zero, so we can sub in here.
    str.print(tapcount); 
    str.print(F(","));
    tapcount=0; //once the data is in the string, you can reset the tap counter to zero
#endif
    str.print(Vcc2); //we have room left in this 28 char buffer, so why not record it too? Wont show up till first new file creation event though.
    str.print(F(","));
    str.print(F("                         "));
    //typical: ,-1255,-255,498,21.56  

    digitalWrite(BLUE_PIN, HIGH);
    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer); // 28 bytes/page is max whole page is written at once here
    CurrentPageStartAddress += EEPromPageSize;
    digitalWrite(BLUE_PIN, LOW);

//Construct third char string of 28 bytes IF you need to
//--------------------------------------------------------------------------------------------------

#if defined(firstOrderCalculation) || defined(HMC5883_ADDRESS)  // 1st order calcs on pressure or compass data means we need to construct a third string
    str = ",";

#if defined(HMC5883_ADDRESS)
    str.print(CompassX);  //max 18 characters from compass, usually less
    str.print(F(","));
    str.print(CompassY);
    str.print(F(","));
    str.print(CompassZ);
    str.print(F(","));
#endif
    // you only have room for MS5803 1st calc if there is no compass attached!!!!
#ifdef firstOrderCalculation
    str.print(wholembar);
    str.print(F("."));
    str.print(fracmbar);
    str.print(F(","));
    str.print(wholeTempP);
    str.print(F("."));
    str.print(fracTempP);
    str.print(F(","));
#endif

    str.print(F("                                          "));

    digitalWrite(BLUE_PIN, HIGH);
    Write_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer); // 28 bytes/page is max whole page is written at once here
    CurrentPageStartAddress += EEPromPageSize;
    digitalWrite(BLUE_PIN, LOW);
#endif

// Check if full set of sample cycles is complete (so eeprom is full), run a loop to dump data to the sd card
// BUT only if Vcc is above CutoffVoltage so we dont hurt the card
//--------------------------------------------------------------------------------------------------

    if (Cycle==(SamplesPerCycle-1) && Vcc >= CutoffVoltage){ 
      
      Vcc = readExternalVcc();  // Check psupply to make sure its ok to write to sd card
      if (Vcc < BlueVWarning){
      READSENSOR_PIN = BLUE_PIN;  //change sensor pip to blue as an early indicator that batteries have fallen low
      }
      if (Vcc < RedVWarning){
      READSENSOR_PIN = RED_PIN;  //If the sensor pip is now red, it is time to change the batteries.
      }
      
      if (Vcc < CutoffVoltage){  //The pro mini board vregulators need 3.35 v minimum for stable output
      #ifdef ECHO_TO_SERIAL
      Serial.println(F("Voltage too low for SD card writing!"));
      #endif
      error();
      }
 
      #ifdef ECHO_TO_SERIAL
      Serial.println(F(" --Writing to SDcard --")); 
      //Serial.print(F("Current eeprom page address:"));Serial.print(CurrentPageStartAddress ); delay (10);
      //Serial.print(F("-now set to zero  & Cycle:"));Serial.println(Cycle); delay (10);
      #endif

      file.open(FileName, O_RDWR | O_AT_END);
      // open the file for write at end like the Native SD library
      // if (!file.open(FileName, O_RDWR | O_AT_END)) {
      // error("L open file fail");
      //}
      
      CurrentPageStartAddress=0;  //reset the page counter back to the beginning of the eeprom stack
      for (int i = 0; i < SamplesPerCycle; i++) {  //loop to read from I2C ee and write to SD card 
        digitalWrite(RED_PIN, HIGH);
        Read_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer, sizeof(EEPROMBuffer) );  //there will be a few blank spaces
        CurrentPageStartAddress += EEPromPageSize;
        digitalWrite(RED_PIN, LOW); 
        file.write(EEPROMBuffer,sizeof(EEPROMBuffer));
        BytesWrittentoSD = BytesWrittentoSD + sizeof(EEPROMBuffer);

        digitalWrite(RED_PIN, HIGH);
        Read_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer, sizeof(EEPROMBuffer) );
        CurrentPageStartAddress += EEPromPageSize;
        digitalWrite(RED_PIN, LOW); 
        file.write(EEPROMBuffer,sizeof(EEPROMBuffer));
        BytesWrittentoSD = BytesWrittentoSD + sizeof(EEPROMBuffer);

#if defined(firstOrderCalculation)  || defined(HMC5883_ADDRESS)  //the compass or pressure sensor 1st ord calcas adds a third data line to each cycle
        digitalWrite(RED_PIN, HIGH);
        Read_i2c_eeprom_page(EEPROM_ADDRESS, CurrentPageStartAddress, EEPROMBuffer, sizeof(EEPROMBuffer) );
        CurrentPageStartAddress += EEPromPageSize;
        digitalWrite(RED_PIN, LOW); 
        file.write(EEPROMBuffer,sizeof(EEPROMBuffer));
        BytesWrittentoSD = BytesWrittentoSD + sizeof(EEPROMBuffer);
#endif

        file.println(F(" "));  //add a carridge return to the file
        BytesWrittentoSD = BytesWrittentoSD + 2;  //plus 2 for the space and c return

        countLogs++;
        // An application which writes to a file using print(), println() or write() must call sync()
        // at the appropriate time to force data and directory information to be written to the SD Card. approximately 512 bytes
        if(BytesWrittentoSD > 440) { 
          syncTheFile; 
          BytesWrittentoSD=0;
        } //I am probably doing this too often  (512-64=448)
      }

      //I used to check Vcc2 here, but thats been moved to New file creation event, so it does not give you any usefull data until the second sd card file
      
      file.close(); 
      //Run the SdFat bench example.  it will print the max latency.  http://forum.arduino.cc/index.php/topic,109862.0.html
      //A latency of over 300 ms is unusually long.  150-200 ms is more common. 
      CurrentPageStartAddress=0; //now that we have dumped the eeprom to the sd, we can start filling it from the beginning again
      //delay(10);  not sure if I need this delay or not to let the sd card close operation finish?
 
    }  //cycles per sample loop terminator for if (Cycle==(SamplesPerCycle-1))

// Now we need to set the next RTC alarm and go back to sleep
//------------------------------------------------------------------------

    // setNextAlarmTime();
    Alarmhour = now.hour(); 
    Alarmminute = now.minute()+SampleIntervalMinutes;
    Alarmday = now.day();
    if (SampleIntervalMinutes > 0) //then our alarm is in (SampleInterval) minutes
    {
      if (Alarmminute > 59) {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
        Alarmminute =0; 
        Alarmhour = Alarmhour+1; 
        if (Alarmhour > 23) {
          Alarmhour =0;
          
          // CAN place your ONCE-PER-DAY events in the code right here!
             
             #ifdef ADXL345_ISON  
             readAdxl345(); //I read the accelerometer xyz once per day to make sure the drip sensor has not moved or fallen over
             file.open(FileName, O_RDWR | O_AT_END);
             file.print(F(",,,,")); //just pushes this once per day line of data out of the way so I can sort the gaps out later in excel
             file.print(F(",ACCx=, ")); 
             file.print(smoothACCx); 
             file.print(F(",ACCy=, ")); 
             file.print(smoothACCy); 
             file.print(F(",ACCz=,"));
             file.print(smoothACCz); 
             file.print(F(","));
             file.println(CycleTimeStamp);
             file.close();
             # endif
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
            
             // CAN place ONCE-PER-DAY events in the code right here!
             
             #ifdef ADXL345_ISON  
             readAdxl345(); //I read the accelerometer xyz once per day to make sure the drip sensor has not moved or fallen over
             file.open(FileName, O_RDWR | O_AT_END);
             file.print(F(",,,,")); //just pushes this once per day line of data out of the way so I can sort the gaps out later in excel
             file.print(F(",ACCx=, ")); 
             file.print(smoothACCx); 
             file.print(F(",ACCy=, ")); 
             file.print(smoothACCy); 
             file.print(F(",ACCz=,"));
             file.print(smoothACCz); 
             file.print(F(","));
             file.println(CycleTimeStamp);
             file.close();
             # endif
             
             // sleep for a total of 64 seconds (12 x 8s) so the day "rolls over" while we are in this loop
             // yes I knwo this causes a gap in the timing, but I only use sub minute sampling for debug anyway.
             //int j;
             //for (j = 0; j <8; j++)
             //{
             /* //setWTD_8s();
             //sleepNwait4WDT(); */
             //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  //two lines above replaced by this
             //}
            //while more elegant the sleeping delay loop did not seem to work, so I am just going with the dumb method
            delay(327670);// Just wait for a 1.5 minutes to pass..
            delay(327670);
            delay(327670); 
            
            DateTime now = RTC.now();  //now set the alarm again with the day already rolled over
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
    delay(5);  //give the RTC a few ms to finish operations
    pinMode (RTCPOWER_PIN, INPUT);
    digitalWrite(RTCPOWER_PIN, LOW); // RTC vcc connected to this pin - driving this LOW FORCES to the RTC to draw power from the coin cell during sleep

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
    Serial.flush();
    //delay(50); //a delay long enought to boot out the serial coms 
#endif

//the following DRIP SENSOR ONLY loop continues updating the drip counter ("tapcount" in the sleepNwait4AccInterrupt() routine) 
//until a RTC interrupt occurs, which breaks out because clockInterrupt = true;
//none of the other datalogger configurations use this ADXL345 little loop!
//------------------------------------------------------------------------

#ifdef ADXL345_ISON  

    while(clockInterrupt == false){   

      // a tap just occured clear so need to read&clear the registers to turn off the interrupt signal
      intSource=i2c_readRegByte(ADXL345_ADDRESS,0x30); //read the ADXL345_INT_SOURCE interrupt register to clear it
      bytebuffer1=i2c_readRegByte(ADXL345_ADDRESS,0x2b); //read the ADXL345_ACT_TAP_STAT register to clear it
      //i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_INT_ENABLE, B00000000); // this disables all acc interupts
      //digitalWrite(adxl345intPin, HIGH);//now that the acc interupt is gone, set internal pull up back the ADXL interrupt pin
      //OR turn off the tap interrupts?
#ifdef ECHO_TO_SERIAL
      if(intSource & B00010000){
        Serial.print(F("*** Activity ***"));
        Serial.print(F("Tapcount:")); 
        Serial.println(tapcount);
      }
      //if(intSource & B01000000){Serial.println(F("*** Single Tap *** ")); }
      Serial.flush();//delay(30); //delay needed to clear coms before sleeping again
      // when running without echo to serial, unit will be faster!
#endif

      // Now we add a delay to the response to let system stop vibrating.
      // LENGTHEN this delay if you start getting "double" hits from single drips

      digitalWrite(READSENSOR_PIN, HIGH);  // WITH a 10K LIMIT RESISTOR, YOU CAN LEAVE THE TAP INDICATOR led ON DURING this short SLEEP
      //OR    
      //setWTD_16ms();  //limits to 1000/16 = 62 cycles per second: "ACTUAL" tested output goes down to 37.5 counts per sec with this delay
      //setWTD_32ms();  //limits to 1000/32 = 30 cycles per second: "ACTUAL" tested output goes down to 22.5 counts per sec with this delay
      //sleepNwait4WDT();
      
      //OR
      //LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF);
      
      LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
      digitalWrite(READSENSOR_PIN, LOW); //pip to show that a tap occured
      
      intSource = 0;      
      intSource=i2c_readRegByte(ADXL345_ADDRESS,0x30); //read the ADXL345_INT_SOURCE interrupt register to clear it in case an int was generated during the delay
      bytebuffer1=i2c_readRegByte(ADXL345_ADDRESS,0x2b); //read the ADXL345_ACT_TAP_STAT register to clear it
      digitalWrite(adxl345intPin, HIGH);//now that the acc interupt is gone, set internal pull up back the ADXL interrupt pin

      pinMode (RTCPOWER_PIN, INPUT);
      digitalWrite(RTCPOWER_PIN, LOW); // RTC vcc connected to this pin - driving this LOW FORCES to the RTC to draw power from the coin cell during sleep
      
      sleepNwait4AccInterrupt();
      
      digitalWrite(RTCPOWER_PIN, HIGH); // about to generate I2C traffic, so power the rtc from the pin
      pinMode (RTCPOWER_PIN, OUTPUT);

    }  //WHILE LOOP TERMINATOR for drip counter! 

    // Note RTC output signal is much longer than our interrupt handling and thus multiple interrupts are sometimes triggered from one output pulse of the RTC.
    // see one solution at http://heliosoph.mit-links.info/atmega328p-wakeup-sleep-via-interrupt/

    if (clockInterrupt) {      //if you broke out of the while loop, then your RTC interrupt fired.
      digitalWrite(RTCPOWER_PIN, HIGH); // about to generate I2C traffic, so power the rtc from the pin
      pinMode (RTCPOWER_PIN, OUTPUT);
      LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF); //give the RTC a few moments to adjust to power on vcc
      clearClockTrigger(); 
      digitalWrite(INTERRUPT_PIN, HIGH);//set weak internal pull up the interrupt pin
    }

#else  //this is what we do if no drip counter is present

    sleepNwait4RTC();  //sleep and wait for RTC ONLY -  call is inside the main cycle counter loop */

#endif

    digitalWrite(RTCPOWER_PIN, HIGH); // driving this HIGH powers the rtc from the pin
    pinMode (RTCPOWER_PIN, OUTPUT);
    LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF); //give the RTC a few moments to adjust to power on vcc
    
  }   //samples per CYCLE LOOP TERMINATOR (# of sample cycles buffered in eeprom before sd card write happens)
  
    digitalWrite(RTCPOWER_PIN, HIGH); // driving this HIGH powers the rtc from the pin
    pinMode (RTCPOWER_PIN, OUTPUT);
    LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF); //give the RTC a few moments to adjust to power on vcc

}   //the MAIN void LOOP TERMINATOR 


/* To set Duration of Rocket Screams low power sleep mode. Use SLEEP_FOREVER to use other wake up resource:
* (a) SLEEP_15Ms - 15 ms sleep  (THERE IS A TYPO IN THE LIBRARY - SLEEP_MS defined as SLEEP_Ms (lower case s)
* (b) SLEEP_30MS - 30 ms sleep
* c) SLEEP_60MS - 60 ms sleep
* d) SLEEP_120MS - 120 ms sleep
* (e) SLEEP_250MS - 250 ms sleep
* (f) SLEEP_500MS - 500 ms sleep
* (g) SLEEP_1S - 1 s sleep
* (h) SLEEP_2S - 2 s sleep
* (i) SLEEP_4S - 4 s sleep
* (j) SLEEP_8S - 8 s sleep  //did not work for me?  perhaps another typo?
* (k) SLEEP_FOREVER - Sleep without waking up through WDT

* example: LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF); */
//  if you want to include a header file which isn't in the library path, you use double quotes, instead of angle brackets:


/**********************************************
 * SLEEP and wait for Accelerometer  tap interrupt
 ***********************************************/
// I might prefer to use an edge-triggered interrupt in this case, as the low-level interrupt triggers continuously?

#ifdef ADXL345_ISON
//this is the NON rocket scream library version:
/* void sleepNwait4AccInterrupt() 
{
  ADCSRA |= (0<<ADEN);                     // Switch ADC OFF: worth 334 µA during sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts ();          // make sure we don't get interrupted before we sleep
  sleep_enable();
  attachInterrupt(0,clockTrigger, LOW);  //this is the main RTC interrupt - It breaks you out of the tap counter while loop
  attachInterrupt(1,accTrigger, LOW);
  interrupts ();           // interrupts allowed now, next instruction WILL be executed
  sleep_mode();
  //HERE AFTER WAKING UP
  sleep_disable();
  detachInterrupt(1);
  detachInterrupt(0); //if you broke out of the while loop, then your RTC interrupt fired.
  ADCSRA |= (1<<ADEN);                  // enable ADC
  if (accInterrupt && tapcount<429496729) {
    tapcount=tapcount+1; 
    accInterrupt=false;
  } // tapcount is now an unsigned long to accomodate really long intervals
}
*/

void sleepNwait4AccInterrupt()   //this version uses the rocket scream libraries - which lets me shut off the BOD fuse
{
  noInterrupts ();          // make sure we don't get interrupted before we sleep
  attachInterrupt(0,clockTrigger, LOW);  //this is the main RTC interrupt - It breaks you out of the tap counter while loop
  attachInterrupt(1,accTrigger, LOW);
  interrupts ();           // interrupts allowed now, next instruction WILL be executed
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  //HERE AFTER WAKING UP
  detachInterrupt(1);
  detachInterrupt(0); //if you broke out of the while loop, then your RTC interrupt fired.
  //ADCSRA |= (1<<ADEN);                  // enable ADC
  if (accInterrupt && tapcount<429496729) {
    tapcount=tapcount+1; 
    accInterrupt=false;
  } 
}

/**********************************************
 * ACC Tap TRIGGER FLAG
 ***********************************************/
void accTrigger() {//do something quick, flip a flag, and handle in loop();
  accInterrupt = true; 
}
#endif

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

// HEADER INFO
file.print(FileName); 
file.print(F(",Drip sensor:"));
file.print(unitNumber);
file.print(F(",Logger #"));
file.println(loggerNumber);

#ifdef unregulatedMCU
file.print(F(",TinyDuino,"));
#endif

#ifdef vRegulatedMCU
file.print(F(",ProMini,"));
#endif

#ifdef BMA180_ADDRESS
file.print(F("Bma180,"));
#endif
#ifdef HMC5883_ADDRESS
file.print(F("Bma250,"));
#endif

#ifdef TS_TMP102
file.print(F("TMP102,"));
#endif

#ifdef TS_DS18B20
file.print(F("DS18B20,"));
#endif

#ifdef HMC5883_ADDRESS
file.print(F("HMC5883,"));
#endif

#ifdef MS5803_02_ISON
file.print(F("MS5803_02,"));
#endif

#ifdef MS5803_05_ISON
file.print(F("MS5803_05,"));
#endif

#ifdef MS5805_02_ISON
file.print(F("MS5805_02,"));
#endif

#ifdef ADXL345_ISON
file.print(F("ADXL345 (DRIP) "));file.print(int(getRate()/2));file.print(F("Hz,"));
#endif

file.println(F(" "));

#if defined(MS580X_I2C_ADDRESS) //If you have a pressure sensor on, write cal constants to the sd card

  file.println(F("Ms580X Pressure sensor Calibration constants are:"));
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
  file.print(F(" min. "));
#ifdef SampleIntSeconds
  file.print(F(" ")); 
  file.print(SampleIntSeconds);
  file.print(F(" Sec. ")); 
#endif
  file.println(F(" "));

#ifdef accelIsAttached
  file.println(F("YYYY/MM/DD HH:MM, Temp(C), Vcc(mV), AccX = , AccY = , AccZ = , Vcc2 , Compass X, Compass Y, Compass Z,"));
#endif
#ifdef MS580X_I2C_ADDRESS  //if we are pressure sensing, the unit is stationary, and we dont have the acc or comass running
  file.println(F("YYYY/MM/DD HH:MM, Temp(C), Vcc(mV), D1(Raw MS580x Press), D2(Raw MS580x Temp), Vcc2 , 1st order Pres (mbar), 1st ord. Temp(C)"));
#endif
#ifdef ADXL345_ISON
  file.println(F("YYYY/MM/DD HH:MM, RTC Temp(C), Vcc(mV), Drip Count ,Vcc after Newfile Created"));
#endif


  file.println(F(" "));
  file.close();

#ifdef ECHO_TO_SERIAL
  Serial.println(F("New log file created on the SD card!"));
  Serial.flush();//delay(100); //short delay to clear serial com lines
#endif  // ECHO_TO_SERIAL

  // write out the header to the file, only upon creating a new file
  if (file.writeError) {
    // check if error writing
    #ifdef ECHO_TO_SERIAL
    Serial.println(F(" Can't write new file header"));Serial.flush();
    #endif
    error();
  } 

  //    if (!file.sync()) {
  // check if error writing
  //    Serial.println(F("File Syncing Error"));error();
  //  } 


//the New file creation event is the by far the biggest current draw on the unit, so using it here as a "test" of the strength of the batteries
Vcc2 = readExternalVcc();  //check what the post new file creation power supply voltage is

if (Vcc2 < BlueVWarning){
      READSENSOR_PIN = BLUE_PIN;  //change sensor pip to blue as an early indicator that batteries are getting low
      }
if (Vcc2 < RedVWarning){
      READSENSOR_PIN = RED_PIN;  //If the sensor pip is now red, it is time to change the batteries.
      }

if (Vcc2 < CutoffVoltage){   //this is the mininimum voltage needed for the voltage regulators
if (file.isOpen()) {
    file.close();
    #ifdef ECHO_TO_SERIAL
      Serial.println(F("V low after newfile event"));
    #endif
  }
delay(100); //wait till file is really closed.
error();
}
      
}

// ***********************************************/

void syncTheFile(void) 
{
  /* don't sync too often - requires 2048 bytes of I/O to SD card. 512 bytes of I/O if using Fat16 library */

  digitalWrite(RED_PIN, HIGH);
  if (!file.sync()) {
    #ifdef ECHO_TO_SERIAL
    Serial.println(F("File Sync error"));Serial.flush();
    #endif 
    error(); //terminal error if you cant save data to sd card
  }  // 15-20 ms for syncing
  digitalWrite(RED_PIN, LOW); 
  
Vcc2 = readExternalVcc();  //track the voltage during tall this SD card usage
if (Vcc2 < BlueVWarning){
      READSENSOR_PIN = BLUE_PIN;  //change sensor pip to blue as an early indicator that batteries are getting low
      }
if (Vcc2 < RedVWarning){
      READSENSOR_PIN = RED_PIN;  //If the sensor pip is now red, it is time to change the batteries.
      }
if (Vcc2 < CutoffVoltage){   //this is the mininimum voltage needed for the voltage regulators
if (file.isOpen()) {
    file.close();
    #ifdef ECHO_TO_SERIAL
      Serial.println(F("V low with sync event"));
    #endif
  }
delay(100); //wait till file is really closed.
error();
}


}

/*****************************************************************************************************************
 * i2c EEPROM    READ & WRITE PAGEs
 *******************************************************************************************************************/
// see http://www.hobbytronics.co.uk/eeprom-page-write
// Address is a page address - But data can be maximum of 28 bytes
// The Arduino Wire library only has a 32 character buffer, so that is the maximun we can send using Arduino. 
// This buffer includes the two address bytes which limits our data payload to 30 bytes, but I have never gotten this routine to 
// work beyond 28 bytes of payload.
// The time overhead for page writes  = 5ms, whilst the overhead for writing individual bytes = 3.5ms
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
  delay(6);  // data sheet says 5ms for page write
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

void sleepNwait4RTC() 
{
  // OLD  cbi(ADCSRA,ADEN); // Switch ADC OFF: worth 334 µA during sleep
  //ADCSRA |= (0<<ADEN); // Switch ADC OFF: worth 334 µA during sleep
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts ();          //  same as cli() make sure we don't get interrupted before we sleep
  attachInterrupt(0,clockTrigger, LOW);
  //sleep_enable();
  //if (Vcc >= 3500){ // only risk turning off BOD if there is lots of power in the power supply
  //sleep_bod_disable();  //http://forum.arduino.cc/index.php?topic=92840.30;wap2 and http://www.seeedstudio.com/wiki/WDT_Sleep
  //}
  // or turn off brown-out enable in software  http://www.gammon.com.au/forum/?id=11497
  //MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
  //MCUCR = bit (BODS);        // this must be done within 4 clock cycles of above
  // also http://jeelabs.org/2010/09/01/sleep/
  // or http://www.seeedstudio.com/wiki/WDT_Sleep
  
  interrupts ();           // same as sei() interrupts allowed now, next instruction WILL be executed
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  //sleep_mode();
  //HERE AFTER WAKING UP
  //sleep_disable();
  detachInterrupt(0);
  //ADCSRA |= (1<<ADEN);  // Switch ADC converter back ON
  // OLD sbi(ADCSRA,ADEN);  // Switch ADC converter back ON
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
  Wire.requestFrom(0x68,1);       //Read one byte
  bytebuffer1=Wire.read();        //In this example we are not interest in actually using the bye
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231 
  Wire.write(0x0F);               //status register
  Wire.write(0b00000000);         //Write the byte.  The last 0 bit resets Alarm 1 //is it ok to just set these all to zeros?
  Wire.endTransmission();
  clockInterrupt=false;           //Finally clear the flag we use to indicate the trigger occurred
}

// what follows is my old variable WDT sleep code that I was using before I switched to the Rocket Screem Libraries (to turn the BOD off) 
// which works pretty well
/**********************************************
 * SLEEP and wait for watchdog timer
 ***********************************************/
 /*
// you must use ONE of the setWTD_Xms() functions before you call this.. otherwise you sleep forever!

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

 *********************************************
 * WDT INTERRUPT HANDLER
 ***********************************************
ISR (WDT_vect) 
{
  wdt_disable();  // disable watchdog
} 

// *********************************************************
// Set WDT delays  (to save power during sensor conversions)  - define only the ones you need!
// ********************************************************* 
// you can make shorter sleeps if you need to with Clear Timer on Compare Match, or CTC: http://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/
#define WDPS_16MS   (0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(0<<WDP0) 
#define WDPS_32MS   (0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0) 
//#define WDPS_64MS   (0<<WDP3)|(0<<WDP2)|(1<<WDP1)|(0<<WDP0) 
#define WDPS_125MS  (0<<WDP3)|(0<<WDP2)|(1<<WDP1)|(1<<WDP0) 
//#define WDPS_250MS  (0<<WDP3)|(1<<WDP2)|(0<<WDP1)|(0<<WDP0) 
//#define WDPS_500MS  (0<<WDP3)|(1<<WDP2)|(0<<WDP1)|(1<<WDP0) 
//#define WDPS_1S     (0<<WDP3)|(1<<WDP2)|(1<<WDP1)|(0<<WDP0) 
//#define WDPS_2S     (0<<WDP3)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0) 
//#define WDPS_4S     (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(0<<WDP0) 
#define WDPS_8S     (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0)

void setWTD_16ms()   // Tmp102 & ADXL  sensor read uses this one
{ 
  cli();         // disable interrupts
  MCUSR = 0;     // clear various "reset" flags
  WDTCSR = (1<<WDCE)|(1<<WDE); // Set Change Enable bit and Enable Watchdog System Reset Mode. 
  WDTCSR = (1<<WDIE)|(1<<WDIF)|(1<<WDE )| WDPS_16MS;   // Set Watchdog prescalar as per your define above
  wdt_reset();   // pat the dog
  sei();         // Enable global interrupts  
} 

//make a new routine for the time intervals you need to use 

void setWTD_32ms()   // Tmp102 & ADXL sensor read uses this one
{ 
  cli();         // disable interrupts
  MCUSR = 0;     // clear various "reset" flags
  WDTCSR = (1<<WDCE)|(1<<WDE); // Set Change Enable bit and Enable Watchdog System Reset Mode. 
  WDTCSR = (1<<WDIE)|(1<<WDIF)|(1<<WDE )| WDPS_32MS;   // Set Watchdog prescalar as per your define above
  wdt_reset();   // pat the dog
  sei();         // Enable global interrupts  
} 

void setWTD_125ms()   //the acclerometer reads use this one
{  
  cli();         // disable interrupts
  MCUSR = 0;     // clear various "reset" flags
  WDTCSR = (1<<WDCE)|(1<<WDE); // Set Change Enable bit and Enable Watchdog System Reset Mode. 
  WDTCSR = (1<<WDIE)|(1<<WDIF)|(1<<WDE )| WDPS_125MS;   // Set Watchdog prescalar as per your define above
  wdt_reset();   // pat the dog
  sei();         // Enable global interrupts  
} 

void setWTD_8s()   //the acclerometer reads use this one
{  
  cli();         // disable interrupts
  MCUSR = 0;     // clear various "reset" flags
  WDTCSR = (1<<WDCE)|(1<<WDE); // Set Change Enable bit and Enable Watchdog System Reset Mode. 
  WDTCSR = (1<<WDIE)|(1<<WDIF)|(1<<WDE )| WDPS_8S;   // Set Watchdog prescalar as per your define above
  wdt_reset();   // pat the dog
  sei();         // Enable global interrupts  
} 
*/


/*******************************************************
 * READ VCC          using internal 1.1 v  OR analog pin
 ********************************************************/
// from http://forum.arduino.cc/index.php/topic,15629.0.html and http://forum.arduino.cc/index.php?topic=88935.0
  
int readExternalVcc() 
{ 
  int result;  

  //10 bit resolution, returning integers from 0 to 1023
  // we have a simple equal value resistor divider supplying this pin so it can read Vraw above Vcc 
//#ifdef vRegulatedMCU   //the supply voltage via analog read from a 10k ohm resistor divider
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

#ifdef ECHO_TO_SERIAL  //OOPS! you forgot there is no voltage on the raw pin when you are connected to USB!
  result = readInternalVcc();result=result+4000; //if you start seeing 7V and the unit is still running you know you forgot something
#endif

//#endif

  return result;
}

// Need to switch between external analog pin read (above) to vcc trick (below) depending on which mcu board is used

int readInternalVcc() //use this one on systems without a voltage divider - Like the TinyDuino
{ 
  int result;   
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(5); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
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

#ifdef ECHO_TO_SERIAL   //only call halt on error if in debug mode
    Serial.print(F("FAIL in I2C reg write! Result code is "));
    Serial.println(result);
    error();
#endif
  }
  //else
  //{
  //  Serial.println(" ");
  //}
  //delay(10);  //sensors often need some settling time after register writing (BMA180 does)
  //setWTD_16ms();
  //sleepNwait4WDT();
  LowPower.powerDown(SLEEP_15Ms, ADC_OFF, BOD_OFF);
  
  return result;
} 

bool getRegisterBit(int devAddress, byte regAdress, int bitPos) {
  byte bytebuffer1;
  readFrom(devAddress, regAdress, 1, &bytebuffer1);
  return ((bytebuffer1 >> bitPos) & 1);
}

void setRegisterBit(int devAddress, byte regAddress, int bitPos, bool state) {
  byte bytebuffer1;
  readFrom(devAddress, regAddress, 1, &bytebuffer1);
  if (state) {
    bytebuffer1 |= (1 << bitPos);  // forces nth bit of bytebuffer1 to be 1.  all other bits left alone.
  } 
  else {
    bytebuffer1 &= ~(1 << bitPos); // forces nth bit of bytebuffer to be 0.  all other bits left alone.
  }
  writeTo(devAddress, regAddress, bytebuffer1);  
}

// Writes val to address register on device  //SAME AS i2c_writeRegByte - COULD REMOVE THIS
void writeTo(int DEVICE, byte address, byte val) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
} 

// Reads num bytes starting from address register on device in to _buff array  THIS ROUTINE COULD REPLACE I2C_READREGBYTE?
void readFrom(int DEVICE, byte address, int num, byte _buff[]) {
  Wire.beginTransmission(DEVICE); // start transmission to device 
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission

    Wire.beginTransmission(DEVICE); // start transmission to device
  Wire.requestFrom(DEVICE, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  { 
    _buff[i] = Wire.read();    // receive a byte
    i++;
  }
  if(i != num){
    //status = ERROR;
    //error_code = READ_ERROR;
#ifdef ECHO_TO_SERIAL   //only call halt on error if in debug mode
    error();
#endif
  }
  Wire.endTransmission();         // end transmission
} 


/**********************************************
 * BOILERPLATE
 ***********************************************/
void serial_boilerplate() 
{
  Serial.println(F("The Cave Pearl: An Open Source Data Logger for Hydrological Research"));
  Serial.println();
  Serial.println(F("Developed by Edward Mallon:    http://edwardmallon.wordpress.com/"));
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
 * ERROR HANDLER "Houston we have a problem..."
 ***********************************************/
// more advanced debugging: http://forum.arduino.cc/index.php?topic=203282.0
void error() 
{ 
  // if less than a day has passed, X seconds of flashing red light on error, otherwise just shut down immediately  
  //byte Currentday;
  //DateTime now = RTC.now(); Currentday = now.day();
  //if(Currentday == Startday){   
    for (int CNTR = 0; CNTR < 50; CNTR++) { //seconds of flashing red light on error = CNTR/2
    digitalWrite(RED_PIN, HIGH);   
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);                 
    digitalWrite(RED_PIN, LOW); 
    LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_ON);   
    }
  
  //}
  
  pinMode(RTCPOWER_PIN, INPUT);    //stop sourcing or sinking current
  digitalWrite(RTCPOWER_PIN, LOW); // driving this LOW FORCES to the RTC to draw power from the coin cell
  
  #if defined POWERDOWN_PIN
  digitalWrite(POWERDOWN_PIN, HIGH);// driving this pin high shuts down the system if pololu power switch attached
  delay(10);// just in case it takes time to power down...
  #endif
  
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON); //error catch if pololu does not work.
  //sleepNwait4WDT(); //since we have gone to sleep without FIRST setting the wdt, this is sleep forever!

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
  Serial.flush(); 
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
  Serial.print(F("TMP102 initialized: First Raw read="));
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
  //setWTD_32ms();
  //sleepNwait4WDT();  
   LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);
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
  Serial.flush();
#endif

  i2c_writeRegByte(BMA180_ADDRESS,BMA180_CMD_RESET,0xB6);  //B6 (hex) forces the reset - can be done before the ee bit is set?
#ifdef ECHO_TO_SERIAL
Serial.print(F("Soft Reset "));
#endif

  delay(10); //delay serial comms after reset  
  //Control, status & image registers are reset to values stored in the EEprom. 
  //puts the BMA in wake-up mode & default low noise mode "00": BW=1200 Noise 150 ug/rt pg28

  int id = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_CHIP_ID);
  if(id == 0x03)
  {
#ifdef ECHO_TO_SERIAL
    Serial.print(F("BMA180 Chip found at: ")); 
    Serial.print(id);
    Serial.flush();//delay(10); 
#endif
    if (i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG0,1, ctrl_reg0_ee_w_MASK) == 0) //enable register writing
    {
#ifdef ECHO_TO_SERIAL
      Serial.println(F(" BMA180 Write Init Pass,"));
      Serial.flush();//delay(10);
#endif

      // disable wakeup mode because we will be sleeping the sensor manually
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG0,0, ctrl_reg0_dis_wake_up_MASK);  
#ifdef ECHO_TO_SERIAL
      Serial.print(F(" Wake up mode disabled, "));
#endif

      // Connect to the bw_tcs register and set the BW filtering level to 10Hz, Only bits 4-7 of the register hold this data
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_BW_TCS,BMA180_BW_10HZ,cmd_bandwidth_MASK);
#ifdef ECHO_TO_SERIAL
      Serial.print(F(" Filtering level set to 10HZ, "));
      Serial.flush();//delay(10);
#endif

      // Connect to the offset_lsb1 register and set the range
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_RANGEnSMP,BMA180_RANGE_1G,range_MASK);

#ifdef ECHO_TO_SERIAL
      Serial.println(F(" Range set to 1G,"));
      Serial.flush();//delay(10);
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
      Serial.flush();//delay(30);
#endif
      //BMAtemperature = i2c_readReg(BMA180, BMA180_CMD_TEMP);
      //Serial.print("Temperature =  ");Serial.println(BMAtemperature);

      //final step in setup is to disable register writing
      i2c_writeRegBits(BMA180_ADDRESS,BMA180_CMD_CTRL_REG0,0, ctrl_reg0_ee_w_MASK);
      delay(10);

    }
    else
    {  
      Serial.println(F(" BMA180 Write Init Fail! "));
      Serial.flush();//delay(10); 
      error();
    }
  }
  else
  {
    Serial.println(F(" BMA180 Chip Detect Fail! "));
    Serial.flush();//delay(10); 
    error();
  }

  readBMA180();  //just to get the sensor arrays loaded

#ifdef ECHO_TO_SERIAL
  Serial.println(F("...BMA180 has been initialized"));
  Serial.flush();//delay(10);
#endif 
}


void readBMA180() 
{ 
  // read in the 3 axis data, each one is 14 bits = +- 16,383 for integer values
  // note negative values in the directions of the arrows printed on the breakout board!
  
// need to clear the registers then wait OR the first reading in the loop that follows will recieve leftover data from the last series on its first pass!
bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_X_MSB);
bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_X_LSB);
bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Y_MSB);
bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Y_LSB);
bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Z_MSB);
bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Z_LSB);
//setWTD_125ms();
//sleepNwait4WDT();

LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);


  for (int thisReading = 0; thisReading < filterSamples; thisReading++){  //fill the smoothing arrays

    bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_X_MSB);
    rawACCx[thisReading] = bytebuffer1 << 8; // puts the most sig bits on the corect side - I am reading 14 bits total
    bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_X_LSB); 
    rawACCx[thisReading] |= bytebuffer1 >> 2; //this shift gets rid of two non-value bits in LSB register

    bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Y_MSB);
    rawACCy[thisReading] = bytebuffer1 << 8;
    bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Y_LSB);
    rawACCy[thisReading] |= bytebuffer1 >> 2; // what about adding the offset here?

    bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Z_MSB);
    rawACCz[thisReading] = bytebuffer1 << 8;
    bytebuffer1 = i2c_readRegByte(BMA180_ADDRESS, BMA180_CMD_ACC_Z_LSB);
    rawACCz[thisReading] |= bytebuffer1 >> 2;

    // we have the internal BMA bandwith filter set to 10 Hz so its not going to give new data without some time!
    //setWTD_125ms();
    //sleepNwait4WDT();
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);

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
  Serial.println(F("...BMA250 has been initialized"));
#endif 

  return(0);
}

// from BMA250_I2C_Sketch.pde -BMA250 Accelerometer using I2C from www.dsscircuits.com/accelerometer-bma250.html
byte readBMA250()
{

  for (int thisReading = 0; thisReading < filterSamples; thisReading++){  //fill the smoothing arrays

    bytebuffer1 = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_X_MSB);
    rawACCx[thisReading] = bytebuffer1 << 8; // puts the most sig bits on the corect side
    bytebuffer1 = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_X_LSB); 
    rawACCx[thisReading] |= bytebuffer1 >> 6; //this shift gets rid of 6 non-value bits in LSB register

    bytebuffer1 = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_Y_MSB);
    rawACCy[thisReading] = bytebuffer1 << 8;
    bytebuffer1 = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_Y_LSB);
    rawACCy[thisReading] |= bytebuffer1 >> 6; // what about adding the offset here?

    bytebuffer1 = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_Z_MSB);
    rawACCz[thisReading] = bytebuffer1 << 8;
    bytebuffer1 = i2c_readRegByte(BMA250_ADDRESS, BMA250_CMD_ACC_Z_LSB);
    rawACCz[thisReading] |= bytebuffer1 >> 6;

    //delay(135); // we have the internal BMA bandwith filter set to 7.81 Hz so its not going to give new data without some time!
    //setWTD_125ms();  //might need a touch more time...
    //sleepNwait4WDT();
    LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  }
  // Now send those readings out to the digital smoothing function - probably dont need to smooth as much as BMA180
  smoothACCx = digitalSmooth(rawACCx);
  smoothACCy = digitalSmooth(rawACCy);
  smoothACCz = digitalSmooth(rawACCz);
}
#endif


// ************************************************************************************************************
// ************************************************************************************************************
// I2C Accelerometer ADXL345    10 bit (+- 1023)
// ************************************************************************************************************
// from https://code.google.com/p/adxl345driver/source/browse/trunk/ADXL345.cpp?r=11
// datasheet: http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf
// for more registers http://www.i2cdevlib.com/devices/adxl345#registers
// ************************************************************************************************************
// ************************************************************************************************************
// ************************************************************************************************************

#ifdef ADXL345_ISON

/* ------- Register names ------- */
//#define ADXL345_DEVID 0x00
//#define ADXL345_RESERVED1 0x01
#define ADXL345_THRESH_TAP 0x1d
//#define ADXL345_OFSX 0x1e
//#define ADXL345_OFSY 0x1f
//#define ADXL345_OFSZ 0x20
#define ADXL345_DUR 0x21
#define ADXL345_LATENT 0x22
#define ADXL345_WINDOW 0x23
#define ADXL345_THRESH_ACT 0x24
//#define ADXL345_THRESH_INAC 0x25
#define ADXL345_THRESH_INAC 0x25
#define ADXL345_TIME_INACT 0x26
#define ADXL345_ACT_INAC_CTL 0x27
//#define ADXL345_THRESH_FF 0x28
//#define ADXL345_TIME_FF 0x29
#define ADXL345_TAP_AXES 0x2a
#define ADXL345_ACT_TAP_STAT 0x2b
#define ADXL345_BW_RATE 0x2c  //d0-d3= output rate, d4=low power, The default value is 0x0A = 100 Hz output data rate p25
#define ADXL345_POWER_CTL 0x2d
#define ADXL345_INT_SOURCE 0x30
//#define ADXL345_INT_ENABLE 0x2e
#define ADXL345_INT_MAP 0x2f
#define ADXL345_DATA_FORMAT 0x31 //The default 4 interrupt pins is active high -changed to active low by setting the INT_INVERT bit in the  DATA_FORMAT (Address 0x31) register
#define ADXL345_DATAX0 0x32
#define ADXL345_DATAX1 0x33
#define ADXL345_DATAY0 0x34
#define ADXL345_DATAY1 0x35
#define ADXL345_DATAZ0 0x36
#define ADXL345_DATAZ1 0x37
//#define ADXL345_FIFO_CTL 0x38
//#define ADXL345_FIFO_STATUS 0x39

#define ADXL345_BW_1600 0xF // 1111
#define ADXL345_BW_800  0xE // 1110
#define ADXL345_BW_400  0xD // 1101  
#define ADXL345_BW_200  0xC // 1100
#define ADXL345_BW_100  0xB // 1011  
#define ADXL345_BW_50   0xA // 1010 
#define ADXL345_BW_25   0x9 // 1001 
#define ADXL345_BW_12   0x8 // 1000 
#define ADXL345_BW_6    0x7 // 0111
#define ADXL345_BW_3    0x6 // 0110

/* Interrupt PINs */
#define ADXL345_INT1_PIN 0x00 //INT1: 0
#define ADXL345_INT2_PIN 0x01 // INT2: 1

/* Interrupt bit position for ADXL345_INT_ENABLE 0x2e*/
#define ADXL345_INT_DATA_READY_BIT 0x07
//#define ADXL345_INT_SING_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
//#define ADXL345_INT_ACTIVIT_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00

#define ADXL345_OK    1 // no error
#define ADXL345_ERROR 0 // indicates error is presedent

#define ADXL345_NO_ERROR   0 // initial state
#define ADXL345_READ_ERROR 1 // problem reading accel
#define ADXL345_BAD_ARG    2 // bad method argument

#define DEVICE 0x53    // ADXL345 device address
#define TO_READ (6)      // num of bytes we are going to read each time (two bytes for each axis)

// ************************************************************************************************************

void initADXL345()
{ 

#ifdef ECHO_TO_SERIAL
  Serial.println(F("Initializing ADXL345 accelerometer..."));
#endif

  byte _s;
  bool bitback;
  //byte bytebuffer1;

#ifdef ECHO_TO_SERIAL
  Serial.print(F("Setting Range to 2g..."));
  Serial.flush();
#endif  

  //setRangeSetting(1); // Sets the range setting, possible values are: 2, 4, 8, 16
  _s =B00000000; //2g = B00000000; 4g=B00000001, 8g=B00000010;16g=B00000011; default=0
  bytebuffer1=i2c_readRegByte(ADXL345_ADDRESS, ADXL345_DATA_FORMAT);
  _s |= (bytebuffer1 & B11101100); // note: D4 is zero in datasheet
  i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_DATA_FORMAT,_s);
  // setFullResBit? //0=10bit mode, 1= output resolution increases with g range, gain is fixed to maintain a 4 mg/LSB scale factor
  // set the gain for each axis in Gs / count? since we did not set the full rez bit?
  // set the OFSX, OFSY and OFSZ bytes? // ignoring justify bit

  // new setRegisterBit(int devAddress, byte regAddress, int bitPos, bool state)
  // new getRegisterBit(int devAddress, byte regAdress, int bitPos)  just add the device addresss
  // i2c_readRegByte(int dev_address, byte reg_address)  //MUST be interger for the i2c address
  // i2c_writeRegByte(int dev_address, byte reg_address, byte data)


  //set Interrupt Level Bit 0=active high (default) 
  setRegisterBit(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 5, 1); //1=interrupts are now active LOW
#ifdef ECHO_TO_SERIAL
  bitback = getRegisterBit(ADXL345_ADDRESS, ADXL345_DATA_FORMAT, 5); // Gets the state of the INT_INVERT bit
  Serial.print(F("ADXL345 int set to:"));
  Serial.println(bitback);
#endif


  i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_INT_ENABLE, B00000000); // this disables all interupts
  // When initially configuring the interrupt pins, it is recommended that the functions and interrupt mapping be done before enabling the interrupts.

  // I am ignoring freefall setup here

  i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_DUR, 60); 
  // The DUR byte contains an unsigned time value representing the maximum time
  // that an event must be above THRESH_TAP threshold to qualify as a tap event
  // The scale factor is 625µs/LSB / A value of 0 disables the tap/double tap funcitons. Max value is 255.
  // set Tap Duration(60) to (15) works / 625μs per increment  (was 0x1F = 31) and that worked ok 

  //set Tap Threshold betw 1 and 255 / NEVER SET to 0!
  i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_THRESH_TAP , tapSensitivity);
  // Sets the THRESH_TAP byte value /the scale factor is 62.5 mg/LSB  
  // 62.5mg per increment / was able to set down to 60 dur &  2 sensitivity hard mounted inside the 4" caps
  // using delay 175ms in the main loop to slow down and reduce bounce errors - so max of 5 drips per second

  //writeTo(ADXL345_TAP_AXES, B00000111);
  i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_TAP_AXES, B00000111); //all 3 axis set to detect taps
  // or could do them one at a time
  //setRegisterBit(ADXL345_ADDRESS, ADXL345_TAP_AXES, 2, 1); // Tap Detection On X,    1,1=Y   0,1=Z

  // set Double Tap Latency 
  i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_LATENT, 0); // 0-255 max / scale factor is 1.25ms/LSB.
  // writing zero here disables the double tap & puts in single tap mode , typical value 200 for double taps /  1.25ms per increment
  // representing the wait time from the detection of a tap event to the start of the time window, during which a possible second tap can be detected.

  // inactivity config?  I am not using this
  //set Inactivity Threshold // 62.5mg per LSB  // was 8?
  //i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_THRESH_INAC, 8); // 0-255 max

  // Set the TIME_INACT register, amount of time that acceleration must be less thant the value in the THRESH_INACT register for inactivity to be declared. 
  //i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_TIME_INACT, 10); // 0-255 max 1 second increments

  // activity config
  // set Activity Threshold //max of 255  62.5mg/ LSB  setting this too sensitive generates bounce hits!
  i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_THRESH_ACT, tapSensitivity); // 0-255 max // factor is 62.5mg/LSB. Never set to 0!// threshold value for detecting activity.
  // The data format is unsigned, so the magnitude of the activity event is compared with the value is compared with the value in the THRESH_ACT register. The scale

  //the processor must respond to the activity and inactivity interrupts by reading the INT_SOURCE register (Address 0x30) and, therefore, clearing the interrupts.

  // set Activity Ac //turn on ac coupled mode - needed to prevent self triggering loop at high sensitivities.
  setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 7, 1);
  // set which axes can trigger activity  
  setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 6, 1);// setActivityX
  setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 5, 1);// setActivityY
  setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 4, 1);// setActivityZ

  //inactivity config  
  //setRegisterBit(ADXL345_ADDRESS,ADXL345_ACT_INAC_CTL, 3, 1); //set Inactivity Ac//using ac-coupled operation
  //setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 2, 1);// setInActivityX
  //setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 1, 1);// setInActivityY
  //setRegisterBit(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, 0, 1);// setInActivityZ //could only use the z axis to set inactivity state?

  //could set the whole lot with:
  //i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_ACT_INAC_CTL, B11111111); // enables inactivity and activity on x,z,y using ac-coupled operation 
  //The interrupt functions are latched and cleared by either reading the data registers (Address 0x32 to Address 0x37) until the interrupt 
  //condition is no longer valid for the data-related interrupts or by reading the INT_SOURCE register (Address 0x30) for the remaining interrupts

  //Now Mapp each interrupt to the right outgoing pin
  setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_MAP, ADXL345_INT_SING_TAP_BIT,ADXL345_INT1_PIN); // 6
  setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_MAP, ADXL345_INT_ACTIVIT_BIT,ADXL345_INT1_PIN); // 4
  //setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_MAP,ADXL345_INT_INACTIVITY_BIT,ADXL345_INT2_PIN); //not using inactivity yet
  // OR i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_INT_MAP, B00000000); // which sends all interrupts to INT1

  // Now enable the interrupts  //setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
  setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_ENABLE, ADXL345_INT_SING_TAP_BIT, 1);//enable single tap interrupts // 6
  setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_ENABLE, ADXL345_INT_ACTIVIT_BIT, 1);//enable activity interrupts  // 4
  //setRegisterBit(ADXL345_ADDRESS, ADXL345_INT_ENABLE, ADXL345_INT_INACTIVITY_BIT, 1); //enable inactivity interrupts

  //then the power control bits
  // set the ADXL345 in measurment and sleep mode-save power but still detect activity
  // Link bit is set to 1 so inactivity and activity aren't concurrent
  // sets auto_sleep bit to 1 so it goes to sleep when inactivity is detected
  //i2c_writeRegByte(ADXL345_ADDRESS,ADXL345_POWER_CTL, B00111100);  // this sets Link, Auto_sleep, Measure, Sleep on
  i2c_writeRegByte(ADXL345_ADDRESS,ADXL345_POWER_CTL, B00001011);  //my tests used full on measure mode no sleep 

  //bandwidth 
  i2c_writeRegByte(ADXL345_ADDRESS, ADXL345_BW_RATE, ADXL345_BW_25);  // ADXL345_BW_50 or ADXL345_BW_25 or ADXL345_BW_100
  //this is pretty low, might have to raise this later (50 seems to be about the best setting!)
  setRegisterBit(ADXL345_ADDRESS,ADXL345_BW_RATE, 4, 1);// set low power mode (draws 50 uA at 50HZ BW) -this is a high power draw test!
  // power save only works up to BW settings of 12.5, 25,50,100,200 hz  don't use above that!
#ifdef ECHO_TO_SERIAL
  Serial.print("Output rate (= 2xBW) is set to "); 
  Serial.println(int(getRate()/2));  //this is a double, probably will eat ram to print this
#endif 


#ifdef ECHO_TO_SERIAL
  Serial.println(F("...ADXL345 has been initialized for tap sensing"));
  Serial.flush();//delay(10);
#endif  

} //end of init ADXL345 function 

//intSource = getInterruptSource();
//byte _b;readFrom(ADXL345_ACT_TAP_STAT, 1, &_b); //read the ACT_TAP_STATUS register to clear it
//the processor must respond to the activity and inactivity interrupts by reading the INT_SOURCE register (Address 0x30) and, therefore, clearing the interrupts.

/*
byte getInterruptSource() {
 byte bytebuffer1;
 readFrom(ADXL345_ADDRESS, ADXL345_INT_SOURCE, 1, &bytebuffer1);
 return bytebuffer1;
 } 
 
 */

//double getRate(){
int getRate(){
  byte bytebuffer1;
  readFrom(ADXL345_ADDRESS, ADXL345_BW_RATE, 1, &bytebuffer1);  // REPLACE THIS WITH I2C READREGBYTE
  bytebuffer1 &= B00001111;
  //return (pow(2,((int) bytebuffer1)-6)) * 6.25);
  return int((pow(2,((int) bytebuffer1)-6)) * 6.25);
} 

void readAdxl345() { 
  for (int thisReading = 0; thisReading < filterSamples; thisReading++){  //fill the smoothing arrays 
  readFrom(ADXL345_ADDRESS, ADXL345_DATAX0, TO_READ, _buff); //read the acceleration data from the ADXL345
  // each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
  // thus we are converting both bytes in to one int
  rawACCx[thisReading] = (((int)_buff[1]) << 8) | _buff[0];   
  rawACCy[thisReading] = (((int)_buff[3]) << 8) | _buff[2];
  rawACCz[thisReading] = (((int)_buff[5]) << 8) | _buff[4];
  //setWTD_125ms();  //might need a touch more time...
  //sleepNwait4WDT();
  LowPower.powerDown(SLEEP_120MS, ADC_OFF, BOD_OFF);
  }
  // Now send those readings out to the digital smoothing function - probably dont need to smooth as much as BMA180
  smoothACCx = digitalSmooth(rawACCx);
  smoothACCy = digitalSmooth(rawACCy);
  smoothACCz = digitalSmooth(rawACCz);
}


// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
/*
  void setTimeInactivity(int timeInactivity) {
 timeInactivity = min(max(timeInactivity,0),255);
 byte bytebuffer1 = byte (timeInactivity);
 writeTo(ADXL345_ADDRESS,ADXL345_TIME_INACT, bytebuffer1);  
 }
 
 Gets the TIME_INACT register
 int getTimeInactivity() {
 byte bytebuffer1;
 readFrom(ADXL345_ADDRESS,ADXL345_TIME_INACT, 1, &bytebuffer1);  
 return int (bytebuffer1);
 } */


// Sets the THRESH_INACT byte which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared 
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the 
// inactivity interrupt is enabled. The maximum value is 255.
/*
void setInactivityThreshold(int inactivityThreshold) {
 inactivityThreshold = min(max(inactivityThreshold,0),255);
 byte bytebuffer1 = byte (inactivityThreshold);
 writeTo(ADXL345_ADDRESS,ADXL345_THRESH_INAC, bytebuffer1);  
 }
 
 Gets the THRESH_INACT byte
 int getInactivityThreshold() {
 byte bytebuffer1;
 readFrom(ADXL345_ADDRESS,ADXL345_THRESH_INAC, 1, &bytebuffer1);  
 return int (bytebuffer1);
 } */

/* Gets the Latent value
 int getDoubleTapLatency() {
 byte bytebuffer1;
 readFrom(ADXL345_ADDRESS,ADXL345_LATENT, 1, &bytebuffer1);  
 return int (bytebuffer1);
 } */

/*
// Gets the DUR byte
 int getTapDuration() {
 byte bytebuffer1;
 readFrom(ADXL345_ADDRESS, ADXL345_DUR, 1, &bytebuffer1);  
 return int (bytebuffer1);
 } */

// Sets the THRESH_TAP byte value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior

#endif   // terminator for the bit ADXL ifdef statement


// ************************************************************************************************************
// Accelerometer SMOOTHING FUNCTION
// ************************************************************************************************************
// this smoothing function based on Paul Badger's  http://playground.arduino.cc/Main/DigitalSmooth
// "int *inputArray" passes an array to the function - the asterisk indicates the array name is a pointer

#if defined(BMA180_ADDRESS) || defined(BMA250_ADDRESS) || defined(ADXL345_ISON)
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

#endif

// ************************************************************************************************************
// MS580x functions   sensor spec http://www.meas-spec.com/downloads/MS5803-02BA.pdf
// ************************************************************************************************************
// see Application Note AN520 http://www.amsys.info/sheets/amsys.en.an520_e.pdf
// the functions here are based on Luke Millers libraries at  https://github.com/millerlp?tab=repositories
// The only reason I re-juggled his excellent 2Bar and 5Bar sensor libraries into the combined code here (in June 2014), is because I did not have enough free ram left in my 
// script to load even one of original his original libraries. Using his code here means that I must release this version of the Cave Pearl codebase under 
// GPL license, https://github.com/millerlp/MS5803_02/blob/master/LICENSE.md  and I am cool with that.  

#if defined(MS580X_I2C_ADDRESS)

void sendCommandMS5803(byte command){
  Wire.beginTransmission(0x76); //MS5803-02 i2c address, same for all
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
    Serial.println(_ms580Xresolution);    		
  }

  // Read sensor coefficients
  for (int i = 0; i < 8; i++ ){
    // The PROM starts at address 0xA0
    Wire.beginTransmission(MS580X_I2C_ADDRESS);
    Wire.write(0xA0 + (i * 2)); //send the location we want to read
    Wire.endTransmission();
    Wire.requestFrom(MS580X_I2C_ADDRESS, 2);
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
      Serial.flush();//delay(10);
    }
  }

  //#ifdef ADXL345_ISON
  // The first 4 bits of the 0th coefficient form a CRC error checking code.
  // unsigned char p_crc = sensorCoeffs[0]; calcs are different too? need to look at that
  //

#if defined(MS5803_02_ISON) || defined(MS5803_05_ISON)
  // The last 4 bits of the 7th coefficient form a CRC error checking code.
  unsigned char p_crc = sensorCoeffs[7];
  //#endif
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
#endif
  // Otherwise, return true when everything checks out OK. 
  return true;
}

//------------------------------------------------------------------
void readPressureSensor() {
  // Choose from CMD_ADC_256, 512, 1024, 2048, 4096 for mbar resolutions
  // of 1, 0.6, 0.4, 0.3, 0.2 respectively. Higher resolutions take longer
  // to read.
  if (_ms580Xresolution == 256){
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_256); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_256); // read raw temperature	
  } 
  else if (_ms580Xresolution == 512) {
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_512); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_512); // read raw temperature		
  } 
  else if (_ms580Xresolution == 1024) {
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_1024); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_1024); // read raw temperature
  } 
  else if (_ms580Xresolution == 2048) {
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_2048); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_2048); // read raw temperature
  } 
  else if (_ms580Xresolution == 4096) {
    D1 = MS_5803_ADC(CMD_ADC_D1 + CMD_ADC_4096); // read raw pressure
    D2 = MS_5803_ADC(CMD_ADC_D2 + CMD_ADC_4096); // read raw temperature
  }

  // Calculate 1st order temperature, dT is a long integer
  // D2 is originally cast as an uint32_t, but can fit in a int32_t, so we'll
  // cast both parts of the equation below as signed values so that we can
  // get a negative answer if needed

#ifdef firstOrderCalculation
  dT = (int32_t)D2 - ( (int32_t)sensorCoeffs[5] * 256 );

  // Use integer division to calculate TEMP. It is necessary to cast
  // one of the operands as a signed 64-bit integer (int64_t) so there's no 
  // rollover issues in the numerator.

  TEMP = 2000 + ((int64_t)dT * sensorCoeffs[6]) / 8388608LL;   //this line would not compile until I declared TEMP to be int64_t at start
  // Recast TEMP as a signed 32-bit integer
  TEMP = (int32_t)TEMP;
#endif

  /***********************************************************************************************
   * //  NOTE: I do not have enough ram left in this CODEBUILD for ANY second order calculations! (Needs >6K! free)
   * //  Just including them here for completeness since I will need to do these later in excel...
   *************************************************************************************************/

#if defined(MS5803_02_ISON) || defined(MS5805_02_ISON) // EXACTLY SAME CALCS FOR ms5803 & ms5805 2 bar sensors!

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
#ifdef firstOrderCalculation
  // Calculate initial Offset and Sensitivity For 2 bar sensor
  // THESE CALCS TAKE 1K OF PROG MEM TO PERFORM!
  Offset = (int64_t)sensorCoeffs[2] * 131072 + (sensorCoeffs[4] * (int64_t)dT) / 64; 
  Sensitivity = (int64_t)sensorCoeffs[1] * 65536 + (sensorCoeffs[3] * (int64_t)dT) / 128;
#endif

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
#ifdef firstOrderCalculation
  Offset = (int64_t)sensorCoeffs[2] * 262144 + (sensorCoeffs[4] * (int64_t)dT) / 32;
  Sensitivity = (int64_t)sensorCoeffs[1] * 131072 + (sensorCoeffs[3] * (int64_t)dT) / 128;
#endif

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
#ifdef firstOrderCalculation  
  mbarInt = ((D1 * Sensitivity) / 2097152 - Offset) / 32768;
  mbar = (float)mbarInt / 100;    
  tempC  = (float)TEMP / 100; 
#endif

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
  return (n_rem ^ 0x00); // ^ (bitwise xor)
}

//-----------------------------------------------------------------
// Send commands and read the temperature and pressure from the sensor
unsigned long MS_5803_ADC(char commandADC) {
  // D1 and D2 will come back as 24-bit values, and so they must be stored in 
  // a long integer on 8-bit Arduinos.
  long result = 0;
  // Send the command to do the ADC conversion on the chip
  Wire.beginTransmission(MS580X_I2C_ADDRESS);
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
  Wire.beginTransmission(MS580X_I2C_ADDRESS);
  Wire.write((byte)CMD_ADC_READ);
  Wire.endTransmission();
  // Then request the results. This should be a 24-bit result (3 bytes)
  Wire.requestFrom(MS580X_I2C_ADDRESS, 3);
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
  Wire.beginTransmission(MS580X_I2C_ADDRESS);
  Wire.write(CMD_RESET);
  Wire.endTransmission();
  delay(10);
}

#endif


// also see https://github.com/mizraith/RTClib  or https://github.com/akafugu/ds_rtc_lib for more DS3231 specific libs
//could also use RTC.getTemperature() from the library here as in:
// RTC.convertTemperature();             //convert current temperature into registers
// Serial.print(RTC.getTemperature());   //read registers and display the temperature 
// get temp from  http://forum.arduino.cc/index.php/topic,22301.0.html

#ifndef TS_TMP102 && TS_DS18B20 && MS580X_I2C_ADDRESS

float getRTCTemp()
{
  float temp3231;
        //temp registers (11h-12h) get updated automatically every 64s
        Wire.beginTransmission(DS3231_ADDRESS);
	Wire.write(0x11);
	Wire.endTransmission();
	Wire.requestFrom(DS3231_ADDRESS, 2);

if(Wire.available()) {
	bytebuffer1 = Wire.read();	// Here's the MSB
	temp3231 = float(bytebuffer1) + 0.25*(Wire.read()>>6);
        // OR temp3231 = ((((short)bytebuffer1 << 8 | (short)bytebuffer1) >> 6) / 4.0);  //by coding badly
    return temp3231;

  }
  else {
    temp3231 = 255.0;  //Use a value of 255 as error flag
  }
  return temp3231;
}
#endif



/*****************************************************
 * RETIRED FUNCTIONS
 ******************************************************/

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












