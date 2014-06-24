/* Codebuild for Cave Pearl Project 2013-11-18 -first stable datalogger build! */
/* for a Tinyduino stack consisting of: a processor board, a BMA250 accelerometer sheild,  
the microSD adapter, and a protoboard, jumpered to a chronodot RTC. */

// Date, Time and Alarm functions using a DS3231 chronodot RTC connected via I2C // and Wire library from https://github.com/MrAlvin/RTClib
// based largely on Jean-Claude Wippler from JeeLab’s https://github.com/jcw
// Clear alarm interupt code from http://forum.arduino.cc/index.php?topic=109062.0
// Get temp from http://forum.arduino.cc/index.php/topic,22301.0.html [which does not use the RTCLIB!]
// BMA250_I2C_Sketch.pde – from
// https://tiny-circuits.com/learn/using-the-accelerometer-tinyshield/
// which links to
// http://www.dsscircuits.com/accelerometer-bma250.html
// floats to string conversion: http://dereenigne.org/arduino/arduino-float-to-string
// note: angle brackets which should be around library names are missing here due to formatting weirdness in WordPress

#include <SD.h>
#include <Wire.h>
#include <SPI.h> // not used here, but needed to prevent a RTClib compile error
#include <avr/sleep.h>
#include <RTClib.h>

#ifndef cbi //defs for stopping the ADC during sleep mode
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define DS3231_I2C_ADDRESS 104 //for the RTC temp reading function

#define BMA250 0×18
#define BW 0×08 //7.81Hz bandwith
#define GSEL 0×03 // set range 0×03 – 2g, 0×05 – 4, 0×08 – 8g, 0x0C – 16g
//#define DELAY 10000 is this line an orphan?

RTC_DS3231 RTC;
byte Alarmhour = 1;
byte Alarmminute = 1;
byte dummyRegister;
int INTERRUPT_PIN = 2;
volatile int state = LOW;
volatile boolean clockInterrupt = false;
int SampleInterval = 1;
// power-down time in minutes before interupt triggers next sample

byte tMSB, tLSB; //for the RTC temp reading function
float temp3231;

const int chipSelect = 10; //sd card chip select

uint8_t dataArray[16];
//variables for accellerometer reading
int8_t BMAtemp;
float BMAtempfloat;
int x,y,z;
String BMAdata; //for passing back data from bma read function
char TimeStampbuffer[ ]= "0000/00/00,00:00:00, ";

int ledpin = 13; //led indicator pin

void setup () {

pinMode(INTERRUPT_PIN, INPUT);
digitalWrite(INTERRUPT_PIN, HIGH);//pull up the interrupt pin
pinMode(13, OUTPUT); // initialize the LED pin as an output.
digitalWrite(13, HIGH); // turn the LED on to warn against SD card removal

Serial.begin(9600);
Wire.begin();
RTC.begin();
//RTC.adjust(DateTime(__DATE__, __TIME__));
//the above line set the time with code compile time – you only run this line ONCE!
clearClockTrigger(); //stops RTC from holding the interrupt low if system reset
// time for next alarm
DateTime now = RTC.now();
Alarmhour = now.hour();
Alarmminute = now.minute()+ SampleInterval ;
if (Alarmminute > 59) { //error catch – if Alarmminute=60 the interrupt never triggers due to rollover
Alarmminute = 0; Alarmhour = Alarmhour+1; if (Alarmhour > 23) {Alarmhour =0;}
}

initializeBMA(); //initialize the accelerometer

//get the SD card ready
pinMode(chipSelect, OUTPUT); //make sure that the default chip select pin is set to output, even if you don’t use it
Serial.print("Initializing SD card…");
if (!SD.begin(chipSelect)) { // see if the card is present and can be initialized:
Serial.println("Card failed, or not present"); // don’t do anything more:
return;
}
Serial.println("card initialized.");
File dataFile = SD.open("datalog.txt", FILE_WRITE); //PRINT THE DATA FILE HEADER
if (dataFile) { // if the file is available, write to it:
dataFile.println("YYYY/MM/DD, HH:MM:SS, Vcc(mV), X = , Y = , Z = , BMATemp (C) , RTC temp (C)");
dataFile.close();
}
else { //if the file isn’t open, pop up an error:
Serial.println("Error opening datalog.txt file!");
}
}

void loop () {

if (clockInterrupt) {
clearClockTrigger();
}

//read in our data
BMAdata = String(""); //clear out the datastring
read3AxisAcceleration(); //loads up the dataString
DateTime now = RTC.now(); // Read the time and date from the RTC
sprintf(TimeStampbuffer, "%04d/%02d/%02d,%02d:%02d:%02d,", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
Serial.println("Timestamp Y/M/D, HH:MM:SS, Vcc = , X = , Y = , Z = , BMATemp (C) , RTC temp (C)");
Serial.print(TimeStampbuffer); Serial.print(readVcc()); Serial.print(",");
Serial.print(BMAdata); Serial.print(","); Serial.println(get3231Temp());

//write data to the SD card
// note that only one file can be open at a time,so you have to close this one before opening another.
File dataFile = SD.open("datalog.txt", FILE_WRITE);

if (dataFile) { // if the file is available, write to it:
dataFile.print(TimeStampbuffer);dataFile.print(readVcc()); dataFile.print(",");
dataFile.print(BMAdata); dataFile.print(","); dataFile.println(get3231Temp());
dataFile.close();
}
else { //if the file isn’t open, pop up an error:
Serial.println("Error opening datalog.txt file");
}

// setNextAlarmTime();
Alarmhour = now.hour(); Alarmminute = now.minute()+SampleInterval;
if (Alarmminute > 59) { //error catch – if alarmminute=60 the interrupt never triggers due to rollover!
Alarmminute =0; Alarmhour = Alarmhour+1; if (Alarmhour > 23) {Alarmhour =0;}
}
RTC.setAlarm1Simple(Alarmhour, Alarmminute);
RTC.turnOnAlarm(1);

Serial.print("Alarm Enabled at: ");
Serial.print(now.hour(), DEC); Serial.print(":’); Serial.println(now.minute(), DEC);
Serial.print("Going to Sleep for "); Serial.print(SampleInterval);Serial.println(" minutes.");
delay(100);
//a delay long enough to boot out the serial coms before sleeping.

sleepNow();

//Serial.println("Alarm 1 has been Triggered!");
}

void sleepNow() {
digitalWrite(13, LOW);
cbi(ADCSRA,ADEN); // Switch ADC OFF
set_sleep_mode(SLEEP_MODE_PWR_DOWN);
sleep_enable();
attachInterrupt(0,clockTrigger, LOW);
sleep_mode();
//HERE AFTER WAKING UP
sleep_disable();
detachInterrupt(0);
sbi(ADCSRA,ADEN); // Switch ADC converter ON
pinMode(13, OUTPUT); digitalWrite(13, HIGH);
// turn the LED on to warn against SD card removal
// But I have some conflict between the SD chip select line, and the led – so the darned thing never lit!
}

void clockTrigger() {
clockInterrupt = true; //do something quick, flip a flag, and handle in loop();
}

void clearClockTrigger()
{
Wire.beginTransmission(0×68); //Tell devices on the bus we are talking to the DS3231
Wire.write(0x0F); //Tell the device which address we want to read or write
Wire.endTransmission(); //Before you can write to and clear the alarm flag you have to read the flag first!
Wire.requestFrom(0×68,1); // Read one byte
dummyRegister=Wire.read(); // In this example we are not interest in actually using the bye
Wire.beginTransmission(0×68); //Tell devices on the bus we are talking to the DS3231
Wire.write(0x0F); //Tell the device which address we want to read or write
Wire.write(0b00000000); //Write the byte. The last 0 bit resets Alarm 1
Wire.endTransmission();
clockInterrupt=false; //Finally clear the flag we used to indicate the trigger occurred
}

// could also use RTC.getTemperature() from the library here as in:
// RTC.convertTemperature(); //convert current temperature into registers
// Serial.print(RTC.getTemperature()); //read registers and display the temperature

float get3231Temp()
{
//temp registers (11h-12h) get updated automatically every 64s
Wire.beginTransmission(DS3231_I2C_ADDRESS);
Wire.write(0×11);
Wire.endTransmission();
Wire.requestFrom(DS3231_I2C_ADDRESS, 2);

if(Wire.available()) {
tMSB = Wire.read(); //2′s complement int portion
tLSB = Wire.read(); //fraction portion

temp3231 = ((((short)tMSB << 8 | (short)tLSB) >> 6) / 4.0); // Allows for readings below freezing – Thanks to Coding Badly
//temp3231 = (temp3231 * 1.8 + 32.0); // Convert Celcius to Fahrenheit
return temp3231;

}
else {
temp3231 = 255.0; //Use a value of 255 to error flag that we did not get temp data from the ds3231
}

return temp3231;
}

byte read3AxisAcceleration()
{
Wire.beginTransmission(BMA250);
Wire.write(0×02);
Wire.endTransmission();
Wire.requestFrom(BMA250,7);
for(int i = 0; i < 7;i++)
{
dataArray[i] = Wire.read();
}
if(!bitRead(dataArray[0],0)){return(0);}

BMAtemp = dataArray[6];
x = dataArray[1] << 8; x |= dataArray[0]; x >>= 6;
y = dataArray[3] << 8; y |= dataArray[2]; y >>= 6;
z = dataArray[5] << 8; z |= dataArray[4]; z >>= 6;

BMAdata += String(x);
BMAdata += ",";
BMAdata += String(y);
BMAdata += ",";
BMAdata += String(z);
BMAdata += ",";
BMAtempfloat = (BMAtemp*0.5)+24.0;
// add digits of BMAtempfloat value to datastring
BMAdata += ((int)BMAtempfloat);
BMAdata += ".";
int temp = (BMAtempfloat – (int)BMAtempfloat) * 100;
BMAdata += (abs(temp));

//the following 2 lines could also be used to to convert floats to string
//dtostrf(floatVariable2convert, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuffer);
//for example: dtostrf(BMAtempfloat, 5, 2, dtostrfbuffer); dataString += dtostrfbuffer;
}

byte initializeBMA()
{
Wire.beginTransmission(BMA250);
Wire.write(0x0F); //set g
Wire.write(GSEL);
Wire.endTransmission();
Wire.beginTransmission(BMA250);
Wire.write(0×10); //set bandwith
Wire.write(BW);
Wire.endTransmission();
return(0);
}

// internal voltage reading trick from: http://forum.arduino.cc/index.php/topic,15629.0.html
long readVcc() { //trick to read the Vin using internal 1.1 v as a refrence
long result;
// Read 1.1V reference against AVcc
ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
delay(2); // Wait for Vref to settle
ADCSRA |= _BV(ADSC); // Convert
while (bit_is_set(ADCSRA,ADSC));
result = ADCL;
result |= ADCH<<8;
result = 1126400L / result; // Back-calculate AVcc in mV
return result;
}
