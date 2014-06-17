// Date, Time and Alarm functions using a DS3231 RTC connected via I2C and Wire lib by https://github.com/MrAlvin/RTClib 
// based largely on Jean-Claude Wippler from JeeLab's excellent RTC library https://github.com/jcw
// clear alarm interupt from http://forum.arduino.cc/index.php?topic=109062.0
// get temp from  http://forum.arduino.cc/index.php/topic,22301.0.html which does not use the RTCLIB!
// BMA250_I2C_Sketch.pde -BMA250 Accelerometer using I2C from www.dsscircuits.com/accelerometer-bma250.html
// combined with internal voltage reading trick //forum.arduino.cc/index.php/topic,15629.0.html
// floats to string conversion:  http://dereenigne.org/arduino/arduino-float-to-string

// free mem stabilizes around 616 with this three cycle buffer code..
// so each extra set of buffered variables adds about 57 bytes to the ram usage

#include <SD.h>
#include <Wire.h>
#include <SPI.h>   // not used here, but needed to prevent a RTClib compile error
#include <avr/sleep.h>
#include <RTClib.h>

#ifndef cbi  //defs for stopping the ADC during sleep mode
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define DS3231_I2C_ADDRESS 104 //for the RTC temp reading function

#define BMA250              0x18
#define BW                  0x08 //7.81Hz bandwith
#define GSEL                0x03 // set range 0x03 - 2g, 0x05 - 4, 0x08 - 8g, 0x0C - 16g

RTC_DS3231 RTC;
byte Alarmhour = 1;
byte Alarmminute = 1;
byte dummyRegister;
byte INTERRUPT_PIN = 2;
//volatile int state = LOW;
volatile boolean clockInterrupt = false;
byte SampleInterval = 30;    // power-down time in minutes before interupt triggers next sample

byte tMSB, tLSB; //for the RTC temp reading function
float temp3231;

const byte chipSelect = 10;  //sd card chip select

uint8_t dataArray[16]; //variables for accellerometer reading  - why is the temp a float value?
int8_t BMAtemp;  //Ok so this thing is an interger
float BMAtempfloat;
int x,y,z;

String BMAdata; //for passing back data from bma read function
String BMAdata1;  
String BMAdata2 = "0000,0000,0000,0000";  
String BMAdata3 = "0000,0000,0000,0000"; 
char TimeStampbuffer1[ ]= "0000/00/00,00:00:00, ";
char TimeStampbuffer2[ ]= "0000/00/00,00:00:00, "; 
char TimeStampbuffer3[ ]= "0000/00/00,00:00:00, "; 
float Temp3231a =0.0;
float Temp3231b =0.0;
float Temp3231c =0.0;
int Vcc1=0; //the supply voltage via 1.1 internal and gap
int Vcc2=0;
int Vcc3=0;
byte cycle=1;

int ledpin = 13;  //led indicator pin not used in this code

void setup () {

  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, HIGH);//pull up the interrupt pin
  pinMode(13, OUTPUT);     // initialize the LED pin as an output.
  digitalWrite(13, HIGH); // turn the LED on to warn against SD card removal
  
  Serial.begin(9600);
  Wire.begin();
  RTC.begin();
  //RTC.adjust(DateTime(__DATE__, __TIME__));  //set the time with code compile time only run this once!
  clearClockTrigger(); //stops RTC from holding the interrupt low if system reset
  // time for next alarm
  DateTime now = RTC.now();
  Alarmhour = now.hour();
  Alarmminute = now.minute()+ SampleInterval ;
  if (Alarmminute > 59) {  //error catch - if Alarmminute=60 the interrupt never triggers due to rollover
  Alarmminute = 0; Alarmhour = Alarmhour+1; if (Alarmhour > 23) {Alarmhour =0;}
  }
 
  initializeBMA();  //initialize the accelerometer - do I have to do this on every wake cycle?
  
  //get the SD card ready
  pinMode(chipSelect, OUTPUT);  //make sure that the default chip select pin is set to output, even if you don't use it 
  //Serial.print(F("Initializing SD card...")); 
  if (!SD.begin(chipSelect)) {      // see if the card is present and can be initialized:
    Serial.println(F("Card failed, or not present"));   // don't do anything more:
    return;
  }
  //Serial.println(F("card initialized."));
  File dataFile = SD.open("datalog.txt", FILE_WRITE);  //PRINT THE DATA FILE HEADER
  if (dataFile) {      // if the file is available, write to it:
  dataFile.println(F("YYYY/MM/DD HH:MM:SS, Vcc(mV),  X = , Y = , Z = , BMATemp (C) , RTC temp (C)"));
  dataFile.close();
  }  
  else {   //if the file isn't open, pop up an error:
  Serial.println(F("Error opening datalog.txt file!"));
  }
  digitalWrite(13, LOW);
}

void loop () {
  
  if (clockInterrupt) {
  clearClockTrigger(); 
  }
  
  //read in our data
  read3AxisAcceleration();  //loads up the dataString
  DateTime now = RTC.now();  // Read the time and date from the RTC
  
  if (cycle==1){
  BMAdata1 = BMAdata;
  Vcc1 = (readVcc());
  Temp3231a = get3231Temp();
  sprintf(TimeStampbuffer1, "%04d/%02d/%02d %02d:%02d:%02d,", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  
  //Serial.print(TimeStampbuffer1); Serial.print(Vcc1); Serial.print(","); 
  //Serial.print(BMAdata1); Serial.print(","); Serial.println(Temp3231a);
  //Serial.println(freeRam()); 
  //delay(100);
  }
  
  if (cycle==2){
  BMAdata2 = BMAdata;
  Vcc2 = int(readVcc());
  Temp3231b = get3231Temp();
  sprintf(TimeStampbuffer2, "%04d/%02d/%02d %02d:%02d:%02d,", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  //Serial.println("Timestamp Y/M/D, HH:MM:SS, Vcc = ,  X = , Y = , Z = , BMATemp (C) , RTC temp (C)");
  //Serial.print(TimeStampbuffer2); Serial.print(Vcc2); Serial.print(","); 
  //Serial.print(BMAdata2); Serial.print(","); Serial.println(Temp3231b);
  //Serial.println(freeRam()); 
  //delay(100);
  }
  
  if (cycle==3){  //only write to mem card on this cycle
  BMAdata3 = BMAdata;
  Vcc3 = int(readVcc());
  Temp3231c = get3231Temp();
  sprintf(TimeStampbuffer3, "%04d/%02d/%02d %02d:%02d:%02d,", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  //Serial.println("Timestamp Y/M/D, HH:MM:SS, Vcc = ,  X = , Y = , Z = , BMATemp (C) , RTC temp (C)");
  //Serial.print(TimeStampbuffer3); Serial.print(Vcc3); Serial.print(","); 
  ///Serial.print(BMAdata3); Serial.print(","); Serial.println(Temp3231c);
  //Serial.println(freeRam()); 
  //delay(100);
  
  // the whole dataset looks like  0000/00/00 00:00:00,0000,0000,0000,0000,00000,00000
  // if Vcc too low, dont write to the sd card
  if (Vcc1 > 2850){
  //Serial.println(F("-write cycle performed-")); delay (50);//for debugging only
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {      // if the file is available, write to it:
  //here we do the two writes at one time - hopefully saving power only writing every second cycle
  dataFile.print(TimeStampbuffer1);dataFile.print(Vcc1); dataFile.print(",");
  dataFile.print(BMAdata1); dataFile.print(","); dataFile.println(Temp3231a);
  
  dataFile.print(TimeStampbuffer2);dataFile.print(Vcc2); dataFile.print(",");
  dataFile.print(BMAdata2); dataFile.print(","); dataFile.println(Temp3231b);
 
  dataFile.print(TimeStampbuffer3);dataFile.print(Vcc3); dataFile.print(",");
  dataFile.print(BMAdata3); dataFile.print(","); dataFile.println(Temp3231c);  
  
  dataFile.close();
  }  
  else {    //if the file isn't open, pop up an error:
  Serial.println(F("Error opening datalog.txt file"));
  }
  }
  }
  // setNextAlarmTime();
  Alarmhour = now.hour(); Alarmminute = now.minute()+SampleInterval;
  if (Alarmminute > 59) {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
  Alarmminute =0; Alarmhour = Alarmhour+1; if (Alarmhour > 23) {Alarmhour =0;}
  }
  RTC.setAlarm1Simple(Alarmhour, Alarmminute);
  RTC.turnOnAlarm(1);
  
  //Serial.print(F("Alarm Enabled at:  "));
  //Serial.print(now.hour(), DEC); Serial.print(':'); Serial.println(now.minute(), DEC);
  //Serial.print(F("Going to Sleep for ")); Serial.print(SampleInterval);Serial.println(F(" minutes."));
  //delay(100); //a delay long enought to boot out the serial coms 
  
  cycle ++; 
  if (cycle>3){
  cycle=1;}
  
  sleepNow();
  
  //Serial.println(F("Alarm 1 has been Triggered!"));
}


void sleepNow() {
  //digitalWrite(13, LOW);
  cbi(ADCSRA,ADEN); // Switch ADC OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(0,clockTrigger, LOW);
  sleep_mode();
  //HERE AFTER WAKING UP
  sleep_disable();
  detachInterrupt(0);
  sbi(ADCSRA,ADEN);  // Switch ADC converter ON
  //digitalWrite(13, HIGH);
}

void clockTrigger() {
  clockInterrupt = true; //do something quick, flip a flag, and handle in loop();
}

void clearClockTrigger()
{
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231
  Wire.write(0x0F);               //Tell the device which address we want to read or write
  Wire.endTransmission();         //Before you can write to and clear the alarm flag you have to read the flag first!
  Wire.requestFrom(0x68,1);       // Read one byte
  dummyRegister=Wire.read();      // In this example we are not interest in actually using the bye
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231 
  Wire.write(0x0F);               //Tell the device which address we want to read or write
  Wire.write(0b00000000);         //Write the byte.  The last 0 bit resets Alarm 1
  Wire.endTransmission();
  clockInterrupt=false;           //Finally clear the flag we use to indicate the trigger occurred
}

// could also use RTC.getTemperature() from the library here as in:
//RTC.convertTemperature();             //convert current temperature into registers
//Serial.print(RTC.getTemperature());   //read registers and display the temperature

float get3231Temp()
{
 //temp registers (11h-12h) get updated automatically every 64s
 Wire.beginTransmission(DS3231_I2C_ADDRESS);
 Wire.write(0x11);
 Wire.endTransmission();
 Wire.requestFrom(DS3231_I2C_ADDRESS, 2);

 if(Wire.available()) {
   tMSB = Wire.read(); //2's complement int portion
   tLSB = Wire.read(); //fraction portion

   temp3231 = ((((short)tMSB << 8 | (short)tLSB) >> 6) / 4.0); // Allows for readings below freezing - Thanks to Coding Badly
   //temp3231 = (temp3231 * 1.8 + 32.0); // Convert Celcius to Fahrenheit
return temp3231;

 }
 else {
   temp3231 = 255.0;  //Use a value of 255 to error flag that we did not get temp data from the ds3231
 }

 return temp3231;
}


byte read3AxisAcceleration()
{
  Wire.beginTransmission(BMA250);
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(BMA250,7);
  for(int i = 0; i < 7;i++)
  {
    dataArray[i] = Wire.read();
  }
  if(!bitRead(dataArray[0],0)){return(0);}
  
  BMAtemp = dataArray[6];
  x = dataArray[1] << 8;
  x |= dataArray[0];
  x >>= 6;
  y = dataArray[3] << 8;
  y |= dataArray[2];
  y >>= 6;
  z = dataArray[5] << 8;
  z |= dataArray[4];
  z >>= 6;
  //sprintf(BMAdatabuffer, "%04d,%4d,%4d,%4d,", readVcc(), x, y, z);  //this does not read x,y.z properly 
  //dataString = BMAdatabuffer;
  //dataString += String(readVcc());  //added with internal voltage meter trick
  //dataString += " , "; 
  BMAdata = String("");  //clear out the datastring
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
  int temp = (BMAtempfloat - (int)BMAtempfloat) * 100;
  BMAdata += (abs(temp));
  //this also works to convert float to string
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
  Wire.write(0x10); //set bandwith
  Wire.write(BW);
  Wire.endTransmission();
  return(0);
}

long readVcc() {    //trick to read the Vin using internal 1.1 v as a refrence
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(3); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


