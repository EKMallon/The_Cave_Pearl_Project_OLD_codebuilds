// Date, Time and Alarm functions using a DS3231 RTC connected via I2C and Wire lib by https://github.com/MrAlvin/RTClib 
// based largely on Jean-Claude Wippler from JeeLab's excellent RTC library https://github.com/jcw
// clear alarm interupt from http://forum.arduino.cc/index.php?topic=109062.0
// get temp from  http://forum.arduino.cc/index.php/topic,22301.0.html which does not use the RTCLIB!
// BMA250_I2C_Sketch.pde -BMA250 Accelerometer using I2C from www.dsscircuits.com/accelerometer-bma250.html
// combined with internal voltage reading trick //forum.arduino.cc/index.php/topic,15629.0.html
// floats to string conversion:  http://dereenigne.org/arduino/arduino-float-to-string

// free ram code trick: http://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
// power saving during sleep from http://www.gammon.com.au/forum/?id=11497

// new name routine from https://github.com/adafruit/Light-and-Temp-logger
// about 12 bytes per data cycle!  681 freeram with 10 cycles!

#include <SD.h>  //a memory hog - takes 512 bytes of ram just to run!
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

#define SampleInterval 1   // power-down time in minutes before interupt triggers the next sample
#define SamplesPerCycle 12  //#  of sample cycles before writing to the sd card 
unsigned int countLogs = 0;      // how many records written to each file
unsigned int fileInterval = 96; // #of log records before new logfile is made
/* count each time a log is written into each file.  Must be less than 65,535
 counts per file.  If the sampleinterval is 15min, and fileInterval is 2880
 seconds, then 96samples/day * 30days/month = 30 day intervals */

//#define ECHO_TO_SERIAL   // echo data that we are logging to the serial monitor
// if you don't want to echo the data to serial, comment out the above define
#ifdef ECHO_TO_SERIAL
//#define WAIT_TO_START
/* Wait for serial input in setup(), only if serial is enabled.  You don't want
 to define WAIT_TO_START unless ECHO_TO_SERIAL is defined, because it would
 wait forever to start if you aren't using the serial monitor. 
 If you want echo to serial, but not wait to start,
 just comment out the above define */
#endif

File logfile;
char filename[] = "LOGGER00.CSV";  //the first file name

RTC_DS3231 RTC;
byte Alarmhour = 1;
byte Alarmminute = 1;
byte dummyRegister;
byte INTERRUPT_PIN = 2;
volatile boolean clockInterrupt = false;
byte tMSB, tLSB; //for the RTC temp reading function
float RTCTempfloat;
char CycleTimeStamp[ ]= "0000/00/00,00:00:00";
byte Cycle=0;

const byte chipSelect = 10;  //sd card chip select

uint8_t dataArray[16]; //variables for accellerometer reading
int8_t BMAtemp;        //why does the bma temp read out as an interger? Temp is in units of 0.5 degrees C
//8 bits given in two's complement representation
float BMAtempfloat;//float BMATempHolder;
//char BMATempHolder[ ]= "00.00";
//components for holding bma temp as two intergers - we have no negative temps in our application
uint8_t wholeBMAtemp[SamplesPerCycle],fracBMAtemp[SamplesPerCycle];  

int x,y,z;   //these guys range to negative values
int xAcc[SamplesPerCycle],yAcc[SamplesPerCycle],zAcc[SamplesPerCycle];

uint8_t wRTCtemp[SamplesPerCycle],fRTCtemp[SamplesPerCycle]; //components for holding RTC temp as two intergers
int temp3231;
int Vcc[SamplesPerCycle];//the supply voltage via 1.1 internal band gap

byte ledpin = 13;  //led indicator pin not used in this code

void setup () {

  pinMode(INTERRUPT_PIN, INPUT);
  digitalWrite(INTERRUPT_PIN, HIGH);//pull up the interrupt pin
  pinMode(13, OUTPUT);     // initialize the LED pin as an output.
  digitalWrite(13, HIGH); // turn the LED on to warn against SD card removal does this work?

  Serial.begin(9600);
  Wire.begin();
  RTC.begin();
  clearClockTrigger(); //stops RTC from holding the interrupt low if system reset
  // time for next alarm
  RTC.turnOffAlarm(1);

#ifdef WAIT_TO_START  // only triggered if WAIT_TO_START is defined at beging of code
  Serial.println(F("Type any character to start"));
  while (!Serial.available());
#endif  


  DateTime now = RTC.now();
  DateTime compiled = DateTime(__DATE__, __TIME__);
  if (now.unixtime() < compiled.unixtime()) {
    Serial.println(F("RTC is older than compile time! Updating"));
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }

  Alarmhour = now.hour();
  Alarmminute = now.minute()+ SampleInterval ;
  if (Alarmminute > 59) {  //error catch - if Alarmminute=60 the interrupt never triggers due to rollover
    Alarmminute = 0; 
    Alarmhour = Alarmhour+1; 
    if (Alarmhour > 23) {
      Alarmhour =0;
    }
  }

  initializeBMA();  //initialize the accelerometer - do I have to do this on every wake cycle?

  delay(1000); //delay to prevent power stutters from writing header to the sd card

  //get the SD card ready
  pinMode(chipSelect, OUTPUT);  //make sure that the default chip select pin is set to output, even if you don't use it 

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  // create a new file
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }

  if (! logfile) {
    Serial.println(F("Error creating logger file!"));
    error("1");
  }

  logfile.print(F("The sample interval for this series is: "));
  logfile.print(SampleInterval);
  logfile.println(F(" minutes"));
  logfile.println(F("YYYY/MM/DD HH:MM:SS, Cycle# = Time offset, Vcc(mV),  X = , Y = , Z = , BMATemp (C) , RTC temp (C)"));
  logfile.close();

  // if (logfile.writeError || !logfile.sync()) {
  //     Serial.println(F("Error writing header to logger file!"));error("2");
  // }

#ifdef ECHO_TO_SERIAL  
  Serial.print("Logging to: ");
  Serial.println(filename);
  Serial.print(F("The sample interval for this series is: "));
  Serial.print(SampleInterval);
  Serial.println(F(" minutes"));
  Serial.println(F("Timestamp Y/M/D, HH:MM:SS,Time offset, Vcc = ,  X = , Y = , Z = , BMATemp (C) , RTC temp (C)"));
#endif

  digitalWrite(13, LOW);
}

void loop () {

  // keep track of how many lines have been written to a file
  // after so many lines, start a new file
  if(countLogs >= fileInterval){    

    // create a new file
    for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        // only open a new file if it doesn't exist
        logfile = SD.open(filename, FILE_WRITE); 
        break;  // leave the loop!
      }
    }

    if (! logfile) {
      Serial.println(F("Error creating logger file!"));
      error("1");
    }

    logfile.print(F("The sample interval for this series is: "));
    logfile.print(SampleInterval);
    logfile.println(F(" minutes"));
    logfile.println(F("YYYY/MM/DD HH:MM:SS, Cycle#, = Time offset, Vcc(mV),  X = , Y = , Z = , BMATemp (C) , RTC temp (C)"));
    logfile.close();

    //  if (logfile.writeError || !logfile.sync()) {
    //     Serial.println(F("Error writing header to logger file!"));error("2");
    //  }

#ifdef ECHO_TO_SERIAL  
    Serial.print("Logging to: ");
    Serial.println(filename);
    Serial.print(F("The sample interval for this series is: "));
    Serial.print(SampleInterval);
    Serial.println(F(" minutes"));
    Serial.println(F("Timestamp Y/M/D, HH:MM:SS,Time offset, Vcc = ,  X = , Y = , Z = , BMATemp (C) , RTC temp (C)"));
#endif

    countLogs = 0;     // reset our counter to zero

  }

  for (int Cycle = 0; Cycle < SamplesPerCycle; Cycle++) { //this counts from 0 to (SamplesPerCycle-1)

    if (clockInterrupt) {
      clearClockTrigger(); 
    }

    read3AxisAcceleration();  //loads up the dataString
    DateTime now = RTC.now();  // Read the time and date from the RTC

    if(Cycle==0){ //timestamp for each cycle only gets set once
      sprintf(CycleTimeStamp, "%04d/%02d/%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
    }

    xAcc[Cycle]=x;
    yAcc[Cycle]=y;
    zAcc[Cycle]=z; //BMAtemp_[Cycle] = BMAtempfloat; 
    wholeBMAtemp[Cycle] = (int)BMAtempfloat; 
    fracBMAtemp[Cycle]= (BMAtempfloat - wholeBMAtemp[Cycle]) * 100; // Float split into 2 intergers
    //can use sprintf(BMATempHolder, "%2d.%2d", wholeBMAtemp[Cycle], fracBMAtemp[Cycle]) if we need to recompose that float
    Vcc[Cycle] = (readVcc()); 
    if (Vcc[Cycle] < 2800){
      Serial.println(F("Voltage too LOW"));
      error ("L");
    } //the hangs the system when the voltage is too low.

    RTCTempfloat= get3231Temp();
    wRTCtemp[Cycle] = (int)RTCTempfloat; 
    fRTCtemp[Cycle]= (RTCTempfloat - wRTCtemp[Cycle]) * 100; // Float split into 2 intergers

    //main serial line output loop - which can be commented out for deployment
#ifdef ECHO_TO_SERIAL
    Serial.print(CycleTimeStamp); 
    Serial.print(F(" Cycle ")); 
    Serial.print(Cycle);
    Serial.print(F(",")); 
    Serial.print(Vcc[Cycle]); 
    Serial.print(F(","));
    Serial.print(xAcc[Cycle]); 
    Serial.print(F(","));
    Serial.print(yAcc[Cycle]); 
    Serial.print(F(",")); 
    ;
    Serial.print(zAcc[Cycle]); 
    Serial.print(F(","));
    Serial.print(wholeBMAtemp[Cycle]);
    Serial.print(F("."));
    Serial.print(fracBMAtemp[Cycle]);
    Serial.print(F(","));
    Serial.print(wRTCtemp[Cycle]);
    Serial.print(F("."));
    Serial.print(fRTCtemp[Cycle]);
    Serial.print(F(",  Ram:"));
    Serial.print(freeRam()); 
    delay(50); //short delay to clear com lines
#endif

    // Once each full set of cycles is complete, dump data to the sd card
    // but if Vcc below 2.85 volts, dont write to the sd card
    if (Cycle==(SamplesPerCycle-1) && Vcc[Cycle] >= 2850){
      Serial.print(F(" --write data --")); 
      delay (50);// this line for debugging only

      File logfile = SD.open(filename, FILE_WRITE); 

      if (logfile) {      // if the file is available, write to it:

        for (int i = 0; i < SamplesPerCycle; i++) {  //loop to dump out one line of data per cycle
          logfile.print(CycleTimeStamp); 
          logfile.print(F(",time offset:,"));
          logfile.print(i);
          logfile.print(F(","));
          logfile.print(Vcc[i]); 
          logfile.print(F(","));
          logfile.print(xAcc[i]); 
          logfile.print(F(","));
          logfile.print(yAcc[i]); 
          logfile.print(",");
          logfile.print(zAcc[i]); 
          logfile.print(F(","));
          logfile.print(wholeBMAtemp[i]);
          logfile.print(F("."));
          logfile.print(fracBMAtemp[i]);
          logfile.print(F(","));
          logfile.print(wRTCtemp[i]);
          logfile.print(F("."));
          logfile.print(fRTCtemp[i]);
          logfile.println(F(","));
          // do I need to add a delay line here for sd card communications? could I buffer this better to save power?
          countLogs++;
        }
        logfile.close();
      }  
      else {    //if the file isn't open, pop up an error:
        Serial.println(F("Error opening datalog.txt file"));
      }
    }

    // setNextAlarmTime();
    Alarmhour = now.hour(); 
    Alarmminute = now.minute()+SampleInterval;
    if (Alarmminute > 59) {  //error catch - if alarmminute=60 the interrupt never triggers due to rollover!
      Alarmminute =0; 
      Alarmhour = Alarmhour+1; 
      if (Alarmhour > 23) {
        Alarmhour =0;
      }
    }
    RTC.setAlarm1Simple(Alarmhour, Alarmminute);
    RTC.turnOnAlarm(1);

    //print lines commented out for deployment
    Serial.print(F("  Alarm Set:")); 
    Serial.print(now.hour(), DEC); 
    Serial.print(':'); 
    Serial.print(now.minute(), DEC);
    Serial.print(F(" Sleep:")); 
    Serial.print(SampleInterval);
    Serial.println(F(" min."));
    delay(100); //a delay long enought to boot out the serial coms 

    sleepNow();  //the sleep call is inside the main cycle counter loop
  }

}



void sleepNow() {
  // set the unused digital pins to output low - only worth 1-2 µA during sleep
  // if you have an LED or something like that on an output pin, you will draw more current.
  //  for (byte i = 0; i <= A5; i++)
  //  {
  //  pinMode (i, OUTPUT);
  //  digitalWrite (i, LOW); 
  //  }

  cbi(ADCSRA,ADEN); // Switch ADC OFF: worth 334 µA during sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  attachInterrupt(0,clockTrigger, LOW);
  // turn off brown-out enable in software: worth 25 µA during sleep
  // BODS must be set to one and BODSE must be set to zero within four clock cycles
  MCUCR = bit (BODS) | bit (BODSE);  // turn on brown-out enable select
  MCUCR = bit (BODS);        // The BODS bit is automatically cleared after three clock cycles
  sleep_mode();
  //HERE AFTER WAKING UP
  sleep_disable();
  detachInterrupt(0);
  sbi(ADCSRA,ADEN);  // Switch ADC converter back ON
  //digitalWrite(13, HIGH); this doesnt work because of conflict with sd card chip select
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
// RTC.convertTemperature();             //convert current temperature into registers
// Serial.print(RTC.getTemperature());   //read registers and display the temperature

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
  for(int j = 0; j < 7;j++)
  {
    dataArray[j] = Wire.read();
  }
  if(!bitRead(dataArray[0],0)){
    return(0);
  }

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

  BMAtempfloat = (BMAtemp*0.5)+24.0;
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

void error(char *str) {
  // always write error messages to the serial monitor but this routine wastes
  // everything passed to the string from the original call is in sram!
  Serial.print(F("error in: "));
  Serial.println(str);
  /* this next statement will start an endless loop, basically stopping all
   operation upon any error.  Change this behavior if you want. */
  // red LED indicates error
  //digitalWrite(redLEDpin, HIGH);
  while (1);
}



