// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


//=============================================================================
//                            DEFINING OUTPUTS
//=============================================================================
//

// README:
// uncomment #define BluetoothTransmit, make bool readDate = true, change baudrate of serial1 to 38400
// and change module to hc-05 in order to stream data to smartphone

// define bluetooth output
//#define BluetoothTransmit // uncomment this to not transmit via bluetooth

// define Serial Output
//#define SerialPrint  // uncomment this to not print in serial monitor

// define SD Card Logger
#define Adalogger  // uncomment this to not print on sd card

// starts logging / streaming when receiving start signal from App
  bool readData = true;

// SD Card Logger Init
//---------------------------------------------

#ifdef Adalogger

  #include <SPI.h>
  #include <SD.h>
  
  // Set the pins used
  #define cardSelect 4
  File logfile;
  // blink out an error code
  void error(uint8_t errno) {
    while(1) {
      uint8_t i;
      for (i=0; i<errno; i++) {
        digitalWrite(13, HIGH);
        delay(100);
        digitalWrite(13, LOW);
        delay(100);
      }
      for (i=errno; i<10; i++) {
        delay(200);
      }
    }
  }
#endif
int flushcount = 0;

// Labeling Initialization
int exercise = 99;
#define LED_PIN 13
int userNumber = 100;

// Timing init
int startTime;
int endTime;

//BNO055 Sensor init
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();
int qw, qy, qx, qz;


// i2xmux init
//#define MPU_addr 0x29
#define TCAADDR 0x70

int sensorNumber = 3;
void tcaselect(uint8_t i) {
  if (i > 7) return; 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission(); 
}
#define LED_PIN 13
bool blinkState = false;

//=============================================================================
//                                   SETUP
//=============================================================================

void setup() {


// SD Card Logger Setup
//----------------------------------------------------------------------------------

  Serial.begin(115200);
  Serial1.begin(38400);   
  Wire.begin();

// BNO055 Sensor initialization
//----------------------------------------------------------------------------------
  // i2c mux initialization
 for (int t = 0; t < sensorNumber; t++)
   {
    tcaselect(t);
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
    delay(500);
    bno.setExtCrystalUse(true); // maybe only used to get calibraiton status
  }
}


// SD Card Logger Setup
//----------------------------------------------------------------------------------
                                                           
  Serial.println("\r\nAnalog logger test");

#ifdef Adalogger
  // see if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println("Card init. failed!");
    error(2);
  }
  char filename[15];
  strcpy(filename, "ANALOG00.TXT");
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  if( ! logfile ) {
    Serial.print("Couldnt create "); 
    Serial.println(filename);
    error(3);
  }
  Serial.print("Writing to "); 
  Serial.println(filename);
  pinMode(8, OUTPUT);
  Serial.println("Ready!");
#endif
    
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

}

uint8_t i=0; // reset for SD Card logging


//=============================================================================
//                                   LOOP
//=============================================================================

void loop() {
startTime = millis();

// labeling with mobile app
 int c;
  if (Serial1.available()) {
    c = Serial1.read();  
    Serial.print(c);

// start sending data when receiving a 255

    if (c == 255)
    {
      readData = true;
    }
    
    if (c > 150)
    {
      userNumber = 250 - c;
    }
    
    if (c == 0)
    {
      exercise = 0;
      digitalWrite(LED_PIN, HIGH);
      readData = false;
    }
    
    int i;
    for (i = 1; i <17; i++)
    {
      if (c == i)
      {
        exercise = i;
        digitalWrite(LED_PIN, HIGH);
        readData = true;
        break;
      }
    }
  }
 for (int t = 0; t < sensorNumber; t++)
   {
    tcaselect(t);

if (readData)
{ 
imu::Quaternion quat = bno.getQuat();
qw = quat.w()*1000;
qy = quat.y()*1000;
qx = quat.x()*1000;
qz = quat.z()*1000;

#ifdef BluetoothTransmit
    digitalWrite(13, HIGH);
    Serial1.write(lowByte(qw);
    Serial1.write(highByte(qw);
    Serial1.write(lowByte(qy);
    Serial1.write(highByte(qy);
    Serial1.write(lowByte(qx);
    Serial1.write(highByte(qx);
    Serial1.write(lowByte(qz);
    Serial1.write(highByte(qz);
    digitalWrite(13, LOW);

  #endif

#ifdef Adalogger
    // SD card logging
    digitalWrite(8, HIGH);
    logfile.print(qw);
    logfile.print(",");
    logfile.print(qy);
    logfile.print(",");
    logfile.print(qx);
    logfile.print(",");
    logfile.print(qz);
    logfile.print(",");
    digitalWrite(8, LOW);
  #endif

#ifdef SerialPrint
    // display quaternions
    Serial.print("qW: ");
    Serial.print(qw);
    Serial.print(" qX: ");
    Serial.print(qy);
    Serial.print(" qY: ");
    Serial.print(qx);
    Serial.print(" qZ: ");
    Serial.print(qz);
    Serial.print("\t\t");
  #endif
}
}


if (readData)
{
  
#ifdef SerialPrint
    Serial.println();
    /*
    Serial.print(millis()); Serial.print(",");
    Serial.print(userNumber); Serial.print(",");
    Serial.print(exercise);
    Serial.println(";");
    */
  #endif

#ifdef Adalogger
    logfile.print(millis()); logfile.print(",");
    logfile.print(userNumber); logfile.print(",");
    logfile.print(exercise);
    logfile.println(";");
    flushcount++;
    if (flushcount >= 30)
    {
    logfile.flush();
    flushcount = 0;
    }
  #endif
// blink LED to indicate activity
blinkState = !blinkState;
digitalWrite(LED_PIN, blinkState);
}
/*
endTime = millis();  // THIS DOESNT NECESSARILY MAKES SENSE -> DATAPOINTS ARENT LINEARLY DISTRIBUTED
if (endTime - startTime < 33)
{
  delay(33 - (endTime - startTime));
}
*/
}
