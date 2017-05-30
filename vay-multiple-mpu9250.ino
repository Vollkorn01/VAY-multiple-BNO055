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
// define Serial Output
#define SerialPrint  // uncomment this to not print in serial monitor

// define Bool to start logging
bool readData = false;

// SD Card Logger Init
//---------------------------------------------

// define SD Card Logger
 #define Adalogger  // uncomment this to not print on sd card


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


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU9150 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU9150.h"
#include "helper_3dmath.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9150 accelGyroMag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// i2xmux init
#define MPU_addr 0x68
#define TCAADDR 0x70

int sensorNumber = 5;
void tcaselect(uint8_t i) {
  if (i > 7) return; 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission(); 
}
#define LED_PIN 13
bool blinkState = false;

void setup() {
delay(2000);

// SD Card Logger Setup
//----------------------------------------------------------------------------------

  Serial.begin(115200);
  Serial1.begin(9600);
  Serial.println("\r\nAnalog logger test");
  pinMode(13, OUTPUT);

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

  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  Serial.println("Ready!");
#endif
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    for(int x = 0; x < sensorNumber; x++)
    {
          tcaselect(x); // make a loop for all sensors here!
      Wire.beginTransmission(MPU_addr);
      Wire.write(0x6B);  // PWR_MGMT_1 register
      Wire.write(0);     // set to zero (wakes up the MPU-6050)
      Wire.endTransmission(true);
      accelGyroMag.enableMag();
    }
  
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelGyroMag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelGyroMag.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

}

uint8_t i=0; // reset for SD Card logging



// Loop
//----------------------------------------------------------------------------------

void loop() {

startTime = millis();

// labeling with mobile app
 int c;
  if (Serial1.available()) {
    c = Serial1.read();  
    //Serial.print(c);

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
    for (i = 1; i <15; i++)
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
    // read raw accel/gyro/mag measurements from device
    accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);  // or     accelGyroMag.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    //these methods (and a few others) are also available
    //accelGyroMag.getAcceleration(&ax, &ay, &az);
    //accelGyroMag.getRotation(&gx, &gy, &gz);
if (readData)
{ 
#ifdef Adalogger
    // SD card logging
    digitalWrite(8, HIGH);
    logfile.print(ax); logfile.print(",");
    logfile.print(ay); logfile.print(",");
    logfile.print(az); logfile.print(",");
    logfile.print(gx); logfile.print(",");
    logfile.print(gy); logfile.print(",");
    logfile.print(gz); logfile.print(",");
    logfile.print(int(mx)); logfile.print(",");
    logfile.print(int(my)); logfile.print(",");
    logfile.print(int(mz)); logfile.print(",");

    digitalWrite(8, LOW);
  #endif

#ifdef SerialPrint
    // display tab-separated accel/gyro/mag x/y/z values
    Serial.print("a/g/m:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print(int(mx)); Serial.print("\t");
    Serial.print(int(my)); Serial.print("\t");
    Serial.print(int(mz)); Serial.print("\t");
  #endif
}

/*
    const float N = 256;
    float mag = mx*mx/N + my*my/N + mz*mz/N;
    Serial.print(mag); Serial.print("\t");
    logfile.print(mag); logfile.print("\t");
*/
    /*
    for (int i=0; i<mag; i++)
        Serial.print("*"); 
        */
   }
if (readData)
{
#ifdef SerialPrint
    Serial.print(millis()); Serial.print(",");
    Serial.print(userNumber); Serial.print(",");
    Serial.print(exercise);
    Serial.println(";");
  #endif
  
#ifdef Adalogger
    logfile.print(millis()); logfile.print(",");
    logfile.print(userNumber); logfile.print(",");
    logfile.print(exercise);
    logfile.println(";");
    flushcount++;
    if (flushcount >= 90)
    {
    logfile.flush();
    flushcount = 0;
    }
   #endif
}
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

endTime = millis();

if (endTime - startTime < 33)
{
  delay(33 - (endTime - startTime));
}
    
}
