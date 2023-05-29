// Teensy 3.6 and MicroMagic Yacht
// Copyright (C) 2023 https://www.roboticboat.uk
// 2da74b77-597f-4f01-82cc-bc7304324e73
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


// Arduino 1.8.19 IDE   https://www.arduino.cc/en/software
// Teensyduino 1.56     https://www.pjrc.com/teensy/td_download.html
// My libraries         https://github.com/pwesson/roboticboat
//                      https://github.com/pwesson/compass
//                      https://github.com/pwesson/gps


#include <stdlib.h>
#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

//===============================================

#include <CompassCMPS14.h>

#include <ParallaxSIM33EAU.h>

#include <Wind.h>

#include <Rudder.h>

#include <WeightedLeastSquares.h>

//===============================================

int i= 0;

//Is Robot in control of the human
bool IsHuman = true;

//Radio Control Receiver
int ch1 = 90;
int ch2 = 90;
int ch3 = 90;

int outputV1 = 0;
int outputV2 = 0;

//LED light
bool islighton = false;

// Timer
unsigned long mytime;
unsigned long lasttime;
unsigned long lasttimeservo;

//Radio Control Servo
int rudder = 90;

// Declare the compass class
CompassCMPS14 compass1;

//A GPS library
ParallaxSIM33EAU gps;

//Radio Control Servo
Servo servoRudder;
Servo servoSail;

// Global variables
String inputSerial1 = "";         // a string to hold incoming data
boolean IsReadySerial1 = false;  // whether the string is complete

// Setup the weighted least squares
float BoatHeading = 0;
float oldBoatHeading = 0;
float bearingchanged = 0;
int bearing;

// Boom location using Hall sensors
int cutoffX = 0;
int cutoffY = 0;
int x = 0;
int y = 0;
float leftmeanx = 0;
float leftmeany = 0;
float rightmeanx = 0;
float rightmeany = 0;
float discount = 0.99;

// Serial
unsigned long mytimeSerial;
unsigned long lasttimeSerial;
byte c = 0;
byte next = 0;
byte buffer[200];
byte ibuffer = 0;

Wind mywind;

Rudder myrudder;

// Buffer must be as long as the output string to disk
char sdbuffer[500];
File dataFile;

// WiFi
String WiFiMessage = "Start...";
char strNumber[15];
char WiFiChecksumMessage[200];

void setup()
{
  // Keep the User informed
  Serial.begin(115200);
  delay(100);
  
  // WiFi module
  Serial5.begin(115200);
  delay(100);

  // LED light
  pinMode(13, OUTPUT);
  
  // Initialise Hall pins
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  
  // Setup the input pins used received by the RC receiver
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);

  // Pins to control the Servos
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  // Attach the servos
  servoRudder.attach(7);
  servoSail.attach(8);

  // Start Serial1 for GPS
  Serial1.begin(9600);

  // Initialize the compass and initial readings
  Wire.begin();

  // Wait for the i2c network to wakeup
  delay(100);

  gps.setSerial(&Serial1);

  // Restrict GPS messages to those required
  gps.SelectSentences();
  
  BoatHeading = bearing;
  oldBoatHeading = BoatHeading;
  
  // Initialise the SD card
  InitialiseSDcard(5);

  //Set the last time
  lasttime = 0;

  // Read the calibration quality
  compass1.getCalibration();
  
  // Check the compass calibration
  Serial.printf(BYTE_TO_BINARY_PATTERN, "\n", BYTE_TO_BINARY(compass1.calibration));

  // Open the data logging file (once)
  dataFile = SD.open("datalog.txt", FILE_WRITE);
}

void serialEvent1()
{
  gps.listen();
}

void loop()
{
  // Timer
  mytime = millis();

  // Read the pulse width of each channel.
  // The problem here is that the code blocks until it has read the signal.
  // This can take up to 20ms (per channel) if the radio control transmitter is off (not transmitting)
  ch1 = pulseIn(3, HIGH, 20000);
  ch2 = pulseIn(4, HIGH, 20000);
  ch3 = pulseIn(5, HIGH, 20000);

  // Map the value to the range 45 to 135 which is what radio control servos like
  ch1 = map(ch1, 1000, 2000, 45, 135);
  ch2 = map(ch2, 1000, 2000, 45, 135);
  ch3 = map(ch3, 1000, 2000, 45, 135);

  // Restrict the servo movement
  if (ch1 < 45) {ch1 = 45;}
  if (ch2 < 45) {ch2 = 45;}
  if (ch3 < 45) {ch3 = 45;}

  // Restrict the servo movement
  if (ch1 > 135) {ch1 = 135;}
  if (ch2 > 135) {ch2 = 135;}
  if (ch3 > 135) {ch3 = 135;}

  // Read the Hall Sensors
  measureHall();

  // Hall (x,y) observation
  x = outputV1 - outputV2;
  y = outputV1 + outputV2;

  // Has 500 milliseconds passed yet?
  if (abs(mytime - lasttime) > 500)
  {  
    // So we enter this section every 1 second
    lasttime = mytime;

    if (islighton)
    {
      // Turn off the light
      islighton = false;
      digitalWrite(13, LOW);
    }
    else 
    {
      // Turn on the light
      islighton = true;
      digitalWrite(13, HIGH);
    }
    
    // The change in the bearing over 1 second.
    // Can be positive or negative and greater than 180 or even 360 degrees
    // The issue is going from 10 to 350 degrees is -20 degrees rather than +340 degrees
    bearingchanged = BoatHeading - oldBoatHeading;

    // Ensure the change in degree is between -180 to 180 degrees.
    if (bearingchanged > 180) bearingchanged -= 360;
    if (bearingchanged < -180) bearingchanged += 360;

    // Update the rudder machine learning with a new 1 second observation
    if (!IsHuman)
    {
      // Add a reading when the Robot is in control.
      // Only happens when the robot is in control. This means it not being pushed around.
      myrudder.AddReading(mytime, ch1, bearingchanged);
    }
    
    // Update the old bearing
    oldBoatHeading = BoatHeading;

    sprintf(sdbuffer, "MicroMagic31,$TMR,%lu,$RC,%d,%d,%d,$CMP,%.0f,%d,%d,%c%c%c%c%c%c%c%c,$ACC,%f,%f,%f,$GYR,%f,%f,%f,$GPS,%.0f,%.0f,%.8f,%c,%.8f,%c,%.1f,%d,%d,%.2f,%.2f,%c,$WND,%d,$HALL,%d,%d", 
                      mytime,ch1,ch2,ch3,
                      BoatHeading,compass1.pitch,compass1.roll,BYTE_TO_BINARY(compass1.calibration),
                      compass1.accelX,compass1.accelY,compass1.accelZ,
                      compass1.gyroX,compass1.gyroY,compass1.gyroZ - myrudder.gyrozmean,
                      gps.gpsdate,gps.gpstime,gps.latitude,gps.latNS,gps.longitude,gps.lonEW,gps.altitude,gps.fixquality,gps.numsatelites,gps.gpsknots,gps.gpstrack,gps.gpsstatus,
                      mywind.Direction,
                      outputV1,outputV2);

    // If the USB Serial port is working
    if (Serial){
      
      // Update the User
      Serial.printf("%s\n",sdbuffer);

      // Ensure the data is not buffered
      Serial.flush();
    }
  
    if (dataFile)
    {
      // Save to SD card
      dataFile.printf("%s\n",sdbuffer);

      // Ensure the data is not buffered
      dataFile.flush();
    }    
  }

    // Create WiFi Message
    sprintf(WiFiChecksumMessage,"MSG,%lu,%.0f,%d,%d,%.8f,%.8f,%d,%d,%d,%d,%d",mytime,BoatHeading,compass1.pitch,compass1.roll,gps.latitude,gps.longitude,gps.numsatelites,ch1,ch2,outputV1,outputV2);
    Serial5.print("$");
    Serial5.print(WiFiChecksumMessage);
    Serial5.print("*");
    sprintf(strNumber,"%02X",MakeChecksum(WiFiChecksumMessage));
    Serial5.print(strNumber); 
    // Note not using println as need to be consistent with interactive serial terminal
    Serial5.print("\n");
    Serial5.flush();

  //If the left RC lever is down - turn on robot
  if (ch3 > 100)
  {
    //The transmitter is indicating to turn on the Robot by the position of channel2
    if (IsHuman)
    {
      //The Human was in control, so now initialise the Robot task

      //The Robot wants to know the line AB to follow.
      //myrudder.NewLine(gps.latitude, gps.longitude, BoatHeading);     
    }
    
    //The Human has handed control of the boat to the Robot
    IsHuman = false;
  }
  else
  {
    //The Human has regained control of the boat
    IsHuman = true;

    myrudder.ClearLine();
  }
  
  // Read the compass.
  compass1.ReadCompass();
  BoatHeading = compass1.bearing;
  if (BoatHeading == 0) BoatHeading = oldBoatHeading;

  // Read the compass gyro
  compass1.ReadGyro();

  // Read the compass accelerator
  compass1.ReadAccelerator();

  // Read the calibration quality
  compass1.getCalibration();

  myrudder.GyroUpdateMean(BoatHeading, compass1.gyroZ);

  myrudder.AddGyroReading(rudder, compass1.gyroZ);

  //Want to update the Wind direction estimate
  mywind.Update(BoatHeading, compass1.roll);

  // The Human has to control the boat if there is no GPS signal
  if (IsHuman)
  {
    // Human sets the rudder position
    rudder = ch1;
    servoRudder.write(ch1);

    // Human sets the sails position
    servoSail.write(ch2);
  }
  else
  { 
    // Human sets the rudder position (still testing)
    rudder = ch1;
    servoRudder.write(ch1);

    // Testing sails at a fixed position
    servoSail.write(113);
  }

}

void InitialiseSDcard(int testseconds)
{
  Serial.println("Initializing SD card...");

  SD.begin(BUILTIN_SDCARD);

  delay(100);

  if (ReadWriteTest()){
    Serial.println("OK");
    digitalWrite(13, HIGH);
    delay(testseconds * 1000);
    digitalWrite(13, LOW);
  }
  else
  {
    Serial.println("ERROR");
    for (i=1;i<=testseconds;i++)
    {
      digitalWrite(13, HIGH);
      delay(500);
      digitalWrite(13, LOW);
      delay(500);
    }
  }  
}

bool ReadWriteTest()
{
  File myFile;
  char filename[] = "testfile.txt";
  char writestring[] = "abcdefghijklmnopqrstuvwxyz1234567890";
  char readstring[40];

  // First remove the file is it already exists
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  // Open file to write
  myFile = SD.open(filename, FILE_WRITE);
  
  // If okay, write to the file
  if (myFile) {
    Serial.print("Writing to file...  ");
    myFile.print(writestring);
    myFile.close();
    Serial.print('[');
    Serial.print(writestring);
    Serial.println("] done.");
  } 
  else 
  {
    // Error writing to the file
    Serial.println("error opening testfile.txt");
  }
  
  // Open file to read. Which is the default option
  myFile = SD.open(filename, FILE_READ);
  if (myFile) {
    Serial.print("Reading from file...");
    int n = 0;
    while (myFile.available()) {
      if (n<39)
      {
        readstring[n] = myFile.read();
        readstring[n+1] = '\0';
      }
      n=n+1;
    }
    myFile.close();
    Serial.print('[');
    Serial.print(readstring);
    Serial.println("] done.");
  } 
  else 
  {
    // Error reading from the file
    Serial.println("error opening testfile.txt");
  }

  // Return true if the two char arrays are equal
  if (strcmp(writestring, readstring) == 0){
    return true;
  }
  return false;
}


int MakeChecksum(char* msg){

  // Length of the GPS message
  int len = strlen(msg);

  // Initialise the checksum
  int cksum = 0;

  // Loop over message characters
  for (int i=0; i < len; i++) {
    cksum ^= msg[i];
  }

  return cksum % 0xffff;
}

bool CheckSum(char* msg) {

  // Check the checksum
  // The message ends with *XX\r\n

  // Length of the message
  // Ignore the '\r' and '\n' characters
  int len = strlen(msg)-2;

  // Does the string contain a checksum?
  if (msg[len-3] != '*') return false;

  // Read the checksum from the message
  int cksum = 16 * Hex2Dec(msg[len-2]) + Hex2Dec(msg[len-1]);

  // Loop over message characters
  for (int i=0; i < len-3; i++) cksum ^= msg[i];
  
  // Do we have a wrong checksum?
  if (cksum != 0) return false;

  // Checksum is correct
  return true;
}

// Convert HEX to DEC
int Hex2Dec(char c) {

  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  else if (c >= 'A' && c <= 'F') {
    return (c - 'A') + 10;
  }
  else {
    return 0;
  }
}

void measureHall()
{
  // Take 10 readings
  outputV1 = 0;
  outputV2 = 0;
  
  for(int i = 0; i < 10; i++)
  {
      outputV1 += analogRead(A7);
      outputV2 += analogRead(A8);
      delay(1);
  }

}
