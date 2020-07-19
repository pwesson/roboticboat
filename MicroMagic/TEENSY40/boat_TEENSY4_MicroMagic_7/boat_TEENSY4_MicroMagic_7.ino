// Copyright (C) 2020 https://www.roboticboat.uk
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


// Libraries are available here
// https://github.com/pwesson/roboticboat/tree/master/libraries

#include <stdlib.h>
#include <Wire.h>
#include <SD.h>
#include <Servo.h>
#include <math.h>

//  ____                _       _               
// |  _ \   _   _    __| |   __| |   ___   _ __ 
// | |_) | | | | |  / _` |  / _` |  / _ \ | '__|
// |  _ <  | |_| | | (_| | | (_| | |  __/ | |   
// |_| \_\  \__,_|  \__,_|  \__,_|  \___| |_|   
                                                
#include <Rudder.h>


#include <CompassCMPS12.h>


#include <uBlox_NEO_M8N.h>

// __        __  _               _
// \ \      / / (_)  _ __     __| |
//  \ \ /\ / /  | | | '_ \   / _` |
//   \ V  V /   | | | | | | | (_| |
//    \_/\_/    |_| |_| |_|  \__,_|
//  

#include <Wind.h>


int i= 0;

//Is Robot in control of the human
bool IsHuman = true;

//Radio Control Receiver
int ch1 = 0;
int ch2 = 0;
int ch3 = 0;

//LED light
bool islighton = false;

// Timer
unsigned long mytime;
unsigned long lasttime;

//Radio Control Servo
Servo servoRudder;
Servo servoSails;
int rudder = 90;

// Declare the compass class
CompassCMPS12 compass1;

//A GPS library
uBlox_NEO_M8N gps;

String WiFiMessage = "Start...";
char strNumber[15];
char WiFiChecksumMessage[200];

// Global variables
String inputSerial1 = "";         // a string to hold incoming data
boolean IsReadySerial1 = false;  // whether the string is complete

// Setup the weighted least squares
float BoatHeading = 0;
float oldBoatHeading = 0;
float bearingchanged = 0;

signed char pitch = 0;
signed char roll = 0;

Wind mywind;

Rudder myrudder;

float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

float accelx = 0;
float accely = 0;
float accelz = 0;

void setup()
{
  // Keep the User informed
  Serial.begin(9600);

  // Initialize the LED pin as an output.
  //pinMode(13, OUTPUT);

  // Setup the input pins used received by the RC receiver
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT); //IsRobot flag

  // Setup the output pins used by the servos
  pinMode(9, OUTPUT);
  pinMode(15, OUTPUT);

  //Setup servo connection for rudder and sail
  servoRudder.attach(9);
  servoSails.attach(15);

  //Set the rudder range
  myrudder.maxposition = 90 + 40;
  myrudder.minposition = 90 - 40;

  // Initialise the GPS module
  Serial.println("Initializing GPS");
  Serial1.begin(9600);
  delay(100);

  // WiFi module
  Serial2.begin(115200);
  delay(100);

  // Initialize the compass and initial readings
  compass1.begin();

  BoatHeading = compass1.getBearing();
  oldBoatHeading = BoatHeading;

  // Initialise the SD card
  InitialiseSDcard(1);

  //Set the last time
  lasttime = 0;

}

void serialEvent1()
{
  //This event is called when Serial1 receives new bytes
  while (Serial1.available())
  {
    // Read the new byte:
    gps.read((char)Serial1.read());
  }
}

void loop()
{
  // Timer
  mytime = millis();

  // Has 1 seconds passed yet?
  // Approx once an hour this will roll over to zero again.
  if (abs(mytime - lasttime) > 1000)
  {
    // So we enter this section every 1 second
    lasttime = mytime;

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

    // Create WiFi Message
    sprintf(WiFiChecksumMessage,"MSG,%lu,%.0f,%d,%d,%.8f,%.8f,%d,%d,%d,%d,%d",mytime,BoatHeading,pitch,roll,gps.latitude,gps.longitude,gps.numsatelites,ch1,ch2,ch3,0);
    Serial2.print("$");
    Serial2.print(WiFiChecksumMessage);
    Serial2.print("*");
    sprintf(strNumber,"%02X",MakeChecksum(WiFiChecksumMessage));
    Serial2.print(strNumber); 
    // Note not using println as need to be consistent with interactive serial terminal
    Serial2.print("\n");

    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    if (dataFile)
    {
      dataFile.print("MicroMagic7,$TMR,");
      dataFile.print(mytime);
      dataFile.print(",$RC,");
      dataFile.print(ch1);
      dataFile.print(",");
      dataFile.print(ch2);
      dataFile.print(",");
      dataFile.print(ch3);
      dataFile.print(",$CMP,");
      dataFile.print(BoatHeading);
      dataFile.print(",");
      dataFile.print(pitch);
      dataFile.print(",");
      dataFile.print(roll);
      dataFile.print(",$ACC,");
      dataFile.print(accelx, 4);
      dataFile.print(",");
      dataFile.print(accely, 4);
      dataFile.print(",");
      dataFile.print(accelz, 4);
      dataFile.print(",$GYR,");
      dataFile.print(gyrox, 4);
      dataFile.print(",");
      dataFile.print(gyroy, 4);
      dataFile.print(",");
      dataFile.print(gyroz, 4);
      dataFile.print(",$GPS,");
      dataFile.print(gps.gpsdate, 0);
      dataFile.print(",");
      dataFile.print(gps.gpstime, 0);
      dataFile.print(",");
      dataFile.print(gps.latitude, 8);
      dataFile.print(",");
      dataFile.print(gps.latNS);
      dataFile.print(",");
      dataFile.print(gps.longitude, 8);
      dataFile.print(",");
      dataFile.print(gps.lonEW);
      dataFile.print(",");
      dataFile.print(gps.altitude);
      dataFile.print(",");
      dataFile.print(gps.fixquality);
      dataFile.print(",");
      dataFile.print(gps.numsatelites);
      dataFile.print(",");
      dataFile.print(gps.gpsknots);
      dataFile.print(",");
      dataFile.print(gps.gpstrack);
      dataFile.print(",");
      dataFile.print(gps.gpsstatus);
      dataFile.print(",$WND,");
      dataFile.print(mywind.Direction);
      dataFile.print(",$WLS,");
      dataFile.print(myrudder.wls.slope);
      dataFile.print(",");
      dataFile.print(myrudder.wls.intercept);
      dataFile.print(",$LINE,");
      dataFile.print(myrudder.alongtrack);
      dataFile.print(",");
      dataFile.print(myrudder.alongtrackobjective);
      dataFile.print(",");
      dataFile.print(myrudder.crosstrack);
      dataFile.print(",$WAYA,");
      dataFile.print(myrudder.waypointA.latitude, 8);
      dataFile.print(",");
      dataFile.print(myrudder.waypointA.longitude, 8);
      dataFile.print(",$WAYB,");
      dataFile.print(myrudder.waypointB.latitude, 8);
      dataFile.print(",");
      dataFile.print(myrudder.waypointB.longitude, 8);
      dataFile.print(",$WAYD,");
      dataFile.print(myrudder.waypointD.latitude, 8);
      dataFile.print(",");
      dataFile.print(myrudder.waypointD.longitude, 8);
      dataFile.print(",$WAYE,");
      dataFile.print(myrudder.waypointE.latitude, 8);
      dataFile.print(",");
      dataFile.print(myrudder.waypointE.longitude, 8);
      dataFile.print(",$TAR,");
      dataFile.print(myrudder.targetbearing);
      dataFile.print(",$RUD,");
      dataFile.print(rudder);
      dataFile.print(",");
      dataFile.print(myrudder.localdistance);
      dataFile.print(",");
      dataFile.println(myrudder.sensitivity);

      dataFile.close();
    }
    
  }

  // Read the pulse width of each channel.
  // The problem here is that the code blocks until it has read the signal.
  // This can take up to 20ms (per channel) if the radio control transmitter is off (not transmitting)
  ch1 = pulseIn(2, HIGH, 20000);
  ch2 = pulseIn(3, HIGH, 20000);
  ch3 = pulseIn(4, HIGH, 20000);

  // Map the value to the range 0 to 180 which is what radio control servos like
  ch1 = map(ch1, 1000, 2000, 0, 180);
  ch2 = map(ch2, 1000, 2000, 0, 180);
  ch3 = map(ch3, 1000, 2000, 0, 180);

  //If the left RC lever is down - turn on robot
  if (ch3 > 100)
  {
    //The transmitter is indicating to turn on the Robot by the position of channel2
    if (IsHuman)
    {
      //The Human was in control, so now initialise the Robot task

      //The Robot wants to know the line AB to follow.
      myrudder.NewLine(gps.latitude, gps.longitude, BoatHeading);
      
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
  BoatHeading = compass1.getBearing();

  //Note sometimes the compass may return 0 if it can't read.
  //In this case, keep the old bearing
  if (BoatHeading == 0) BoatHeading = oldBoatHeading;

  // Read the pitch and roll of the boat
  pitch = compass1.getPitch();
  roll = compass1.getRoll();

  // Read the compass gyro
  gyrox = compass1.getGyroX() * compass1._gyroScale;
  gyroy = compass1.getGyroY() * compass1._gyroScale;
  gyroz = compass1.getGyroZ() * compass1._gyroScale;

  // Read the compass accelerator
  accelx = compass1.getAcceleroX() * compass1._accelScale;
  accely = compass1.getAcceleroY() * compass1._accelScale;
  accelz = compass1.getAcceleroZ() * compass1._accelScale;

  myrudder.GyroUpdateMean(BoatHeading, gyroz);

  myrudder.AddGyroReading(rudder, gyroz);

  //Want to update the Wind direction estimate
  mywind.Update(BoatHeading, roll);

  // The Human has to control the boat if there is no GPS signal
  if (IsHuman || gps.gpsstatus == 'V')
  {
    // Human sets the rudder position
    rudder = ch1;
    servoRudder.write(rudder);

    // Human sets the sails position
    servoSails.write(ch2);

    //Have sometime for the servo to move
    delay(100);
  }
  else
  {
    // Update the new rudder position
    rudder = myrudder.Update(gps.latitude, gps.longitude, BoatHeading);

    // Robot updates the rudder position
    servoRudder.write(rudder);

    // Human still controls the sails
    servoSails.write(ch2);
  }


  // Print data to Serial Monitor window
  Serial.print("MicroMagic7,$TMR,");
  Serial.print(mytime);
  Serial.print(",$RC,");
  Serial.print(ch1);
  Serial.print(",");
  Serial.print(ch2);
  Serial.print(",");
  Serial.print(ch3);
  Serial.print("\t$CMP,");
  Serial.print(BoatHeading);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print("\t$GYR,");
  Serial.print(gyrox, 4);
  Serial.print(",");
  Serial.print(gyroy, 4);
  Serial.print(",");
  Serial.print(gyroz - myrudder.gyrozmean, 4);
  Serial.print(",");
  Serial.print(myrudder.gyro.slope);
  Serial.print(",");
  Serial.print(myrudder.gyro.intercept);
  Serial.print("\t$GPS,");
  Serial.print(gps.gpsdate, 0);
  Serial.print(",");
  Serial.print(gps.gpstime, 0);
  Serial.print(",");
  Serial.print(gps.latitude, 8);
  Serial.print(",");
  Serial.print(gps.latNS);
  Serial.print(",");
  Serial.print(gps.longitude, 8);
  Serial.print(",");
  Serial.print(gps.lonEW);
  Serial.print(",");
  Serial.print(gps.altitude);
  Serial.print(",");
  Serial.print(gps.fixquality);
  Serial.print(",");
  Serial.print(gps.numsatelites);
  Serial.print(",");
  Serial.print(gps.gpsknots);
  Serial.print(",");
  Serial.print(gps.gpstrack);
  Serial.print(",");
  Serial.print(gps.gpsstatus);
  Serial.print(",$WND,");
  Serial.print(mywind.Direction);
  Serial.print("\t$WLS,");
  Serial.print(myrudder.wls.slope);
  Serial.print(",");
  Serial.print(myrudder.wls.intercept);
  Serial.print("\t$LINE,");
  Serial.print(myrudder.alongtrack);
  Serial.print(",");
  Serial.print(myrudder.crosstrack);
  Serial.print(",$WAYA,");
  Serial.print(myrudder.waypointA.latitude, 8);
  Serial.print(",");
  Serial.print(myrudder.waypointA.longitude, 8);
  Serial.print(",$WAYB,");
  Serial.print(myrudder.waypointB.latitude, 8);
  Serial.print(",");
  Serial.print(myrudder.waypointB.longitude, 8);
  Serial.print(",$TAR,");
  Serial.print(myrudder.targetbearing);
  Serial.print(",$RUD,");
  Serial.print(rudder);
  Serial.print(",");
  Serial.print(myrudder.localdistance);
  Serial.print(",");
  Serial.println(myrudder.sensitivity);
}


void InitialiseSDcard(int testseconds)
{
  Serial.print("Initializing SD card...");

  SD.begin();

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
