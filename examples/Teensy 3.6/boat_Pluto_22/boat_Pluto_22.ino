// Teensy 3.6 and Duplex 575 Yacht
// Copyright (C) 2020 https://www.roboticboat.uk
// 69b9ac59-2a12-422a-b620-678f577ba2af
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
// https://github.com/pwesson/roboticboat

#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

//   ____   __  __   ____    ____   
//  / ___| |  \/  | |  _ \  / ___|  
// | |     | |\/| | | |_) | \___ \ .
// | |___  | |  | | |  __/   ___) |
//  \____| |_|  |_| |_|     |____/ 

#include <CompassCMPS11.h>

//   ____   ____    ____
//  / ___| |  _ \  / ___|
// | |  _  | |_) | \___ \ .
// | |_| | |  __/   ___) |
//  \____| |_|     |____/

#include <AdaUltimateGPS3.h>

// __        __  _               _
// \ \      / / (_)  _ __     __| |
//  \ \ /\ / /  | | | '_ \   / _` |
//   \ V  V /   | | | | | | | (_| |
//    \_/\_/    |_| |_| |_|  \__,_|
//  

#include <Wind.h>

//  ____                _       _               
// |  _ \   _   _    __| |   __| |   ___   _ __ 
// | |_) | | | | |  / _` |  / _` |  / _ \ | '__|
// |  _ <  | |_| | | (_| | | (_| | |  __/ | |   
// |_| \_\  \__,_|  \__,_|  \__,_|  \___| |_|   
                                                
#include <Rudder.h>

//Is Robot in control of the human
bool IsHuman = true;

int i=0;

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
Servo servoSail;
int rudder = 90;

// Declare the compass class
CompassCMPS11 compass1;

//A GPS library
AdaUltimateGPS3 gps(&Serial5);

// Global variables
String inputSerial5 = "";         // a string to hold incoming data
boolean IsReadySerial5 = false;  // whether the string is complete

// Setup the weighted least squares
float BoatHeading = 0;
float oldBoatHeading = 0;
float desiredHeading = 0;

signed char pitch = 0;
signed char roll = 0;

Wind mywind;

Rudder myrudder;

void setup()
{
  // Keep the User informed
  Serial.begin(9600);

  // initialize the LED pin as an output.
  pinMode(13, OUTPUT);

  // Setup the input pins used received by the RC receiver
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT); //IsRobot flag

  // Setup the output pins used by the servos
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);

  //Setup servo connection for rudder and sail
  servoSail.attach(16);
  servoRudder.attach(17);

  //Set the rudder range
  myrudder.maxposition = 90 + 40;
  myrudder.minposition = 90 - 40;

  // Initialise the GPS module
  Serial.println("Initializing GPS");
  Serial5.begin(9600);
  delay(100);

  // Select only the GPS sentences required
  gps.SelectSentences();

  // Initialize the compass and initial readings
  compass1.begin();

  BoatHeading = compass1.getBearing();
  oldBoatHeading = BoatHeading;

  // Initialise the SD card
  InitialiseSDcard(10);
  
  //Set the last time
  lasttime = 0;

}

void serialEvent5()
{
  // New GPS sentences have arrived
  gps.listen();
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
    //bearingchanged = BoatHeading - oldBoatHeading;

    // Ensure the change in degree is between -180 to 180 degrees.
    //if (bearingchanged > 180) bearingchanged -= 360;
    //if (bearingchanged < -180) bearingchanged += 360;

    // Update the rudder machine learning with a new 1 second observation
    //if (!IsHuman)
    //{
      // Add a reading when the Robot is in control.
      // Only happens when the robot is in control. This means it not being pushed around.
    //  myrudder.AddReading(mytime, ch1, bearingchanged);
    //}

    // Update the old bearing
    //oldBoatHeading = BoatHeading;

    //The GPS signal must be active (not void)
    if (gps.gpsstatus == 'A')
    {
      //Flash the light when receiving a GPS signal
      if (islighton)
      {
        // Turn off the light
        digitalWrite(13, LOW);
        islighton = false;
      }
      else
      {
        // Turn on the light
        digitalWrite(13, HIGH);
        islighton = true;
      }
    }
    else
    {
      // Turn off the light as no valid GPS location
      digitalWrite(13, LOW);
      islighton = false;
    }

    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    if (dataFile)
    {
      dataFile.print("pluto22,$TMR,");
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
      dataFile.print(compass1.accelX, 4);
      dataFile.print(",");
      dataFile.print(compass1.accelY, 4);
      dataFile.print(",");
      dataFile.print(compass1.accelZ, 4);
      dataFile.print(",$GYR,");
      dataFile.print(compass1.gyroX, 4);
      dataFile.print(",");
      dataFile.print(compass1.gyroY, 4);
      dataFile.print(",");
      dataFile.print(compass1.gyroZ, 4);
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
      dataFile.print(",");
      dataFile.print(desiredHeading);
      dataFile.print(",");
      dataFile.print(mywind.mean);
      dataFile.print(",");
      dataFile.print(mywind.stdev);
      dataFile.print(",");
      dataFile.print(mywind.msg);
      dataFile.print(",");
      dataFile.print(mywind.rollPort);
      dataFile.print(",");
      dataFile.print(mywind.rollCentre);
      dataFile.print(",");
      dataFile.print(mywind.rollStarboard);
      dataFile.print(",$WLS,");
      dataFile.print(myrudder.wls.slope);
      dataFile.print(",");
      dataFile.print(myrudder.wls.intercept);
      dataFile.print(",");
      dataFile.print(myrudder.gyro.slope);
      dataFile.print(",");
      dataFile.print(myrudder.gyro.intercept);
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
  ch1 = pulseIn(6, HIGH, 20000);
  ch2 = pulseIn(7, HIGH, 20000);
  ch3 = pulseIn(8, HIGH, 20000);

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
      //myrudder.NewLine(gps.latitude, gps.longitude, BoatHeading);
      rudder = 90;   
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
  pitch = compass1.pitch;
  roll = compass1.roll;
  if (BoatHeading == 0) BoatHeading = oldBoatHeading;

  // Read the compass gyro
  compass1.ReadGyro();

  // Read the compass accelerator
  compass1.ReadAccelerator();
  

// Update the rudder if there is a valid GPS signal
  // Reason is that I wait until the GPS signal arrives before launching the boat
  if (gps.gpsstatus == 'A')
  {
    myrudder.GyroUpdateMean(BoatHeading, compass1.gyroZ);

    myrudder.AddGyroReading(rudder, compass1.gyroZ);    
  }

  //Want to update the Wind direction estimate
  mywind.Update(BoatHeading, roll);
  mywind.UpdateDistribution(roll);
  mywind.UpdateRoll(BoatHeading, roll);

  // The Human has to control the boat if there is no GPS signal
  if (IsHuman)
  {
    // Human sets the rudder position
    rudder = ch1;

    servoRudder.write(rudder);

    // Human sets the sails position
    servoSail.write(ch2);

    //Have sometime for the servo to move
    delay(100);
  }
  else
  {
    // Update the new rudder position
    //rudder = myrudder.Update(gps.latitude, gps.longitude, BoatHeading);

    // Setup a tack - which is estimated wind direction +/- 50 degrees
    desiredHeading = myrudder.ChooseTarget(mywind.Direction, BoatHeading);
    
    // Update the rudder to point towards desiredHeading
    rudder = myrudder.UpdateTarget(desiredHeading, BoatHeading);

    // Robot updates the rudder position
    servoRudder.write(rudder);

    // Human still controls the sails
    servoSail.write(ch2);
  }

  // Print data to Serial Monitor window
  Serial.print("pluto22,$TMR,");
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
  Serial.print(compass1.gyroX, 4);
  Serial.print(",");
  Serial.print(compass1.gyroY, 4);
  Serial.print(",");
  Serial.print(compass1.gyroZ - myrudder.gyrozmean, 4);
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
  Serial.print(",");
  Serial.print(desiredHeading);
  Serial.print(",");
  Serial.print(mywind.rollPort);
  Serial.print(",");
  Serial.print(mywind.rollCentre);
  Serial.print(",");
  Serial.println(mywind.rollStarboard);
 
}

void InitialiseSDcard(int testseconds)
{
  Serial.print("Initializing SD card...");

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD failed");
    for (i=1;i<=testseconds;i++)
    {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(900);
    }
    return;
  }
  Serial.println("done.");

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
