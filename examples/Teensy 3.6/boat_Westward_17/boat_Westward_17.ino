// Teensy 3.6 and Westward Yacht
// Copyright (C) 2019 https://www.roboticboat.uk
// 5ac0afe7-ec98-4217-bb27-d3d29584c463
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

// Arduino 1.8.12

// Libraries are available here
// https://github.com/pwesson/roboticboat

#include <SD.h>
#include <Wire.h>
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

#include <ParallaxSIM33EAU.h>

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
int rudder = 95;

// Declare the compass class
CompassCMPS11 compass1;

//A GPS library
ParallaxSIM33EAU gps(&Serial1);


// Global variables
String inputSerial1 = "";         // a string to hold incoming data
boolean IsReadySerial1 = false;  // whether the string is complete

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
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT); //IsRobot flag

  // Initialise the GPS module
  Serial.println("Initializing GPS");
  Serial1.begin(9600);
  delay(100);

  // Initialize the compass and initial readings
  compass1.begin();

  BoatHeading = compass1.getBearing();

  // Set Servo Controller the serial baud rate.
  Serial2.begin(9600);
  delay(100);
  
  Serial2.println((byte)0xAA);
  delay(100);
  
  // Initialise the SD card
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("SD failed");
    return;
  }

  //Set the last time
  lasttime = 0;

}

void serialEvent1()
{
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
      dataFile.print("westwood17,$TMR,");
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
      dataFile.print(mywind.rollPort);
      dataFile.print(",");
      dataFile.print(mywind.rollCentre);
      dataFile.print(",");
      dataFile.print(mywind.rollStarboard);
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
  compass1.ReadCompass();
  BoatHeading = compass1.bearing;
  pitch = compass1.pitch;
  roll = compass1.roll;
  if (BoatHeading == 0) BoatHeading = oldBoatHeading;

  // Read the compass gyro
  compass1.ReadGyro();

  // Read the compass accelerator
  compass1.ReadAccelerator();

  myrudder.GyroUpdateMean(BoatHeading, compass1.gyroZ);

  myrudder.AddGyroReading(rudder, compass1.gyroZ);

  //Want to update the Wind direction estimate
  mywind.Update(BoatHeading, roll);
  mywind.UpdateDistribution(roll);
  mywind.UpdateRoll(BoatHeading, roll);

  // The Human has to control the boat if there is no GPS signal
  if (IsHuman)
  {
    // Human sets the rudder position
    setPosition(0, ch1);
    
    // Human sets the sails position
    setPosition(1, ch2);
    
    //Have sometime for the servo to move
    delay(100);
  }
  else
  {
    // Update the new rudder position

    // Setup a tack - which is estimated wind direction +/- 50 degrees
    //desiredHeading = myrudder.ChooseTarget(mywind.Direction, BoatHeading);
    
    // Update the rudder to point towards desiredHeading
    rudder = 90; //myrudder.UpdateTarget(desiredHeading, BoatHeading);

    // Robot updates the rudder position
    setPosition(0, rudder);
    //servoRudder.write(rudder);

    // Human still controls the sails
    //servoSail.write(ch2);
    setPosition(1, ch2);
  }

  // Print data to Serial Monitor window
  Serial.print("westwood17,$TMR,");
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
  Serial.print(gps.gpsdate);
  Serial.print(",");
  Serial.print(gps.gpstime);
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
  Serial.print(mywind.rollPort);
  Serial.print(",");
  Serial.print(mywind.rollCentre);
  Serial.print(",");
  Serial.println(mywind.rollStarboard);
 
}

void setPosition(uint8_t channelNumber, uint16_t ServoPosition)
{
  ServoPosition = 5.5 * ServoPosition + 1000;
  if (ServoPosition < 1000) ServoPosition = 1000;
  if (ServoPosition > 2000) ServoPosition = 2000;
  
  // Note using cyclicRedundancyCheck

  // Compact protocol (no cyclicRedundancyCheck)
  // 0x84, channel number, target low bits, target high bits
  // Remembering not to send data greater than 7 bits at a time
  Serial2.write(0x84);

  // Note data bytes have their most significant bits set to 0.
  // This means data bytes can only transmit 7-bits of information per byte.

  // Send the channelNumber. Making sure significant bit is 0.
  Serial2.write(channelNumber & 0x7F);

  // First multiply position by 4
  // Send the first 7 bits of the data
  // Making sure significant bit is 0.
  Serial2.write(ServoPosition << 2 & 0x7F);

  // Shift 7 bits to the right
  // Then send the next 7 bits of the data
  // Making sure significant bit is 0.
  // Shift 5 right = 2 left + 7 right
  Serial2.write((ServoPosition >> 5) & 0x7F);

  // Cyclic Redundancy Check (CRC) Error Detection
  // The Maestro servo controller expects an extra byte at the end of the command statement.
  // Calculate the cyclicRedundancyCheck
  uint8_t cyclicRedundancyCheck = 0; 
  cyclicRedundancyCheck = UpdateCRC(cyclicRedundancyCheck, 0x84);
  cyclicRedundancyCheck = UpdateCRC(cyclicRedundancyCheck, channelNumber & 0x7F);
  cyclicRedundancyCheck = UpdateCRC(cyclicRedundancyCheck, ServoPosition << 2 & 0x7F);
  cyclicRedundancyCheck = UpdateCRC(cyclicRedundancyCheck, (ServoPosition >> 5) & 0x7F);

  // Send the cyclicRedundancyCheck
  // As the cyclicRedundancyCheck is still data, the most significant bit must be set to 0
  // By construction the Maestro uses CRC-7, so the most significant bit is already 0. 
  Serial2.write(cyclicRedundancyCheck & 0x7F);
}

uint8_t UpdateCRC(uint8_t cyclicRedundancyCheck, uint8_t data)
{
  cyclicRedundancyCheck ^= data;

  // Loop over the bits of the byte
  for (uint8_t i = 0; i < 8; i++)
  {
    // Look at the least significant bit
    if (cyclicRedundancyCheck & 1) cyclicRedundancyCheck ^= 0x91;

    // Shift 1 bit to the right
    cyclicRedundancyCheck >>= 1;
  }

  return cyclicRedundancyCheck;
}
