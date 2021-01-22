// Teensy 3.6 doing a straight line
// Copyright (C) 2017 https://www.roboticboat.uk
// e65b84ac-12f2-40f4-a6d5-90f73fd1c033
//
// https://www.roboticboat.uk/boats/Yardstick/2017-08-12-Yardstick-Chipstead/20170812125143-50101-map.html
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


#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

//  ____                _       _               
// |  _ \   _   _    __| |   __| |   ___   _ __ 
// | |_) | | | | |  / _` |  / _` |  / _ \ | '__|
// |  _ <  | |_| | | (_| | | (_| | |  __/ | |   
// |_| \_\  \__,_|  \__,_|  \__,_|  \___| |_| 

#include <Rudder.h>

//   ____   __  __   ____    ____      _    _
//  / ___| |  \/  | |  _ \  / ___|    / |  / |
// | |     | |\/| | | |_) | \___ \    | |  | |
// | |___  | |  | | |  __/   ___) |   | |  | |
//  \____| |_|  |_| |_|     |____/    |_|  |_|

#include <CompassCMPS11.h>

//     _          _            __                  _   _        ____   ____    ____
//    / \      __| |   __ _   / _|  _ __   _   _  (_) | |_     / ___| |  _ \  / ___|
//   / _ \    / _` |  / _` | | |_  | '__| | | | | | | | __|   | |  _  | |_) | \___ \ .
//  / ___ \  | (_| | | (_| | |  _| | |    | |_| | | | | |_    | |_| | |  __/   ___) |
// /_/   \_\  \__,_|  \__,_| |_|   |_|     \__,_| |_|  \__|    \____| |_|     |____/

#include <AdaUltimateGPS3.h>

// __        __  _               _
// \ \      / / (_)  _ __     __| |
//  \ \ /\ / /  | | | '_ \   / _` |
//   \ V  V /   | | | | | | | (_| |
//    \_/\_/    |_| |_| |_|  \__,_|
//

#include <Wind.h>

//
// The GPS is set on serialEvent1 to pick up the GPS messages
// Square
// A: 51.28750000,  0.153888889
// B: 51.28839932,  0.153888889
// C: 51.28839932,  0.152450896
// D: 51.28750000,  0.152450896
// A: 51.28750000,  0.153888889
// float listlatitude[] = {51.28750000, 51.28839932, 51.28839932, 51.28750000, 51.28750000};
// float listlongitude[] = {0.153888889, 0.153888889, 0.152450896, 0.152450896, 0.153888889};
//

// 800m line
// A: 51.289822222,0.160686111
// B: 51.289983333,0.160505555
// C: 51.287491375,0.149560475
// D: 51.287427800,0.154475000
//

int nlist = 0;
float listlatitude[] = {51.289822222, 51.289983333, 51.287491375, 51.287427800};
float listlongitude[] = {0.160686111, 0.160505555, 0.149560475, 0.154475000};

// Chipstead lake
//int nlist = 0;
//float listlatitude[] = { 51.2876453803405,  51.2883124767495, 51.2883461648971,  51.2876790684929,  51.2877127564939,  51.2883798528934,  51.2884135407382,  51.2877464443435,  51.2877801320417,  51.2884472284317,  51.2884809159737,  51.2878138195884,  51.2878475069838, 51.2885146033643,  51.2885482906035,  51.2878811942278,  51.2879148813203,  51.2885819776913,  51.2886156646277, 51.2879485682615,  51.2879822550512,  51.2886493514126,  51.2886830380462,  51.2880159416895,  51.2880496281765,  51.2887167245284,  51.2887504108591,  51.288083314512,   51.2881170006961,  51.2887840970385,  51.2888177830664,  51.2881506867288,  51.2881843726101,  51.288851468943,  51.2888851546681,  51.2882180583399,  51.2882517439184,  51.2889188402418,  51.2889525256641,  51.2882854293454,  51.2883191146211,  51.288986210935,   51.2890198960545,  51.2883527997453,  51.2883864847181,  51.2890535810225,  51.2890872658392,  51.2884201695395,  51.2884538542095,  51.2891209505044, 51.2891546350183,  51.2884875387281,  51.2885212230953,  51.2891883193807,  51.2892220035917,  51.2885549073111,  51.2885885913754,  51.2892556876513,  51.2892893715595,  51.2886222752883, 51.2886559590499,  51.2893230553163,  51.2893567389217,  51.28868964266,    51.2878475069838};
//float listlongitude[] = { 0.152966671985588, 0.15253571718532, 0.152669046071425, 0.153100001184697, 0.153233330579473, 0.152802375153143, 0.152935704430498, 0.153366660169882, 0.153499989955894, 0.153069033903478, 0.153202363572087, 0.15363331993753, 0.153766650114819, 0.153335693436337, 0.153469023496198, 0.153899980487728, 0.154033311056279, 0.153602353751692, 0.15373568420282, 0.154166641820417, 0.154299972780205, 0.153869014849571, 0.154002345691954, 0.154433303935618, 0.154566635286694, 0.154135676729964, 0.154269007963628, 0.154699966833327, 0.154833298575649, 0.154402339392905, 0.154535671017822, 0.154966630513576, 0.155099962647155, 0.15466900283837, 0.154802334854545, 0.155233294976347, 0.155366627501168, 0.154935667066364, 0.155068999473821, 0.155499960221654, 0.155633293137769, 0.155202332076899, 0.155335664875624, 0.155766626249507, 0.155899959556819, 0.155468997869984, 0.155602331059974, 0.156033293059835, 0.156166626758565, 0.15573566444562, 0.155868998026876, 0.156299960652765, 0.156433294742683, 0.156002331803793, 0.156135665776348, 0.156566629028268, 0.156699963509447, 0.156268999944527, 0.156402334308358, 0.15683329818624, 0.156966633058742, 0.156535668867832, 0.156669003622957, 0.157099968126804, 0.153766650114819};


//Is Robot in control of the human
bool IsHuman = true;

volatile int wifiptr = 0;
char wifibuffer[60];

//Radio Control Receiver
int ch1 = 0;
int ch2 = 0;
int ch3 = 0;

//LED light
const int ledPin = 13;
bool islighton = false;

// Timer
unsigned long mytime;
unsigned long lasttime;

//Radio Control Servo
Servo servoRudder;
Servo servoSails;
int rudder = 90;

// Declare the compass class
CompassCMPS11 compass1;

//A GPS library
AdaUltimateGPS3 gps(&Serial1);

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

  // Setup the input pins used received by the RC receiver
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(35, INPUT); //IsRobot flag

  // initialize the LED pin as an output.
  pinMode(ledPin, OUTPUT);

  // Setup the output pins used by the servos
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  //Setup servo connection for rudder and sail
  servoRudder.attach(20);
  servoSails.attach(21);

  //Set the rudder range
  myrudder.maxposition = 90 + 40;
  myrudder.minposition = 90 - 40;

  // Initialise the GPS module
  Serial.println("Initializing GPS");
  Serial1.begin(9600);
  delay(100);

  // Just want RMC and GGA messages
  // Send setup command to the GPS module
  Serial1.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");
  delay(100);

  // 1MHz update
  // Send setup command to the GPS module
  Serial1.println("$PMTK220,1000*1F");
  delay(100);

  // Initialize the compass and initial readings
  compass1.begin();

  BoatHeading = compass1.getBearing();
  oldBoatHeading = BoatHeading;

  // Initialise the SD card
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Serial.println("SD failed");
    return;
  }

  // Setup connection to the ESP8266 WiFi module
  Serial2.begin(9600);

  //Set the last time
  lasttime = 0;
}

void serialEvent1()
{
  //This event is called when Serial1 receives new bytes
  gps.listen();
}

void serialEvent2()
{
  //This event is called when Serial2 receives new bytes
  while (Serial2.available())
  {
    // Read the new byte:
    char nextChar = (char)Serial2.read();
    if (nextChar == '\n')
    {
      // Message has come.
      wifiptr = 0;
      
      String myString = wifibuffer;
      if (myString.indexOf("value") >= 0)
      {
        int i = myString.indexOf("_") + 1;
        int j = myString.length();
        if (i > 1 && j > 0) {
          Serial.print("value ");
          Serial.println(myString.substring(i, j));
          myrudder.sensitivity = myString.substring(i, j).toInt();
          if (myrudder.sensitivity < 2) myrudder.sensitivity = 2;
        }
      }
      if (myString.indexOf("local") >= 0)
      {
        int i = myString.indexOf("_") + 1;
        int j = myString.length();
        if (i > 1 && j > 0) {
          Serial.print("local ");
          Serial.println(myString.substring(i, j));
          myrudder.localdistance =  myString.substring(i, j).toInt();
          if (myrudder.localdistance < 2) myrudder.localdistance = 2;
        }
      }
    }
    else
    {
      // Message still being built
      wifibuffer[wifiptr] = nextChar;
      wifibuffer[wifiptr + 1] = '\0';
      wifiptr = wifiptr + 1;
      if (wifiptr >= 50) wifiptr = 49;
    }
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

    //The GPS signal must be active (not void)
    if (gps.gpsstatus == 'A')
    {
      //Flash the light when receiving a GPS signal
      if (islighton)
      {
        // Turn off the light
        digitalWrite(ledPin, LOW);
        islighton = false;
      }
      else
      {
        // Turn on the light
        digitalWrite(ledPin, HIGH);
        islighton = true;
      }
    }
    else
    {
      // Turn off the light as no valid GPS location
      digitalWrite(ledPin, LOW);
      islighton = false;
    }
  }

  // Read the pulse width of each channel.
  // The problem here is that the code blocks until it has read the signal.
  // This can take up to 20ms (per channel) if the radio control transmitter is off (not transmitting)
  ch1 = pulseIn(16, HIGH, 20000);
  ch2 = pulseIn(17, HIGH, 20000);
  ch3 = pulseIn(35, HIGH, 20000);

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
      nlist = 0;
      myrudder.NewLine(gps.latitude, gps.longitude, listlatitude[0], listlongitude[0]);
    }

    // Have we reached the next waypoint F?
    if (myrudder.alongtrack >= myrudder.alongtrackobjective)
    {
      // Move to the next waypoint 
      nlist = nlist+1;
      if (nlist >4) nlist = 4;
      myrudder.NewLine(gps.latitude, gps.longitude, listlatitude[nlist], listlongitude[nlist]);
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
  gyrox = compass1.getGyroX() * compass1.gyroScale;
  gyroy = compass1.getGyroY() * compass1.gyroScale;
  gyroz = compass1.getGyroZ() * compass1.gyroScale;

  // Read the compass accelerator
  accelx = compass1.getAcceleroX() * compass1.accelScale;
  accely = compass1.getAcceleroY() * compass1.accelScale;
  accelz = compass1.getAcceleroZ() * compass1.accelScale;

  myrudder.GyroUpdateMean(BoatHeading, gyroz);

  //myrudder.AddGyroReading(rudder, gyroz);

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
  Serial.print("v10,$TMR,");
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
  Serial.print("\t$WND,");
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

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile)
  {
    dataFile.print("v10,$TMR,");
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
