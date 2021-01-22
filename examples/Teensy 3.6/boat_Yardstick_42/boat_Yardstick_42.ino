// Teensy 3.6 and Yardstick
// Copyright (C) 2020 https://www.roboticboat.uk
// af877a4a-a80e-402b-8de9-7addb3611e51
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


#define N2K_SOURCE 15

#define SERIAL_PRINT true

// Libraries are available here
// https://github.com/pwesson/roboticboat

#include <SD.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>

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

//  ____                _       _               
// |  _ \   _   _    __| |   __| |   ___   _ __ 
// | |_) | | | | |  / _` |  / _` |  / _ \ | '__|
// |  _ <  | |_| | | (_| | | (_| | |  __/ | |   
// |_| \_\  \__,_|  \__,_|  \__,_|  \___| |_|   
                                                
#include <Rudder.h>

// __        __  _               _
// \ \      / / (_)  _ __     __| |
//  \ \ /\ / /  | | | '_ \   / _` |
//   \ V  V /   | | | | | | | (_| |
//    \_/\_/    |_| |_| |_|  \__,_|
//  

#include <Wind.h>

// These libraries were downloaded from https://github.com/ttlappalainen
#include <NMEA2000_CAN.h>
#include <N2kMsg.h>
#include <NMEA2000_teensy.h>
#include <N2kMessages.h>
#include <N2kDeviceList.h>

// Pointers to NMEA 2000 devices
tN2kDeviceList *pN2kDeviceList;
const tNMEA2000::tDevice* pDevice;
const unsigned long* listPGN;

double nmea2000Depth = 0;
double nmea2000Speed = 0;
double nmea2000Temperature = 0;
double nmea2000latitude = 0;
double nmea2000longitude = 0;
double debug1 = 0;
double debug2 = 0;
  
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
int rudder = 90;

// Declare the compass class
CompassCMPS11 compass1;

//A GPS library
ParallaxSIM33EAU gps(&Serial1);

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

Rudder myrudder;

void setup()
{
  // Keep the User informed
  Serial.begin(9600);

  // initialize the LED pin as an output.
  pinMode(13, OUTPUT);

  // Setup the input pins used received by the RC receiver
  pinMode(7, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT); //IsRobot flag

  // Initialise the GPS module
  Serial.println("Initializing GPS");
  Serial1.begin(9600);
  delay(100);

  gps.SelectSentences();
  delay(100);

  // WiFi module
  Serial2.begin(115200);
  delay(100);

  // Initialize the compass and initial readings
  compass1.begin();
  
  // Do not forward bus messages
  NMEA2000.EnableForward(false);

  // NMEA2000 event function to call
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  // Define address (50) to something not already on the network
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 50);

  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);

  // Open connection to the NMEA2000 network
  NMEA2000.Open();

  BoatHeading = compass1.getBearing();
  oldBoatHeading = BoatHeading;

  // Set Servo Controller the serial baud rate.
  Serial5.begin(9600);
  delay(100);
  
  Serial5.println((byte)0xAA);
  delay(100);
  
  // Initialise the SD card
  InitialiseSDcard(5);
 
  //Set the last time
  lasttime = 0;

  

}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {

    // We want to process the message
  if (N2kMsg.PGN == 128259L) Speed(N2kMsg);
  if (N2kMsg.PGN == 128267L) WaterDepth(N2kMsg);
  if (N2kMsg.PGN == 129029L) GNSS(N2kMsg);
  if (N2kMsg.PGN == 130311L) WaterTemperature(N2kMsg);
  
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

    debug1 = nmea2000Speed;
    debug2 = nmea2000Temperature;
    if (debug1 == 0) debug1 = nmea2000latitude;
    if (debug2 == 0) debug2 = nmea2000longitude;
   
    // Create WiFi Message
    sprintf(WiFiChecksumMessage,"MSG,%lu,%.0f,%d,%d,%.8f,%.8f,%d,%d,%d,%d,%.2f,%.2f,%.2f",mytime,BoatHeading,pitch,roll,gps.latitude,gps.longitude,gps.numsatelites,ch1,ch2,ch3,nmea2000Depth,debug1,debug2);
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
      dataFile.print("yardstick42,$TMR,");
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
  
      dataFile.print(",$NMEA,");
      dataFile.print(nmea2000latitude,8);
      dataFile.print(",");
      dataFile.print(nmea2000longitude,8);
      dataFile.print(",");
      dataFile.print(nmea2000Depth,4);
      dataFile.print(",");
      dataFile.print(nmea2000Speed,4);
      dataFile.print(",");
      dataFile.print(nmea2000Temperature,4);
  
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
  ch1 = pulseIn(7, HIGH, 20000);
  ch2 = pulseIn(16, HIGH, 20000);
  ch3 = pulseIn(17, HIGH, 20000);

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

  // The Human has to control the boat if there is no GPS signal
  if (IsHuman)
  {
    // Human sets the rudder position
    setPosition(0, ch1);
    
    // Human sets the motor to position
    setPosition(1, ch2);

    // Human sets the motor to position
    setPosition(2, ch2);
    
    //Have sometime for the servo to move
    delay(100);
  }
  else
  {
    // Update the new rudder position
    rudder = 90; // myrudder.Update(gps.latitude, gps.longitude, BoatHeading);

    // Human sets the rudder position
    setPosition(0, ch1);
    
    // Human sets the motor to position
    setPosition(1, ch2);

    // Human sets the motor to position
    setPosition(2, ch2);
  }

  if (SERIAL_PRINT){

    // Print data to Serial Monitor window
    Serial.print("yardstick42,$TMR,");
    Serial.print(mytime);
    Serial.print(",$RC,");
    Serial.print(ch1);
    Serial.print(",");
    Serial.print(ch2);
    Serial.print(",");
    Serial.print(ch3);
    Serial.print(",$CMP,");
    Serial.print(BoatHeading);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",$GYR,");
    Serial.print(compass1.gyroX, 4);
    Serial.print(",");
    Serial.print(compass1.gyroY, 4);
    Serial.print(",");
    Serial.print(compass1.gyroZ - myrudder.gyrozmean, 4);
    Serial.print(",");
    Serial.print(myrudder.gyro.slope);
    Serial.print(",");
    Serial.print(myrudder.gyro.intercept);
    Serial.print(",\t$GPS,");
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
    Serial.print("\t$NMEA,");
    Serial.print(nmea2000latitude,8);
    Serial.print(",");
    Serial.print(nmea2000longitude,8);
    Serial.print(",");
    Serial.print(nmea2000Depth,4);
    Serial.print(",");
    Serial.print(nmea2000Speed,4);
    Serial.print(",");
    Serial.println(nmea2000Temperature,4);
  }

  // Read the NMEA 2000 network

  NMEA2000.ParseMessages();
  CheckDevices();
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


// NMEA 2000 network data

void WaterDepth(const tN2kMsg &N2kMsg) {
  
    unsigned char SID;
    double DepthBelowTransducer;
    double Offset;

    if (ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset) ) 
    {
        nmea2000Depth = DepthBelowTransducer;
    
        File dataFile = SD.open("Depth.txt", FILE_WRITE);

        if (dataFile)
        {
              dataFile.print("yardstick42,Depth,");
              dataFile.print(nmea2000latitude,10); 
              dataFile.print(","); 
              dataFile.print(nmea2000longitude,10);
              dataFile.print(","); 
              dataFile.println(DepthBelowTransducer);
            
              dataFile.close();
        }
    }
}

void Speed(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  double WaterReferenced;
  double GroundReferenced;
  tN2kSpeedWaterReferenceType SWRT;

  if (ParseN2kPGN128259(N2kMsg, SID, WaterReferenced, GroundReferenced, SWRT))
  {
    nmea2000Speed = WaterReferenced;

    File dataFile = SD.open("Speed.txt", FILE_WRITE);

    if (dataFile)
    {
        dataFile.print("yardstick42,Depth,");
        dataFile.print(nmea2000latitude,10); 
        dataFile.print(","); 
        dataFile.print(nmea2000longitude,10);
        dataFile.print(","); 
        dataFile.println(WaterReferenced);
            
        dataFile.close();
    }
  }
}

void WaterTemperature(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  tN2kTempSource TempSource;
  double Temperature;
  tN2kHumiditySource HumiditySource;
  double Humidity;
  double AtmosphericPressure;

  if (ParseN2kPGN130311(N2kMsg, SID, TempSource, Temperature, HumiditySource, Humidity, AtmosphericPressure))
  {
    nmea2000Temperature = Temperature;

    File dataFile = SD.open("Temp.txt", FILE_WRITE);

    if (dataFile)
    {
        dataFile.print("yardstick42,Temp,");
        dataFile.print(nmea2000latitude,10); 
        dataFile.print(","); 
        dataFile.print(nmea2000longitude,10);
        dataFile.print(","); 
        dataFile.println(Temperature);
            
        dataFile.close();
    }

  }
}


void GNSS(const tN2kMsg &N2kMsg) {
  
    unsigned char SID;
    uint16_t DaysSince1970;
    double SecondsSinceMidnight; 
    double Latitude;
    double Longitude;
    double Altitude; 
    tN2kGNSStype GNSStype;
    tN2kGNSSmethod GNSSmethod;
    unsigned char nSatellites;
    double HDOP;
    double PDOP;
    double GeoidalSeparation;
    unsigned char nReferenceStations;
    tN2kGNSStype ReferenceStationType;
    uint16_t ReferenceSationID;
    double AgeOfCorrection;

    if (ParseN2kGNSS(N2kMsg,SID,DaysSince1970,SecondsSinceMidnight,
                  Latitude,Longitude,Altitude,
                  GNSStype,GNSSmethod,
                  nSatellites,HDOP,PDOP,GeoidalSeparation,
                  nReferenceStations,ReferenceStationType,ReferenceSationID,
                  AgeOfCorrection) ) 
      {
        //Store the values
        nmea2000latitude = Latitude;
        nmea2000longitude = Longitude;

        File dataFile = SD.open("nmea2000.txt", FILE_WRITE);

        if (dataFile)
        {
              dataFile.print("yardstick42,GNSS,");
              dataFile.print(DaysSince1970); dataFile.print(",");
              dataFile.print(SecondsSinceMidnight); dataFile.print(",");
              dataFile.print(Latitude,10); dataFile.print(",");
              dataFile.print(Longitude,10); dataFile.print(",");
              dataFile.print(Altitude); dataFile.print(",");
              dataFile.print(GNSStype); dataFile.print(",");
              dataFile.print(GNSSmethod); dataFile.print(",");
              dataFile.print(nSatellites); dataFile.print(",");
              dataFile.print(HDOP); dataFile.print(",");
              dataFile.print(PDOP); dataFile.print(",");
              dataFile.print(GeoidalSeparation); dataFile.print(",");
              dataFile.println(nReferenceStations);

              dataFile.close();
        }
    }
}

void CheckDevices() {

  // Only find out information if a new device is detected
  if (!pN2kDeviceList->ReadResetIsListUpdated() ) return;

  // Loop over possible devices
  for (uint8_t i = 0; i < N2kMaxBusDevices; i++) {

      // Using the Source to find the device
      pDevice = pN2kDeviceList->FindDeviceBySource(i);
      
      // If nothing detected then continue to next device
      if (pDevice == 0) continue;

      uint8_t j;
      
      // We have detected a device, so upgrade User with information
      Serial.println("Device=" ); Serial.print(pDevice->GetModelID()); 
      Serial.print(",Source="); Serial.print(pDevice->GetSource()); 
      Serial.print(",Manufactuer="); Serial.print(pDevice->GetManufacturerCode());
      Serial.print(",Unique="); Serial.print(pDevice->GetUniqueNumber());
      Serial.print(",Software="); Serial.print(pDevice->GetSwCode());
      Serial.println(",Model="); Serial.print(pDevice->GetModelVersion());
      
      listPGN = pDevice->GetTransmitPGNs();
      Serial.print("PGN=");
      for (j=0; listPGN[j]!=0; j++){
        Serial.print(" "); Serial.print(listPGN[j]);
      }
      Serial.println();
    
      File dataFile = SD.open("nmea2000.txt", FILE_WRITE);
    
      if (dataFile)
      {
          dataFile.print("Device=" ); dataFile.print(pDevice->GetModelID()); 
          dataFile.print(",Source="); dataFile.print(pDevice->GetSource()); 
          dataFile.print(",Manufactuer="); dataFile.print(pDevice->GetManufacturerCode());
          dataFile.print(",Unique="); dataFile.print(pDevice->GetUniqueNumber());
          dataFile.print(",Software="); dataFile.print(pDevice->GetSwCode());
          dataFile.print(",Model="); dataFile.print(pDevice->GetModelVersion());
          dataFile.print(",PGN=");
        for (j=0; listPGN[j]!=0; j++){
            dataFile.print(" "); dataFile.print(listPGN[j]);
        }
          dataFile.println();
          dataFile.close();
      }
  }
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

void setPosition(uint8_t channelNumber, uint16_t ServoPosition)
{
  ServoPosition = 5.5 * ServoPosition + 1000;
  if (ServoPosition < 1000) ServoPosition = 1000;
  if (ServoPosition > 2000) ServoPosition = 2000;
  // Note using cyclicRedundancyCheck

  // Compact protocol (no cyclicRedundancyCheck)
  // 0x84, channel number, target low bits, target high bits
  // Remembering not to send data greater than 7 bits at a time
  Serial5.write(0x84);

  // Note data bytes have their most significant bits set to 0.
  // This means data bytes can only transmit 7-bits of information per byte.

  // Send the channelNumber. Making sure significant bit is 0.
  Serial5.write(channelNumber & 0x7F);

  // First multiply position by 4
  // Send the first 7 bits of the data
  // Making sure significant bit is 0.
  Serial5.write(ServoPosition << 2 & 0x7F);

  // Shift 7 bits to the right
  // Then send the next 7 bits of the data
  // Making sure significant bit is 0.
  // Shift 5 right = 2 left + 7 right
  Serial5.write((ServoPosition >> 5) & 0x7F);

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
  Serial5.write(cyclicRedundancyCheck & 0x7F);
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
