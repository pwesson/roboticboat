#include <SD.h>
#include <Servo.h>
#include <math.h>
#include <Arduino.h>

// #define USE_DUE_CAN 1
#define N2K_SOURCE 15

//  ____            _     _           
// |  _ \ _   _  __| | __| | ___ _ __ 
// | |_) | | | |/ _` |/ _` |/ _ \ '__|
// |  _ <| |_| | (_| | (_| |  __/ |   
// |_| \_\\__,_|\__,_|\__,_|\___|_|     
                                                
#include <Rudder.h>

//   ____ __  __ ____  ____   _  _ 
//  / ___|  \/  |  _ \/ ___| /  / |
// | |   | |\/| | |_) \___ \ |  | |
// | |___| |  | |  __/ ___)  |  | |
//  \____|_|  |_|_|   |____/ |_ |_|
                               
#include <CompassCMPS11_Teensy.h>

//     _       _        __            _ _      ____ ____  ____  
//    / \   __| | __ _ / _|_ __ _   _(_) |_   / ___|  _ \/ ___| .
//   / _ \ / _` |/ _` | |_| '__| | | | | __| | |  _| |_) \___ \ .
//  / ___ \ (_| | (_| |  _| |  | |_| | | |_  | |_| |  __/ ___) |
// /_/   \_\__,_|\__,_|_| |_|   \__,_|_|\__|  \____|_|   |____/ 
                                                              
#include <AdaGPS.h>

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

double gpslatitude = 0;
double gpslongitude = 0;

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
Servo servoMotors;
int rudder = 90;

// Declare the compass class
CompassCMPS11_Teensy compass1;

//A GPS library
AdaGPS gps;

//
// The Adafruit GPS flashes oncee every 15 seconds when it has found a fix
//

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

  // initialize the LED pin as an output.
  pinMode(13, OUTPUT);

  // Setup the input pins used received by the RC receiver
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(35, INPUT); //IsRobot flag

  // Setup the output pins used by the servos
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  //Setup servo connection for rudder and sail
  servoRudder.attach(20);
  servoMotors.attach(21);

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

  // Send setup command to the GPS module
  Serial1.println("$PMTK220,1000*1F"); // Once per second (1Hz)
  delay(100);

  // Initialize the compass and initial readings
  compass1.begin(0x60, 0);

  BoatHeading = compass1.getBearing();
  oldBoatHeading = BoatHeading;

 // Initialise the SD card
  InitialiseSDcard(10);

  //Set the last time
  lasttime = 0;

  // Set the Yardstick regression parameters. 
  // These numbers were found by trial and error
  // The rudders on Yardstick are very sensitive
  myrudder.SetGyroRegressionStatic(0.70, -65);

  // Do not forward bus messages
  NMEA2000.EnableForward(false);

  // NMEA2000 event function to call
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  // Define address (50) to something not already on the network
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 50);

  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);

  // Open connection to the NMEA2000 network
  NMEA2000.Open();
  
}

void serialEvent1()
{
  //This event is called when Serial1 receives new bytes
  //It needs to be an event else GPS characters will be missed
  while (Serial1.available())
  {
    // Read the new byte:
    gps.read((char)Serial1.read());
  }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {

  // We want to process the message
  if (N2kMsg.PGN == 128259L) Speed(N2kMsg);
  if (N2kMsg.PGN == 128267L) WaterDepth(N2kMsg);
  if (N2kMsg.PGN == 129029L) GNSS(N2kMsg);
  if (N2kMsg.PGN == 130311L) WaterTemperature(N2kMsg);
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

  // The Human has to control the boat if there is no GPS signal
  if (IsHuman || gps.gpsstatus == 'V')
  {
    // Human sets the rudder position
    rudder = ch1;
    servoRudder.write(rudder);

    // Human sets the motor speed
    servoMotors.write(ch2);

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
    servoMotors.write(ch2);
  }


  // Print data to Serial Monitor window
  Serial.print("yardstick21,$TMR,");
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
    dataFile.print("yardstick21,$TMR,");
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

  NMEA2000.ParseMessages();
  CheckDevices();
}

void WaterDepth(const tN2kMsg &N2kMsg) {

  unsigned char SID;
  double DepthBelowTransducer;
  double Offset;

  if (ParseN2kWaterDepth(N2kMsg, SID, DepthBelowTransducer, Offset) )
  {
    Serial.print("v8,Depth,");
    Serial.println(DepthBelowTransducer);
    
    File dataFile = SD.open("nmea2000.txt", FILE_WRITE);

    if (dataFile)
    {
      dataFile.print("yardstick21,Depth,");
      dataFile.print(DepthBelowTransducer); dataFile.print(",");
      dataFile.print(gpslatitude, 10); dataFile.print(",");
      dataFile.println(gpslongitude, 10);

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
    Serial.print("yardstick21,Speed,");
    Serial.println(WaterReferenced);

    File dataFile = SD.open("nmea2000.txt", FILE_WRITE);

    if (dataFile)
    {
      dataFile.print("yardstick21,Speed,");
      dataFile.print(WaterReferenced); dataFile.print(",");
      dataFile.print(gpslatitude, 10); dataFile.print(",");
      dataFile.println(gpslongitude, 10);

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
    Serial.print("yardstick21,Temperature,");
    Serial.println(Temperature);

    File dataFile = SD.open("nmea2000.txt", FILE_WRITE);

    if (dataFile)
    {
      dataFile.print("yardstick21,Temperature,");
      dataFile.print(Temperature - 273.15); dataFile.print(",");
      dataFile.print(gpslatitude, 10); dataFile.print(",");
      dataFile.println(gpslongitude, 10);

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

  if (ParseN2kGNSS(N2kMsg, SID, DaysSince1970, SecondsSinceMidnight,
                   Latitude, Longitude, Altitude,
                   GNSStype, GNSSmethod,
                   nSatellites, HDOP, PDOP, GeoidalSeparation,
                   nReferenceStations, ReferenceStationType, ReferenceSationID,
                   AgeOfCorrection) )
  {
    //Store the values
    gpslatitude = Latitude;
    gpslongitude = Longitude;

    Serial.print("yardstick21,GNSS,");
    Serial.print(DaysSince1970); Serial.print(",");
    Serial.print(SecondsSinceMidnight); Serial.print(",");
    Serial.print(Latitude, 10); Serial.print(",");
    Serial.print(Longitude, 10); Serial.print(",");
    Serial.print(Altitude); Serial.print(",");
    Serial.print(GNSStype); Serial.print(",");
    Serial.print(GNSSmethod); Serial.print(",");
    Serial.print(nSatellites); Serial.print(",");
    Serial.print(HDOP); Serial.print(",");
    Serial.print(PDOP); Serial.print(",");
    Serial.print(GeoidalSeparation); Serial.print(",");
    Serial.println(nReferenceStations);

    File dataFile = SD.open("nmea2000.txt", FILE_WRITE);

    if (dataFile)
    {
      dataFile.print("yardstick21,GNSS,");
      dataFile.print(DaysSince1970); dataFile.print(",");
      dataFile.print(SecondsSinceMidnight); dataFile.print(",");
      dataFile.print(Latitude, 10); dataFile.print(",");
      dataFile.print(Longitude, 10); dataFile.print(",");
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
    Serial.print("Device=" ); Serial.print(pDevice->GetModelID());
    Serial.print(",Source="); Serial.print(pDevice->GetSource());
    Serial.print(",Manufactuer="); Serial.print(pDevice->GetManufacturerCode());
    Serial.print(",Unique="); Serial.print(pDevice->GetUniqueNumber());
    Serial.print(",Software="); Serial.print(pDevice->GetSwCode());
    Serial.print(",Model="); Serial.print(pDevice->GetModelVersion());
    listPGN = pDevice->GetTransmitPGNs();
    Serial.print(",PGN=");
    for (j = 0; listPGN[j] != 0; j++) {
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
      for (j = 0; listPGN[j] != 0; j++) {
        dataFile.print(" "); dataFile.print(listPGN[j]);
      }
      dataFile.println();
      dataFile.close();
    }
  }
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

void InfoTest()
{
  Sd2Card SDcard;
  SdVolume volume;
  SdFile root;

  // Card information
  Serial.print("\nCard info: ");
  switch(SDcard.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.print("SD1"); break;
    case SD_CARD_TYPE_SD2:
      Serial.print("SD2"); break;
    case SD_CARD_TYPE_SDHC:
      Serial.print("SDHC"); break;
    default:
      Serial.print("Unknown");
  }

  // Find the volume on the SD card
  if (!volume.init(SDcard)) {
    Serial.println("\nNo FAT16/FAT32 partition.");
    return;
  }

  // FAT type
  uint32_t volumesize;
  Serial.print(", FAT"); Serial.print(volume.fatType(), DEC);
  Serial.print(", ");
  
  // Sector size (or Blocks) is fixed at 512 bytes
  volumesize = volume.blocksPerCluster() * volume.clusterCount() * 512;
  Serial.print(volumesize/1024);
  Serial.println(" Kb"); 

  Serial.println("\nFiles on the SD card: ");
  root.openRoot(volume);
  
  // list all files in the card with date and size
  root.ls(LS_R | LS_DATE | LS_SIZE);
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

