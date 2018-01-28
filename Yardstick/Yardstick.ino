#include <SD.h>
#include <Servo.h>
#include <math.h>

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

//  _     _____ ____    ____  _           _             
// | |   | ____|  _ \  |  _ \(_)___ _ __ | | __ _ _   _ 
// | |   |  _| | | | | | | | | / __| '_ \| |/ _` | | | |
// | |___| |___| |_| | | |_| | \__ \ |_) | | (_| | |_| |
// |_____|_____|____/  |____/|_|___/ .__/|_|\__,_|\__, |
//                                 |_|            |___/ 
#include <Adafruit_AlphaNum_Teensy.h>
                   

//Is Robot in control of the human
bool IsHuman = true;

volatile int wifiptr = 0;
volatile int wifilen = 0;
char wifibuffer[105];

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

// LED Display
Adafruit_AlphaNum_Teensy Alpha;

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

  // LED display setup
  Alpha.begin(0x70, 1);  // pass in the address
  Alpha.clear();

  // The single zero shows the LED works.
  // It will got blank once depth signal is pickedup
  Alpha.writeDigitAscii(3, 0, false);
  Alpha.writeDisplay();
  Alpha.setBrightness(1);
  
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
  //Serial1.println("$PMTK220,200*2C");  //  5 times a second
  //Serial1.println("$PMTK220,100*2F");  // 10 times a second (10Hz)
  //At faster speeds need to set Serial1 at faster frequency
  delay(100);

  // Initialize the compass and initial readings
  compass1.begin(0x60, 0);

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

  // Set the Yardstick regression parameters. 
  // These numbers were found by trial and error
  // The rudders on Yardstick are very sensitive
  myrudder.SetGyroRegressionStatic(0.70, -65);
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

void serialEvent2()
{
  // This event is called when Serial2 receives new bytes
  //
  // Yardstick sets up a local WiFi called "Yardstick Robotic Boat" using ESP8266
  // Remember it takes some minutes for the ESP8266 to set itself up before website works
  // The boat IP address on that WiFi is  http://192.168.4.1/
  // A Windows tablet onboard connected to NMEA2000 measures the depth.
  // Every second tablet then requests the web page http://192.168.4.1/Depth=00.00
  // The Teensy (via WiFi) reads the requested webpage. So can figure out the depth
  // 
  while (Serial2.available())
  {
    // Read the new byte:
    char nextChar = (char)Serial2.read(); 
    
    // End of Message?
    if (nextChar == '\n')
    {
      // Open the wifi disk file
      File wifiFile = SD.open("wifilog.txt", FILE_WRITE);

      Serial.print("yard18,$TMR,");
      Serial.print(mytime);
      Serial.print(",$WIFI,");
      Serial.println(wifibuffer);

      // Do we have the string we are looking for?
      // Check the first 6 characters
      if (strncmp("Depth=", wifibuffer, 6) == 0)
      {
        // We match the first characters. Now check the rest of the format Depth=00.00
        // strlen gets the length of a null-terminated string. Check for the .
        // Can show reading up to 100m, approx max depth of DST 800 transducer
        if (strlen(wifibuffer) == 12 && wifibuffer[8] == '.'){
          
            // So we want to show the following characters
            // Depth= wifibuffer[6] wifibuffer[7] . wifibuffer[9] wifibuffer[10]
            // Character '0'=48, '1'=49, '2'=50, ...       '9'=57
            
            // Need to check for data quality
            if (wifibuffer[6] >= 48 && wifibuffer[6] <=57){
              Alpha.writeDigitAscii(0, wifibuffer[6]-48, false); // false = no decimal point  
            }
            if (wifibuffer[7] >= 48 && wifibuffer[7] <=57){
              Alpha.writeDigitAscii(1, wifibuffer[7]-48, true); // false = no decimal point
            }
            if (wifibuffer[9] >= 48 && wifibuffer[9] <=57){
              Alpha.writeDigitAscii(2, wifibuffer[9]-48, false); // true = decimal point
            }
            if (wifibuffer[10] >= 48 && wifibuffer[10] <=57){
              Alpha.writeDigitAscii(3, wifibuffer[10]-48, false); // false = no decimal point
            }
                       
            Alpha.writeDisplay();
        }
        else
        {
          // We have lost the depth reading. So turn off the display
          Alpha.clear();
          Alpha.writeDisplay();
        }
      }
        
      // Can we write to the file
      if (wifiFile)
      {
        wifiFile.print("yard18,$TMR,");
        wifiFile.print(mytime);
        wifiFile.print(",$WIFI,");
        wifiFile.println(wifibuffer);       
        wifiFile.close();
      }
      
      // Reset the message counter
      wifiptr = 0;
    }
    else
    {
      // Message still being built
      wifibuffer[wifiptr] = nextChar;
      // Ensure a null-terminated string
      wifibuffer[wifiptr + 1] = '\0';

      // Move on to the next character
      wifiptr = wifiptr + 1;
      if (wifiptr >= 100) wifiptr = 100;
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
  Serial.print("yard18,$TMR,");
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
    dataFile.print("yard18,$TMR,");
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
}


