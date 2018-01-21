#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

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
                                                                                   
#include <AdaGPS.h>

// __        __  _               _
// \ \      / / (_)  _ __     __| |
//  \ \ /\ / /  | | | '_ \   / _` |
//   \ V  V /   | | | | | | | (_| |
//    \_/\_/    |_| |_| |_|  \__,_|
//  

#include <Wind.h>

//
// The GPS is set on serialEvent1 to pick up the GPS messages
//


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
CompassCMPS11 compass1;

//A GPS library
AdaGPS gps;

// The Adafruit GPS flashes oncee every 15 seconds when it has found a fix
//

// Global variables
String inputSerial1 = "";         // a string to hold incoming data
boolean IsReadySerial1 = false;  // whether the string is complete

// Setup the weighted least squares
float bearing = 0;
float oldbearing = 0;
float bearingchanged = 0;

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
  pinMode(16, INPUT); 
  pinMode(17, INPUT);
  pinMode(35, INPUT); //IsRobot flag

  // Setup the output pins used by the servos
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);

  //Setup servo connection for rudder and sail
  servoRudder.attach(20);
  servoSails.attach(21);

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
  delay(100);
  bearing = compass1.getBearing();
  oldbearing = bearing;
  
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
      //This event is called when Serial2 receives new bytes
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
     bearingchanged = bearing - oldbearing;
     
     // Ensure the change in degree is between -180 to 180 degrees.
     if (bearingchanged > 180) bearingchanged -= 360;
     if (bearingchanged < -180) bearingchanged += 360;

     // Update the rudder machine learning with a new 1 second observation 
     myrudder.AddReading(mytime, ch1, bearingchanged);

     // Update the old bearing
     oldbearing = bearing;

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
  ch1 = map(ch1,1000,2000,0,180);
  ch2 = map(ch2,1000,2000,0,180);
  ch3 = map(ch3,1000,2000,0,180);
  
  //If the left RC lever is down - turn on robot
  if (ch3 > 100)
  { 
    //The transmitter is indicating to turn on the Robot by the position of channel2
    if (IsHuman)
    {
      //The Human was in control, so now initialise the Robot task
      
      //The Robot wants to know the line AB to follow. 
      myrudder.NewLine(gps.latitude, gps.longitude, bearing);   
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
    rudder = myrudder.Update(gps.latitude, gps.longitude, bearing);
      
    // Robot updates the rudder position
    servoRudder.write(rudder);

    // Human still controls the sails
    servoSails.write(ch2);
  }

  // Read the compass. 
  bearing = compass1.getBearing();
  pitch = compass1.getPitch();
  roll = compass1.getRoll();

  //Note sometimes the compass may return 0 if it can't read.
  //In this case, keep the old bearing
  if (bearing == 0) bearing = oldbearing;

  //Want to update the Wind direction estimate
  mywind.Update(bearing, roll);

  // Print data to Serial Monitor window
  Serial.print("b1,$TMR,");
  Serial.print(mytime);
  Serial.print(",$RC,");
  Serial.print(ch1);
  Serial.print(",");
  Serial.print(ch2);  
  Serial.print(",");
  Serial.print(ch3);  
  Serial.print(",$CMP,");
  Serial.print(bearing);
  Serial.print(",");
  Serial.print(pitch); 
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",$ACC,");
  Serial.print(compass1.getAcceleroX() * compass1._accelScale,4);
  Serial.print(",");
  Serial.print(compass1.getAcceleroY() * compass1._accelScale,4);
  Serial.print(",");
  Serial.print(compass1.getAcceleroZ() * compass1._accelScale,4);
  Serial.print(",$GPS,");
  Serial.print(gps.gpsdate,0);
  Serial.print(",");
  Serial.print(gps.gpstime,0);
  Serial.print(",");
  Serial.print(gps.latitude,8);
  Serial.print(",");
  Serial.print(gps.latNS);
  Serial.print(",");
  Serial.print(gps.longitude,8);
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
  Serial.print(",$WLS,");
  Serial.print(myrudder.wls.slope);
  Serial.print(",");
  Serial.print(myrudder.wls.intercept);
  Serial.print(",$LINE,");
  Serial.print(myrudder.alongtrack);
  Serial.print(",");
  Serial.print(myrudder.crosstrack);
  Serial.print(",$WAYA,");
  Serial.print(myrudder.waypointA.latitude,8);
  Serial.print(",");
  Serial.print(myrudder.waypointA.longitude,8);
  Serial.print(",$WAYB,");
  Serial.print(myrudder.waypointB.latitude,8);
  Serial.print(",");
  Serial.print(myrudder.waypointB.longitude,8);
  Serial.print(",$TAR,");
  Serial.print(myrudder.targetbearing);
  Serial.print(",$RUD,");
  Serial.print(rudder);
  Serial.print(",");
  Serial.println(myrudder.sensitivity);
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) 
  {
    dataFile.print("b1,$TMR,");
    dataFile.print(mytime);
    dataFile.print(",$RC,");
    dataFile.print(ch1);
    dataFile.print(",");
    dataFile.print(ch2);  
    dataFile.print(",");
    dataFile.print(ch3);  
    dataFile.print(",$CMP,");
    dataFile.print(bearing);
    dataFile.print(",");
    dataFile.print(pitch); 
    dataFile.print(",");
    dataFile.print(roll);
    dataFile.print(",$ACC,");
    dataFile.print(compass1.getAcceleroX() * compass1._accelScale,4);
    dataFile.print(",");
    dataFile.print(compass1.getAcceleroY() * compass1._accelScale,4);
    dataFile.print(",");
    dataFile.print(compass1.getAcceleroZ() * compass1._accelScale,4);
    dataFile.print(",$GPS,");
    dataFile.print(gps.gpsdate,0);
    dataFile.print(",");
    dataFile.print(gps.gpstime,0);
    dataFile.print(",");
    dataFile.print(gps.latitude,8);
    dataFile.print(",");
    dataFile.print(gps.latNS);
    dataFile.print(",");
    dataFile.print(gps.longitude,8);
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
    dataFile.print(myrudder.crosstrack);
    dataFile.print(",$WAYA,");
    dataFile.print(myrudder.waypointA.latitude,8);
    dataFile.print(",");
    dataFile.print(myrudder.waypointA.longitude,8);
    dataFile.print(",$WAYB,");
    dataFile.print(myrudder.waypointB.latitude,8);
    dataFile.print(",");
    dataFile.print(myrudder.waypointB.longitude,8);
    dataFile.print(",$TAR,");
    dataFile.print(myrudder.targetbearing);
    dataFile.print(",$RUD,");
    dataFile.print(rudder);
    dataFile.print(",");
    dataFile.println(myrudder.sensitivity);
  
    dataFile.close();
  }   
}


