#include <SD.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

// __        __          _     _                            _     ____     ___  
// \ \      / /   __ _  | |_  | |       ___    __ _   ___  | |_  / ___|   / _ \ .
//  \ \ /\ / /   / _` | | __| | |      / _ \  / _` | / __| | __| \___ \  | | | |
//   \ V  V /   | (_| | | |_  | |___  |  __/ | (_| | \__ \ | |_   ___) | | |_| |
//    \_/\_/     \__, |  \__| |_____|  \___|  \__,_| |___/  \__| |____/   \__\_\.
//               |___/                                                          

#include <WeightedLeastSquares.h>

//   ____   __  __   ____    ____      _   _ 
//  / ___| |  \/  | |  _ \  / ___|    / | / |
// | |     | |\/| | | |_) | \___ \    | | | |
// | |___  | |  | | |  __/   ___) |   | | | |
//  \____| |_|  |_| |_|     |____/    |_| |_|
                                           
#include <CompassCMPS11.h>

//  ____       _      __  __     _____    ___         ____   ____    ____  
// |  _ \     / \    |  \/  |   |___  |  / _ \       / ___| |  _ \  / ___| 
// | |_) |   / _ \   | |\/| |      / /  | | | |     | |  _  | |_) | \___ \ .
// |  __/   / ___ \  | |  | |     / /   | |_| |     | |_| | |  __/   ___) |
// |_|     /_/   \_\ |_|  |_|    /_/     \__\_\      \____| |_|     |____/ 
 
#include <Pam7qGPS.h>

//Is Robot in control of the human
bool IsHuman = true;

//Radio Control Receiver
int ch1 = 0;
int ch2 = 0;

//LED light
bool islighton = false;

// Timer
unsigned long mytime;
unsigned long lasttime;

// Array to store the wind roll
const int n = 36;  //Ensure it is even as it is divided by 2.
int Windyn[n];
int Windsum[n];
int WindDirection = 0;
float tmp = 0;
float lambda = 0.10;
float Wind[n];
float WindSort[n];

//Radio Control Servo
Servo myservo;

// Declare the compass class
CompassCMPS11 compass1;

//A GPS library
Pam7qGPS gps;

// Global variables
String inputSerial1 = "";        // a string to hold incoming data
boolean IsReadySerial1 = false;  // whether the string is complete

// Setup the weighted least squares
WeightedLeastSquares wls;
float bearing = 0;
float oldbearing = 0;
float dbearing = 0;
float targetbearing = 0;
signed char pitch = 0;
signed char roll = 0;

void setup()
{
  // Keep the User informed
  Serial.begin(9600);

  // initialize the digital pin as an output.
  pinMode(13, OUTPUT);

  // Setup radio control input pins
  pinMode(16, INPUT); 
  pinMode(17, INPUT); 

  //Setup servo connection
  myservo.attach(20);  // attaches the servo on pin 20 
     
   Serial.println("Initializing GPS");
  
   Serial1.begin(9600);
  
   // Initialize the compass
   compass1.begin();
  
   //Initialise the SD card
   if (!SD.begin(BUILTIN_SDCARD)) 
   {
     Serial.println("SD failed");
     return;
   }

   // Initialise the Wind array
   for (int i=0; i<n; i++)
   {
    Wind[i] = 0;
   }

   //Set the last time
  lasttime = 0;

  bearing = compass1.getBearing();
  oldbearing = bearing;
}

void loop()
{
  //Timer
  mytime = millis();

  //Has 1 seconds passed yet? 
  //Approx once an hour this will roll over to zero again.
  if (abs(mytime - lasttime) > 1000)
  {
     lasttime = mytime;

     //Ensure the change in degree is between -180 to 180 degrees.
     //This handles the scenarios when bearing goes from 10 to 350 degress, say.
     dbearing = bearing - oldbearing;
     if (dbearing > 180) dbearing = dbearing - 360;
     if (dbearing < -180) dbearing = dbearing + 360;
     if (ch1 >=0 && ch1 <=180) wls.AddReading(ch1, dbearing);
     Serial.print("$ROB,");
     Serial.print(ch1);
     Serial.print(",");
     Serial.print(bearing);
     Serial.print(",");
     Serial.print(oldbearing);
     Serial.print(",");
     Serial.println(dbearing);
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
  
  // Read the pulse width of each channel
  ch1 = pulseIn(16, HIGH, 20000);
  ch2 = pulseIn(17, HIGH, 20000);

  // Map the value to the range 0 to 180 which is what radio control servos like
  ch1 = map(ch1,1000,2000,0,180);
  ch2 = map(ch2,1000,2000,0,180);

  //If the left RC lever is down - turn on robot
  if (ch2 > 150 || ch2 < 0)
  { 
    if (IsHuman) targetbearing = bearing;
    IsHuman = false;
  } 
  else 
  {
    IsHuman = true;
  }

  if (IsHuman)
  {
      myservo.write(ch1);
  }
  else
  {
     dbearing = targetbearing - oldbearing;
     if (dbearing > 180) dbearing = dbearing - 360;
     if (dbearing < -180) dbearing = dbearing + 360;

    //Restrict the rudder position (within Pluto between 0 and 140 position of servo)
    //Halve the degree error by 2, so hopefully get on target in 2 seconds.
    int rudder = (int)wls.PredictXlimit(dbearing/2, 0, 140);

    myservo.write(rudder);
  }
  
  // read the compass
  bearing = compass1.getBearing();
  pitch = compass1.getPitch();
  roll = compass1.getRoll();

  //Want to update the Wind array
  int i = bearing/10; //Will range from 0 to 35
  Wind[i] = lambda * roll + (1-lambda) * Wind[i-1]; 

  WDirection();

  while (Serial1.available()) 
  {
    gps.read((char)Serial1.read());
  }
  
  // Print data to Serial Monitor window
  Serial.print("$TMR,");
  Serial.print(mytime);
  Serial.print(",$RC,");
  Serial.print(ch1);
  Serial.print(",");
  Serial.print(ch2);  
  Serial.print(",$CMP,");
  Serial.print(bearing);
  Serial.print(",");
  Serial.print(pitch); 
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",$ACC,");
  Serial.print(compass1.getAcceleroX() * compass1._accelScale);
  Serial.print(",");
  Serial.print(compass1.getAcceleroY() * compass1._accelScale);
  Serial.print(",");
  Serial.print(compass1.getAcceleroZ() * compass1._accelScale);  
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
  Serial.print(gps.gpsstatus);
  Serial.print(",$WND,");
  Serial.print(WindDirection);
  Serial.print(",$WLS,");
  Serial.print(wls.slope);
  Serial.print(",");
  Serial.print(wls.intercept);
  Serial.print(",");
  Serial.println(targetbearing);
  
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) 
  {
    //Write the data to disk and then close the file
    dataFile.print("$TMR,");
    dataFile.print(mytime);
    dataFile.print(",$RC,");
    dataFile.print(ch1);
    dataFile.print(",");
    dataFile.print(ch2);  
    dataFile.print(",$CMP,");
    dataFile.print(bearing);
    dataFile.print(",");
    dataFile.print(pitch); 
    dataFile.print(",");
    dataFile.print(roll);
    dataFile.print(",$ACC,");
    dataFile.print(compass1.getAcceleroX() * compass1._accelScale);
    dataFile.print(",");
    dataFile.print(compass1.getAcceleroY() * compass1._accelScale);
    dataFile.print(",");
    dataFile.print(compass1.getAcceleroZ() * compass1._accelScale);  
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
    dataFile.print(gps.gpsstatus);
    dataFile.print(",$WND,");
    dataFile.print(WindDirection);
    dataFile.print(",$WLS,"); 
    dataFile.print(wls.slope);
    dataFile.print(",");
    dataFile.print(wls.intercept);
    dataFile.print(",");
    dataFile.println(targetbearing);
    dataFile.close();
  }  
  else 
  {
    Serial.println("error opening datalog.txt");
  } 
}

void WDirection()
{
  //Want to find the median of the Wind array
   
  //Copy the array first
  for (int i=0; i<n; i++)
  {
    WindSort[i] = Wind[i];    
  }

  //Sort into order - looking for the median
  //So could do 1/2 the sort
  for (int i=0; i<n; i++)
  {
    for (int j=i+1; j<n; j++)
    {
      if (WindSort[j] < WindSort[i])
      {
        tmp = WindSort[i];
        WindSort[i] = WindSort[j];
        WindSort[j] = tmp;          
      }
    }
  }

  float median = WindSort[n/2];

  //Now flag the values above the median
  for (int i=0; i<n; i++)
  {
    Windyn[i] = 0;
    if (Wind[i] >= median) Windyn[i] = 1;
  }

  int sum_ones = 0;
  for (int i=0; i<18; i++)
  {
     sum_ones = sum_ones + Windyn[i];
  }

  for (int i=0; i<18; i++)
  {
    //Save the result
     Windsum[i] = sum_ones;

     //From symmetry we also know
     Windsum[18+i] = n/2 - Windsum[i];

     //Update the sum. Add the end one and take off the first
     sum_ones = sum_ones + Windyn[n/2+i] - Windyn[i];
  }

  //What is the max Windsum
  WindDirection=0;
  int maxWindsum = 0;

  for (int i=0; i<n; i++)
  {
     if (Windsum[i] > maxWindsum)
     {
        maxWindsum = Windsum[i];
        WindDirection = i;  
     }
  }

}

