// Receiving GPS data from the uBlox NEO-M8N
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

// $GNRMC,133513.00,A,51xx.xxxxx,N,00009.xxxxx,E,0.076,,060620,,,A*6E
// $GNVTG,,T,,M,0.076,N,0.141,K,A*38
// $GNGGA,133513.00,51xx.xxxxx,N,00009.xxxxx,E,1,12,0.86,70.8,M,45.4,M,,*76
// $GNGSA,A,3,07,16,27,08,26,18,,,,,,,1.72,0.86,1.49*1A
// $GNGSA,A,3,78,79,88,87,81,71,72,,,,,,1.72,0.86,1.49*1E
// $GPGSV,4,1,13,07,17,317,29,08,33,284,18,10,34,141,18,11,08,253,*79
// $GPGSV,4,2,13,13,03,025,,15,05,052,,16,69,198,08,18,33,057,18*7F
// $GPGSV,4,3,13,20,47,099,14,21,57,065,,26,37,167,22,27,67,293,39*7C
// $GPGSV,4,4,13,30,02,343,*4E
// $GLGSV,3,1,10,65,20,109,,71,13,013,19,72,28,062,17,77,03,182,*64
// $GLGSV,3,2,10,78,44,217,16,79,48,300,37,80,07,341,17,81,39,269,26*69
// $GLGSV,3,3,10,87,36,058,23,88,72,012,34*63
// $GNGLL,51xx.xxxxx,N,00009.xxxxx,E,133513.00,A,A*74



//The UNO needs the software emulator of the serial port
#include <SoftwareSerial.h>

// Global variables
String inputMessage = "";        // A string to hold incoming data
boolean IsMessageReady = false;  // Whether the string is complete

// gpsSerial(receive from GPS,transmit to the GPS module)
SoftwareSerial gpsSerial(2,3);

void setup()
{
  // Keep the User informed
  Serial.begin(9600);
  Serial.println("Initializing GPS");
  
  //Receive from the GPS device (the NMEA sentences) - Green wire
  pinMode(2, INPUT);   

  //Transmit to the GPS device - Yellow wire
  pinMode(3, OUTPUT);  

  // Connect to the GPS module
  gpsSerial.begin(9600);

  delay(1000);

  AllSentences();
  
  // Reserve 200 bytes for the SoftwareSerial string
  inputMessage.reserve(200);
}

void loop()
{
  while (gpsSerial.available() && IsMessageReady == false) 
  {
     // Read the new byte:
     char nextChar = (char)gpsSerial.read();
     
     // Append to the inputSerial1 string
     inputMessage += nextChar;
     
     // If a newline, then set flag so that the main loop will process the string
     if (nextChar == '\n') {
       IsMessageReady = true;
     }
   }
   
    // Does SoftwareSeria1 have a new message?
   if (IsMessageReady) 
   {
     // Print new message on a new line. 
     // The last character of the SoftwareSerial is a new line
     Serial.print(inputMessage);
     
     // clear the string:
     inputMessage = "";
     IsMessageReady = false;
   }
}

void AllSentences()
{
  // NMEA_GLL output interval - Geographic Position - Latitude longitude
  // NMEA_RMC output interval - Recommended Minimum Specific GNSS Sentence
  // NMEA_VTG output interval - Course Over Ground and Ground Speed
  // NMEA_GGA output interval - GPS Fix Data
  // NMEA_GSA output interval - GNSS DOPS and Active Satellites
  // NMEA_GSV output interval - GNSS Satellites in View

  // Enable $PUBX,40,GLL,0,1,0,0*5D
  gpsSerial.println("$PUBX,40,GLL,0,1,0,0*5D");
  delay(100);

  // Enable $PUBX,40,RMC,0,1,0,0*46
  gpsSerial.println("$PUBX,40,RMC,0,1,0,0*46");
  delay(100);
  
  // Enable $PUBX,40,VTG,0,1,0,0*5F
  gpsSerial.println("$PUBX,40,VTG,0,1,0,0*5F");
  delay(100);

  // Enable $PUBX,40,GGA,0,1,0,0*5B
  gpsSerial.println("$PUBX,40,GGA,0,1,0,0*5B");
  delay(100);
  
  // Enable $PUBX,40,GSA,0,1,0,0*4F
  gpsSerial.println("$PUBX,40,GSA,0,1,0,0*4F");
  delay(100);  

  // Enable $PUBX,40,GSV,0,5,0,0*5C
  gpsSerial.println("$PUBX,40,GSV,0,5,0,0*5C");
  delay(100);
}
