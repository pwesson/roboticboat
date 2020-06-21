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


#include "uBlox_PAM_7Q.h"

uBlox_PAM_7Q::uBlox_PAM_7Q(HardwareSerial *serialPort){

  gpsSerial = serialPort;
}

void uBlox_PAM_7Q::read(char nextChar){

  // Start of a GPS message
  if (nextChar == '$') {
    if (flag) {
      redbuffer[ptr] = '\0';
    }
    else {
      blubuffer[ptr] = '\0';
    }
    ptr = 0;
  }

  // End of a GPS message
  if (nextChar == '\n') {

    if (flag) {
      flag = false;
      redbuffer[ptr] = '\0';
      if (CheckSum((char*) redbuffer )) {
        parseString((char*) redbuffer );
        checkGPGGA();
        checkGPRMC();
      }
    }
    else
    {
      flag = true;
      blubuffer[ptr] = '\0';
      if (CheckSum((char*) blubuffer )) {
        parseString((char*) blubuffer );
        checkGPGGA();
        checkGPRMC();     
      }
    }   
    ptr = 0; 
  }

  // Add a new character
  if (flag) {
    redbuffer[ptr] = nextChar;
  }
  else {
    blubuffer[ptr] = nextChar;
  }

  ptr++;
  if (ptr >= 120) {
    ptr = 120-1;
  }
}

bool uBlox_PAM_7Q::CheckSum(char* msg) {

  // Check the checksum
  //$GPGGA,.........................0000*6A
  
  // Length of the GPS message
  int len = strlen(msg);

  // Does it contain the checksum, to check
  if (msg[len-4] == '*') {

	// Read the checksum from the message
	int cksum = 16 * Hex2Dec(msg[len-3]) + Hex2Dec(msg[len-2]);

	// Loop over message characters
	for (int i=1; i < len-4; i++) {
      	  cksum ^= msg[i];
    	}

	// The final result should be zero
	if (cksum == 0){
	  return true;
	}
  }

  return false;
}


void uBlox_PAM_7Q::checkGPGGA() {

  // Do we have a GPGGA message?
  if (strstr(gpsfields[0], "$GPGGA")) {

    gpstime = atof(gpsfields[1]);      // 130048.000
    latitude = atof(gpsfields[2]);     // 0000.0000
    latNS = gpsfields[3][0];           // N
    longitude = atof(gpsfields[4]);    // 00000.0000
    lonEW = gpsfields[5][0];           // E
    fixquality = atof(gpsfields[6]);   // Fix quality (1=GPS)(2=DGPS)
    numsatelites = atoi(gpsfields[7]); // Number of satellites being tracked
                                       // Horizontal dilution of position
    altitude = atof(gpsfields[9]);     // Altitude above mean sea level, Meters 
                                       // Height of geoid (mean sea level)
                                       // Time in seconds since last DGPS update
                                       // DGPS station ID number
                                       // the checksum data, always begins with *

    latitude = DegreeToDecimal(latitude, latNS);
    longitude = DegreeToDecimal(longitude, lonEW);
  }
}


void uBlox_PAM_7Q::checkGPRMC() {

  // Do we have a GPRMC message?
  if (strstr(gpsfields[0], "$GPRMC")) {

    gpstime = atof(gpsfields[1]);   // 111837.000
    gpsstatus = gpsfields[2][0];    // Status A=active or V=Void.
    latitude = atof(gpsfields[3]);  // 0000.0000
    latNS = gpsfields[4][0];        // N
    longitude = atof(gpsfields[5]); // 00000.0000
    lonEW = gpsfields[6][0];        // E
                                    // Speed over the ground in knots
                                    // Track angle in degrees True
    gpsdate = atof(gpsfields[9]);   // Date - 11th of October 2015
                                    // Magnetic Variation
                                    // The checksum data, always begins with *

    latitude = DegreeToDecimal(latitude, latNS);
    longitude = DegreeToDecimal(longitude, lonEW);
  }
}


float uBlox_PAM_7Q::DegreeToDecimal(float num, byte sign)
{
   // Want to convert DDMM.MMMM to a decimal number DD.DDDDD

   int intpart= (int) num;
   float decpart = num - intpart;

   int degree = (int)(intpart / 100);
   int mins = (int)(intpart % 100);

   if (sign == 'N' || sign == 'E')
   {
     // Return positive degree
     return (degree + (mins + decpart)/60);
   }   
   // Return negative degree
   return -(degree + (mins + decpart)/60);
}

void uBlox_PAM_7Q::parseString(char* msg) {

  // Length of the GPS message
  int len = strlen(msg);
  int n=0;
  int j=0;

  // Loop over the string
  for (int i=0; i<len; i++) {

    if(msg[i] == ',' || msg[i] == '*') {
      if (j == 0) {
        gpsfields[n][0] = '_';
        gpsfields[n][1] = '\0';
      }
      n++;
      j=0;
      
      if (n >= 25)
      {
      	n = 25-1;
      }
    }
    else {
      gpsfields[n][j] = msg[i];
      j++;
      gpsfields[n][j] = '\0';
    }
  }
}


// Convert HEX to DEC
int uBlox_PAM_7Q::Hex2Dec(char c) {

  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  else if (c >= 'A' && c <= 'F') {
    return (c - 'A') + 10;
  }
  else {
    return 0;
  }
}


void uBlox_PAM_7Q::AllSentences()
{
  // NMEA_GLL output interval - Geographic Position - Latitude longitude
  // NMEA_RMC output interval - Recommended Minimum Specific GNSS Sentence
  // NMEA_VTG output interval - Course Over Ground and Ground Speed
  // NMEA_GGA output interval - GPS Fix Data
  // NMEA_GSA output interval - GNSS DOPS and Active Satellites
  // NMEA_GSV output interval - GNSS Satellites in View

  // Enable $PUBX,40,GLL,0,1,0,0*5D
  gpsSerial->println("$PUBX,40,GLL,0,1,0,0*5D");
  delay(100);

  // Enable $PUBX,40,RMC,0,1,0,0*46
  gpsSerial->println("$PUBX,40,RMC,0,1,0,0*46");
  delay(100);
  
  // Enable $PUBX,40,VTG,0,1,0,0*5F
  gpsSerial->println("$PUBX,40,VTG,0,1,0,0*5F");
  delay(100);

  // Enable $PUBX,40,GGA,0,1,0,0*5B
  gpsSerial->println("$PUBX,40,GGA,0,1,0,0*5B");
  delay(100);
  
  // Enable $PUBX,40,GSA,0,1,0,0*4F
  gpsSerial->println("$PUBX,40,GSA,0,1,0,0*4F");
  delay(100);  

  // Enable $PUBX,40,GSV,0,5,0,0*5C
  gpsSerial->println("$PUBX,40,GSV,0,5,0,0*5C");
  delay(100);
}


void uBlox_PAM_7Q::SelectSentences()
{
  // NMEA_GLL output interval - Geographic Position - Latitude longitude
  // NMEA_RMC output interval - Recommended Minimum Specific GNSS Sentence
  // NMEA_VTG output interval - Course Over Ground and Ground Speed
  // NMEA_GGA output interval - GPS Fix Data
  // NMEA_GSA output interval - GNSS DOPS and Active Satellites
  // NMEA_GSV output interval - GNSS Satellites in View

  // disable $PUBX,40,GLL,0,0,0,0*5C
  gpsSerial->println("$PUBX,40,GLL,0,0,0,0*5C");
  delay(100);

  // Enable $PUBX,40,RMC,0,1,0,0*46
  gpsSerial->println("$PUBX,40,RMC,0,1,0,0*46");
  delay(100);
  
  // disable $PUBX,40,VTG,0,0,0,0*5E
  gpsSerial->println("$PUBX,40,VTG,0,0,0,0*5E");
  delay(100);

  // Enable $PUBX,40,GGA,0,1,0,0*5B
  gpsSerial->println("$PUBX,40,GGA,0,1,0,0*5B");
  delay(100);
  
  // disable $PUBX,40,GSA,0,0,0,0*4E
  gpsSerial->println("$PUBX,40,GSA,0,0,0,0*4E");
  delay(100);  

  // disable $PUBX,40,GSV,0,0,0,0*59
  gpsSerial->println("$PUBX,40,GSV,0,0,0,0*59");
  delay(100);
  
}