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


#include "ParallaxSIM33EAU.h"

ParallaxSIM33EAU::ParallaxSIM33EAU(HardwareSerial *serialPort){

  gpsSerial = serialPort;

}

void ParallaxSIM33EAU::read(char nextChar){

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
        checkGNGGA();
        checkGNRMC();
      }
    }
    else
    {
      flag = true;
      blubuffer[ptr] = '\0';
      if (CheckSum((char*) blubuffer )) {
        parseString((char*) blubuffer );
        checkGNGGA();
        checkGNRMC();     
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


bool ParallaxSIM33EAU::CheckSum(char* msg) {

  // Check the checksum
  //$GNGGA,.........................0000*6A

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


void ParallaxSIM33EAU::checkGNGGA() {

// $GNGGA,130048.000,0000.0000,N,00000.0000,E,1,10,1.1,75.5,M,47.0,M,,0000*00

  // Do we have a GGA message?
  if (strstr(gpsfields[0], "GGA")) {

    gpstime = atol(gpsfields[1]);      // 130048.000
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


void ParallaxSIM33EAU::checkGNRMC() {

//$GNRMC,111837.000,A,0000.0000,N,00000.0000,E,0.44,147.21,111015,,,A*6A

  // Do we have a RMC message?
  if (strstr(gpsfields[0], "RMC")) {

    gpstime = atol(gpsfields[1]);      // 111837.000
    gpsstatus = gpsfields[2][0];       // Status A=active or V=Void.
    latitude = atof(gpsfields[3]);     // 0000.0000
    latNS = gpsfields[4][0];           // N
    longitude = atof(gpsfields[5]);    // 00000.0000
    lonEW = gpsfields[6][0];           // E
    gpsknots = atof(gpsfields[7]);     // Speed over the ground in knots
    gpstrack = atof(gpsfields[8]);     // Track angle in degrees True
    gpsdate = atof(gpsfields[9]);      // Date - 11th of October 2015
                                       // Magnetic Variation
                                       // The checksum data, always begins with *

    latitude = DegreeToDecimal(latitude, latNS);
    longitude = DegreeToDecimal(longitude, lonEW);
  }
}


float ParallaxSIM33EAU::DegreeToDecimal(float num, byte sign)
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


void ParallaxSIM33EAU::parseString(char* msg) {

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
      if (n == 25){n--;}
      j=0;
    }
    else {
      gpsfields[n][j] = msg[i];
      j++;
      gpsfields[n][j] = '\0';
    }
  }

}


// Convert HEX to DEC
int ParallaxSIM33EAU::Hex2Dec(char c) {

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


void ParallaxSIM33EAU::AllSentences()
{
  // Turn-off Static mode
  // PMTK_API_SET_STATIC_NAV_THD
  // Command in the MT3337 Platform NMEA Message Specification_V1.00
  gpsSerial->println("$PMTK386,0*23");
  delay(100);

  // Select output sentences
  // PMTK_API_SET_MNEA_OUTPUT
  // Supported NMEA Sentences: 
  //    0 NMEA_SEN_GLL,  // GPGLL interval - Geographic Latitude longitude  
  //    1 NMEA_SEN_RMC,  // GPRMC interval - Recomended Minimum Specific  
  //    2 NMEA_SEN_VTG,  // GPVTG interval - Course Over Ground Speed  
  //    3 NMEA_SEN_GGA,  // GPGGA interval - GPS Fix Data  
  //    4 NMEA_SEN_GSA,  // GPGSA interval - GNSS Satellites Active   
  //    5 NMEA_SEN_GSV,  // GPGSV interval - GNSS Satellites in View  
  //    6 NMEA_SEN_GRS,  // GPGRS interval – GNSS Range Residuals 
  //    7 NMEA_SEN_GST,  // GPGST interval – GNSS Pseudorange Errors Statistics 
  //   17 NMEA_SEN_ZDA,  // GPZDA interval – Time & Date  
  //   18 NMEA_SEN_MCHN, // PMTKCHN interval – GNSS channel status 
  //   19 NMEA_SEN_DTM,  // GPDTM interval – Datum reference

  // To work out the checksum I used the spreadsheet below
  // https://www.roboticboat.uk/Excel/NMEAchecksum.xlsx
  
  gpsSerial->println("$PMTK314,1,1,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*30");
  delay(100);
}


void ParallaxSIM33EAU::SelectSentences()
{
  // Turn-off Static mode
  // PMTK_API_SET_STATIC_NAV_THD
  // Command in the MT3337 Platform NMEA Message Specification_V1.00
  gpsSerial->println("$PMTK386,0*23");
  delay(100);

  // Select output sentences
  // PMTK_API_SET_MNEA_OUTPUT
  // Supported NMEA Sentences: 
  //    0 NMEA_SEN_GLL,  // GPGLL interval - Geographic Latitude longitude  
  // *  1 NMEA_SEN_RMC,  // GPRMC interval - Recomended Minimum Specific  
  //    2 NMEA_SEN_VTG,  // GPVTG interval - Course Over Ground Speed  
  // *  3 NMEA_SEN_GGA,  // GPGGA interval - GPS Fix Data  
  //    4 NMEA_SEN_GSA,  // GPGSA interval - GNSS Satellites Active   
  //    5 NMEA_SEN_GSV,  // GPGSV interval - GNSS Satellites in View  
  //    6 NMEA_SEN_GRS,  // GPGRS interval – GNSS Range Residuals 
  //    7 NMEA_SEN_GST,  // GPGST interval – GNSS Pseudorange Errors Statistics 
  //   17 NMEA_SEN_ZDA,  // GPZDA interval – Time & Date  
  //   18 NMEA_SEN_MCHN, // PMTKCHN interval – GNSS channel status 
  //   19 NMEA_SEN_DTM,  // GPDTM interval – Datum reference

  // To work out the checksum I used the spreadsheet below
  // https://www.roboticboat.uk/Excel/NMEAchecksum.xlsx
  
  gpsSerial->println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*34");
  delay(100);
  
}