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


#include "ParallaxPMB688.h"

ParallaxPMB688::ParallaxPMB688(HardwareSerial *serialPort){

  gpsSerial = serialPort;
}

void ParallaxPMB688::read(char nextChar){

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

bool ParallaxPMB688::CheckSum(char* msg) {

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


void ParallaxPMB688::checkGPGGA() {

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


void ParallaxPMB688::checkGPRMC() {

  // Do we have a GPRMC message?
  if (strstr(gpsfields[0], "$GPRMC")) {

    gpstime = atof(gpsfields[1]);      // 111837.000
    gpsstatus = gpsfields[2][0];       // Status A=active or V=Void.
    latitude = atof(gpsfields[3]);     // 0000.0000
    latNS = gpsfields[4][0];           // N
    longitude = atof(gpsfields[5]);    // 00000.0000
    lonEW = gpsfields[6][0];           // E
                                       // Speed over the ground in knots
                                       // Track angle in degrees True
    gpsdate = atof(gpsfields[9]);      // Date - 11th of October 2015
                                       // Magnetic Variation
                                       // The checksum data, always begins with *

    latitude = DegreeToDecimal(latitude, latNS);
    longitude = DegreeToDecimal(longitude, lonEW);
  }
}


float ParallaxPMB688::DegreeToDecimal(float num, byte sign)
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

void ParallaxPMB688::parseString(char* msg) {

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
      if (n >= 25)
      {
         n = 25-1;
      }
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
int ParallaxPMB688::Hex2Dec(char c) {

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

void ParallaxPMB688::NMEA2Binary()
{
  // Switch to Binary communication mode with the GPS Module
  // Sets it to SiRF Binary Mode
  gpsSerial->println("$PSRF100,0,4800,8,1,0*0F"); 

  // Wait for the GPS to think
  delay(2000);  
}

void ParallaxPMB688::Binary2NMEA()
{
  // Start Sequence
  gpsSerial->write(0xA0);       
  gpsSerial->write(0xA2);
  gpsSerial->write((byte)0x00); // Payload length 
  gpsSerial->write(0x02);       // dec2hex(2 bytes)=0x02

  gpsSerial->write(0x87);       // Message ID 135
  gpsSerial->write(0x02);       // NMEA = 2

  // Message Checksum
  gpsSerial->write((byte)0x00); 
  gpsSerial->write(0x89);       // 0x8F+0x02 (the payload)

  // End Sequence
  gpsSerial->write(0xB0);       
  gpsSerial->write(0xB3);
}

void ParallaxPMB688::SetGPSWalkMode()
{
  //The GPS is by default set up as Static Navigation
  //which means the lat/lon will not change unless moving say 50 meters
  //The script below turns the GPS to pedestrian/walk mode but need to go via 
  //SiRF binary mode to set the parameters. The GPS will of course must have
  //the SiRF chipset.
   
  // Switch from NMEA to SiRF binary - REQUIRES THE SiRF chip!
  // $PSRF100 - Message ID
  // 0 = SiRF binary (1=NMEA)
  // Band = 4800
  // DataBits = 8
  // StopBits = 1
  // Parity = 0
  // Checksum *0C

  NMEA2Binary();

  // Static Navigation (Message ID 143) ------------------
  // Allows the user to enable or disable static navigation to the receiver
  // Static model is meant to be used in cars and thus can be turned off
  gpsSerial->write(0xA0); // Start Sequence
  gpsSerial->write(0xA2);
  gpsSerial->write((byte)0x00); // Payload length 
  gpsSerial->write(0x02); // dec2hex(2 bytes)=0x02

  gpsSerial->write(0x8F); //  1 Message ID = 143
  gpsSerial->write((byte)0x00); //  2 Disable = 0 (Enable = 1)

  gpsSerial->write((byte)0x00); // Message Checksum
  gpsSerial->write(0x8F); // 0x8F+0x00 (the payload)
  
  gpsSerial->write(0xB0); // End Sequence
  gpsSerial->write(0xB3);
   
  delay(1000);

  Binary2NMEA();
}

void ParallaxPMB688::SelectSentences()
{  

  NMEA2Binary();

  // Switch to NMEA Protocol (Message ID 129) ------------
  gpsSerial->write((byte)0xA0); // Start Sequence
  gpsSerial->write((byte)0xA2);
  gpsSerial->write((byte)0x00); // Payload length
  gpsSerial->write((byte)0x18); // dec2hex(24 bytes)=0x18
   
  delay(100);
   
  gpsSerial->write((byte)0x81); // byte  1 Message ID = 129
  gpsSerial->write((byte)0x02); // byte  2 Mode
  gpsSerial->write((byte)0x01); // byte  3 GGA message (ON) 1 second
  gpsSerial->write((byte)0x01); // byte  4 Checksum
  gpsSerial->write((byte)0x00); // byte  5 GLL message (OFF) 0 second
  gpsSerial->write((byte)0x01); // byte  6 Checksum
  gpsSerial->write((byte)0x00); // byte  7 GSA message (OFF) 0 second
  gpsSerial->write((byte)0x01); // byte  8 Checksum
  gpsSerial->write((byte)0x00); // byte  9 GSV message (OFF) 0 second
  gpsSerial->write((byte)0x01); // byte 10 Checksum
  gpsSerial->write((byte)0x01); // byte 11 RMC message (ON) 1 second
  gpsSerial->write((byte)0x01); // byte 12 Checksum

  delay(100);

  gpsSerial->write((byte)0x00); // byte 13 VTG message (OFF) 0 second  
  gpsSerial->write((byte)0x01); // byte 14 Checksum
  gpsSerial->write((byte)0x00); // byte 15 MSS message (OFF) 0 second
  gpsSerial->write((byte)0x01); // byte 16 Checksum
  gpsSerial->write((byte)0x00); // byte 17 EPE message
  gpsSerial->write((byte)0x01); // byte 18 Checksum
  gpsSerial->write((byte)0x00); // byte 19 ZDA message
  gpsSerial->write((byte)0x01); // byte 20 Checksum
  gpsSerial->write((byte)0x00); // byte 21 Unused field
  gpsSerial->write((byte)0x01); // byte 22 Unused field
  gpsSerial->write((byte)0x12); // byte 23 Bit rate dec2hex(4800)=12C0
  gpsSerial->write((byte)0xC0); // byte 24

  delay(100);

  gpsSerial->write((byte)0x01); // Message Checksum
  gpsSerial->write((byte)0x61); // 0x81+0x02+0x01+0x01+0x00+ .... + 0x12+0xC0
  gpsSerial->write((byte)0xB0); // End Sequence
  gpsSerial->write((byte)0xB3);

  //Returns in NMEA format, so no need to request Binary2NMEA
}

void ParallaxPMB688::AllSentences()
{  

  NMEA2Binary();

  // Switch to NMEA Protocol (Message ID 129) ------------
  gpsSerial->write((byte)0xA0); // Start Sequence
  gpsSerial->write((byte)0xA2);
  gpsSerial->write((byte)0x00); // Payload length
  gpsSerial->write((byte)0x18); // dec2hex(24 bytes)=0x18
   
  delay(100);
   
  gpsSerial->write((byte)0x81); // byte  1 Message ID = 129
  gpsSerial->write((byte)0x02); // byte  2 Mode
  gpsSerial->write((byte)0x01); // byte  3 GGA message (ON) 1 second
  gpsSerial->write((byte)0x01); // byte  4 Checksum
  gpsSerial->write((byte)0x01); // byte  5 GLL message (OFF) 0 second
  gpsSerial->write((byte)0x01); // byte  6 Checksum
  gpsSerial->write((byte)0x01); // byte  7 GSA message (OFF) 0 second
  gpsSerial->write((byte)0x01); // byte  8 Checksum
  gpsSerial->write((byte)0x05); // byte  9 GSV message (OFF) 0 second
  gpsSerial->write((byte)0x01); // byte 10 Checksum
  gpsSerial->write((byte)0x01); // byte 11 RMC message (ON) 1 second
  gpsSerial->write((byte)0x01); // byte 12 Checksum

  delay(100);

  gpsSerial->write((byte)0x00); // byte 13 VTG message (OFF) 0 second  
  gpsSerial->write((byte)0x01); // byte 14 Checksum
  gpsSerial->write((byte)0x00); // byte 15 MSS message (OFF) 0 second
  gpsSerial->write((byte)0x01); // byte 16 Checksum
  gpsSerial->write((byte)0x00); // byte 17 EPE message
  gpsSerial->write((byte)0x01); // byte 18 Checksum
  gpsSerial->write((byte)0x00); // byte 19 ZDA message
  gpsSerial->write((byte)0x01); // byte 20 Checksum
  gpsSerial->write((byte)0x00); // byte 21 Unused field
  gpsSerial->write((byte)0x01); // byte 22 Unused field
  gpsSerial->write((byte)0x12); // byte 23 Bit rate dec2hex(4800)=12C0
  gpsSerial->write((byte)0xC0); // byte 24

  delay(100);

  gpsSerial->write((byte)0x01); // Message Checksum
  gpsSerial->write((byte)0x68); // 0x81+0x02+0x01+0x01+0x00+ .... + 0x12+0xC0
  gpsSerial->write((byte)0xB0); // End Sequence
  gpsSerial->write((byte)0xB3);

  //Returns in NMEA format, so no need to request Binary2NMEA
}