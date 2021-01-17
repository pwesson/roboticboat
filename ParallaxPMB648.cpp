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


#include "ParallaxPMB648.h"

ParallaxPMB648::ParallaxPMB648(HardwareSerial *serialPort){

  gpsSerial = serialPort;

  // Initialise bytes
  latNS = '.';
  lonEW = '.';
  gpsstatus = 'v';
}

void ParallaxPMB648::listen(){

  while (gpsSerial->available())
  {
     read(gpsSerial->read());
  }
}

void ParallaxPMB648::read(char nextChar){

  // Start of a GPS message
  if (nextChar == '$') {
    
    flag ? redbuffer[ptr] = '\0' : blubuffer[ptr] = '\0';

    ptr = 0;
  }

  // End of a GPS message
  if (nextChar == '\n') {

    if (flag) {
      flag = false;

      // Set termination character of the current buffer
      redbuffer[ptr] = '\0';

      // Process the message if the checksum is correct
      if (CheckSum((char*) redbuffer )) {parseString((char*) redbuffer );}
    }
    else
    {
      flag = true;
      
      // Set termination character of the current buffer
      blubuffer[ptr] = '\0';

      // Process the message if the checksum is correct
      if (CheckSum((char*) blubuffer )) {parseString((char*) blubuffer );}
    }   

    // Reset the pointer
    ptr = 0; 

  }

  // Add a new character
  flag ? redbuffer[ptr] = nextChar : blubuffer[ptr] = nextChar;

  // Check we stay within allocated memory
  if (ptr < 119) ptr++;

}

bool ParallaxPMB648::CheckSum(char* msg) {

  // Check the checksum
  //$GPGGA,.........................0000*6A
  
  // Length of the GPS message
  int len = strlen(msg);

  // Does it contain the checksum, to check
  if (len>3 && msg[len-4] == '*') {

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

float ParallaxPMB648::DegreeToDecimal(float num, byte sign)
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

void ParallaxPMB648::parseString(char* msg) {
 
  messageGGA(msg);
  messageRMC(msg);
}


void ParallaxPMB648::messageGGA(char* msg) 
{
  // $GPGGA,094728.000,5126.4900,N,00016.0200,E,2,08,1.30,19.4,M,47.0,M,0000,0000*52
  // Ensure the checksum is correct before doing this
  // Replace all the commas by end-of-string character '\0'
  // Read the first string
  // Knowing the length of the first string, can jump over to the next string
  // Repeat the process for all the known fields.
  
  // Do we have a GGA message?
  if (!strstr(msg, "GGA")) return;

  // Length of the GPS message
  int len = strlen(msg);

  // Replace all the commas with end character '\0'
  for (int j=0; j<len; j++){
    if (msg[j] == ',' || msg[j] == '*'){
      msg[j] = '\0';
    }
  }

  // Allocate working variables
  int i = 0;

  //$GPGGA

  // GMT time  094728.000
  i += strlen(&msg[i])+1;
  gpstime = atof(&msg[i]);
  
  // Latitude 5126.4900
  i += strlen(&msg[i])+1;
  latitude = atof(&msg[i]);
  
  // North or South (single char)
  i += strlen(&msg[i])+1;
  latNS = msg[i];
  if (latNS == '\0') latNS = '.';
  
  // Longitude 00016.0200
  i += strlen(&msg[i])+1;
  longitude = atof(&msg[i]);
  
  // East or West (single char)
  i += strlen(&msg[i])+1;
  lonEW = msg[i];
  if (lonEW == '\0') lonEW = '.';
  
  // Fix quality (1=GPS)(2=DGPS)
  i += strlen(&msg[i])+1;
  fixquality = atof(&msg[i]);   
      
  // Number of satellites being tracked
  i += strlen(&msg[i])+1;
  numsatelites = atoi(&msg[i]); 
  
  // Horizontal dilution of position
  i += strlen(&msg[i])+1;
  
  // Altitude
  i += strlen(&msg[i])+1;
  altitude = atof(&msg[i]);     
  
  // Height of geoid (mean sea level)
  i += strlen(&msg[i])+1;
  
  // Time in seconds since last DGPS update
  i += strlen(&msg[i])+1;
  
  // DGPS station ID number
  i += strlen(&msg[i])+1;
  
  // Convert from degrees and minutes to degrees in decimals
  latitude = DegreeToDecimal(latitude, latNS);
  longitude = DegreeToDecimal(longitude, lonEW);   
}


void ParallaxPMB648::messageRMC(char* msg) 
{
  // $GPRMC,094728.000,A,5126.4900,N,00016.0200,E,0.01,259.87,310318,,,D*6B
  // Ensure the checksum is correct before doing this
  // Replace all the commas by end-of-string character '\0'
  // Read the first string
  // Knowing the length of the first string, can jump over to the next string
  // Repeat the process for all the known fields.
  
  // Do we have a RMC message?
  if (!strstr(msg, "RMC")) return;

  // Length of the GPS message
  int len = strlen(msg);

  // Replace all the commas with end character '\0'
  for (int j=0; j<len; j++){
    if (msg[j] == ',' || msg[j] == '*'){
      msg[j] = '\0';
    }
  }

  // Allocate working variables
  int i = 0;

  //$GPRMC

  // GMT time  094728.000
  i += strlen(&msg[i])+1;
  gpstime = atof(&msg[i]);

  // Status A=active or V=Void.
  i += strlen(&msg[i])+1;
  gpsstatus = msg[i];

  // Latitude 5126.4900
  i += strlen(&msg[i])+1;
  latitude = atof(&msg[i]);

  // North or South (single char)
  i += strlen(&msg[i])+1;
  latNS = msg[i];
  if (latNS == '\0') latNS = '.';

  // Longitude 00016.0200
  i += strlen(&msg[i])+1;
  longitude = atof(&msg[i]);

  // East or West (single char)
  i += strlen(&msg[i])+1;
  lonEW = msg[i];   
  if (lonEW == '\0') lonEW = '.';          

  // // Speed over the ground in knots
  i += strlen(&msg[i])+1;
  gpsknots = atof(&msg[i]);

  // Track angle in degrees True North
  i += strlen(&msg[i])+1;
  gpstrack = atof(&msg[i]); 
  
  // Date - 31st of March 2018
  i += strlen(&msg[i])+1;
  gpsdate = atof(&msg[i]); 
                     
  // Magnetic Variation
  
  // Convert from degrees and minutes to degrees in decimals
  latitude = DegreeToDecimal(latitude, latNS);
  longitude = DegreeToDecimal(longitude, lonEW);
}


// Convert HEX to DEC
int ParallaxPMB648::Hex2Dec(char c) {

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

void ParallaxPMB648::NMEA2Binary()
{
  // Switch to Binary communication mode with the GPS Module
  // Sets it to SiRF Binary Mode
  gpsSerial->println("$PSRF100,0,4800,8,1,0*0F"); 

  // Wait for the GPS to think
  delay(2000);  
}

void ParallaxPMB648::Binary2NMEA()
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

void ParallaxPMB648::SetGPSWalkMode()
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

void ParallaxPMB648::SelectSentences()
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

void ParallaxPMB648::AllSentences()
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
  gpsSerial->write((byte)0x01); // byte  5 GLL message (ON) 1 second
  gpsSerial->write((byte)0x01); // byte  6 Checksum
  gpsSerial->write((byte)0x01); // byte  7 GSA message (ON) 1 second
  gpsSerial->write((byte)0x01); // byte  8 Checksum
  gpsSerial->write((byte)0x05); // byte  9 GSV message (ON) 5 seconds
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