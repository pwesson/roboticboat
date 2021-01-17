// Receiving GPS data from the uBlox NEO-M8N
// Copyright (C) 2017 https://www.roboticboat.uk
// 3b485f21-b4b2-4d1a-918c-493009b8fa42
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


// The UNO needs the software emulator of the serial port
// For this reason it is not recommended to use the UNO when processing GPS signals
#include <SoftwareSerial.h>

#include "uBlox_NEO_M8N_SoftSerial.h"

// gpsSerial(receive from GPS,transmit to the GPS module)
SoftwareSerial gpsSerial(2,3);

uBlox_NEO_M8N_SoftSerial gps(&gpsSerial);

void setup()
{
  // Connect to the computer
  Serial.begin(9600);

  // Keep the User informed
  Serial.println("Initializing GPS");
  
  // Receive from the GPS device (the NMEA sentences) - Green wire
  pinMode(2, INPUT);   

  // Transmit to the GPS device - Yellow wire
  pinMode(3, OUTPUT);  

  // Connect to the GPS module
  gpsSerial.begin(9600);
  
  // Because we have to process the GPS messages, we can make our life
  // easier if we only request the sentences we require.
  gps.SelectSentences();
}

void loop()
{
  // Print out the current latitude and longitude
  Serial.print(gps.latitude,8);
  Serial.print(", ");
  Serial.println(gps.longitude,8);

  // Check the software serial port.
  // The UNO does not have a serialEvent on the software serial so we have to keep checking the port.
  gps.listen();     
}
