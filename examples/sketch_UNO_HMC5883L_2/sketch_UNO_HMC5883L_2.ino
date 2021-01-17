// Arduino UNO connected to a HMC5883L compass
// Copyright (C) 2021 https://www.roboticboat.uk
// 78414ba9-3ea0-4df9-b08b-54f0481010f9
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

#include "CompassHMC5883L.h"

#include <Wire.h>

CompassHMC5883L compass;
    
void setup(){
  
  // Initialize the serial port to the User
  // Set this up early in the code, so the User sees all messages
  Serial.begin(9600);

  // Start the i2c network
  Wire.begin();

  // Initialise the compass
  compass.Setup();
}

void loop(){
  
  // Read the compass
  float bearing = compass.getBearing();

  // Print data to Serial Monitor window
  Serial.print("$CMP,");
  Serial.println(bearing);

}
