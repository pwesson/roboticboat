// Digital Compass HMC5883L
// Copyright (C) 2021 https://www.roboticboat.uk
// a9cd6479-e046-449b-8e49-3ee84eb83270
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

#include <Arduino.h> 

// Register Map
//
//	00	Configuration Register A	R/W
//	01	Configuration Register B	R/W
//	02	Mode Register			R/W
//	03	Data Output X MSB Register	Read
//	04	Data Output X LSB Register	Read
//	05	Data Output Z MSB Register	Read (notice Z before Y)
//	06	Data Output Z LSB Register	Read
//	07	Data Output Y MSB Register	Read
//	08	Data Output Y LSB Register 	Read
//	09	Status Register			Read
//	10	Identification Register A	Read
//	11	Identification Register B	Read
//	12	Identification Register C	Read
//
// Configuration Register B
// The configuration register B sets the device gain.
// Choose a lower gain value (higher GN number) when total field strength causes overflow
// in any of the data output registers. The GN number are the left 3 bits of the register
// followed by 5 zeros. Thus shift left << by 5 bits.
//
// GN number, Recommended Sensor Field Range, Gain (LSb/Gauss), Digital Resolution (mG/LSb)
//    0x00      +/- 0.88 Ga     1370            0.73
//    0x01      +/- 1.3  Ga		1090 (default)	0.92
//    0x02      +/- 1.9  Ga		 820			1.22
//    0x03      +/- 2.5  Ga		 660			1.52
//    0x04      +/- 4.0  Ga		 440			2.27
//    0x05      +/- 4.7  Ga		 390			2.56
//    0x06      +/- 5.6  Ga		 330			3.03
//    0x07      +/- 8.1  Ga		 230			4.35
//
// Mode Register
// The mode register is used to select the operating mode of the device.
//    Mode, 		Description
//    0x00 		Continuous performs measurements and places the result in the data register
//    0x01 (default)	Single measurement mode. Mode register returns to idle mode
//    0x10		Puts the device into idle mode
//    0x11		Puts the device into idle mode
//

CompassHMC5883L::CompassHMC5883L(){
	
}

void CompassHMC5883L::Setup()
{
  // Set the device gain and the digital resolution
  Wire.beginTransmission(_i2cAddress);
  Wire.write(_REGISTER_CONFIG_B);
  Wire.write(_GN_130_Gauss);
  Wire.endTransmission();

  //Set the device to continuous readings
  Wire.beginTransmission(_i2cAddress);
  Wire.write(_REGISTER_MODE);
  Wire.write(0x00);
  Wire.endTransmission();
	
}

float CompassHMC5883L::getBearing()
{
  //Begin communication with HMC5883L
  Wire.beginTransmission(_i2cAddress);

  //Tell register you want some data
  Wire.write(_REGISTER_DATA);

  //End the transmission
  int nackCatcher = Wire.endTransmission();
  
  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}

  // Request 6 bytes from HMC5883L
  nReceived = Wire.requestFrom(_i2cAddress, SIX_BYTES);
  
  // Something went wrong
  if (nReceived != SIX_BYTES) return 0;
  
  // Set space to read in six bytes
  uint8_t buffer[SIX_BYTES];

  // Read the values into an array
  buffer[0] = Wire.read();
  buffer[1] = Wire.read();
  buffer[2] = Wire.read();
  buffer[3] = Wire.read();
  buffer[4] = Wire.read();
  buffer[5] = Wire.read();

  _x = ((buffer[0] << 8) + buffer[1]);
  _z = ((buffer[2] << 8) + buffer[3]);
  _y = ((buffer[4] << 8) + buffer[5]);
  
  // Calculate the magnetic field in the x,y and z axis
  float _xf = (float)_x * _GN_130_Resolution;
  float _yf = (float)_y * _GN_130_Resolution;

  // Note this is a very simple derivation of true North
  // The compass has to be completely flat to have a change of being right
  // Having an acceleometer and gyroscrope would improve the accuracy.
  float bearing = RAD_TO_DEG * atan2(_yf, _xf);

  // Bearing to be between 0 and 360
  if(bearing < 0) bearing = bearing + 360;
  if(bearing > 360) bearing = bearing - 360;

  return bearing;
}
