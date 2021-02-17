// Digital Compass CMPS12
// Copyright (C) 2021 https://www.roboticboat.uk
// 34060d00-1ab6-4283-93e0-0cafaf184a46
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


#include "CompassCMPS12.h"

#include <Wire.h>

// Register Function
// 0        Command register (write) / Software version (read)

// 1        Compass Bearing as a byte, i.e. 0-255 for a full circle
// 2,3      Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees. Register 2 being the high byte

// 4        Pitch angle - signed byte giving angle in degrees from the horizontal plane, Kalman filtered with Gyro
// 5        Roll angle - signed byte giving angle in degrees from the horizontal plane, Kalman filtered with Gyro

// 6,7      Magnetometer X axis raw output, 16 bit signed integer with register 6 being the upper 8 bits
// 8,9      Magnetometer Y axis raw output, 16 bit signed integer with register 8 being the upper 8 bits
// 10,11    Magnetometer Z axis raw output, 16 bit signed integer with register 10 being the upper 8 bits

// 12,13    Accelerometer  X axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
// 14,15    Accelerometer  Y axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
// 16,17    Accelerometer  Z axis raw output, 16 bit signed integer with register 16 being the upper 8 bits

// 18,19    Gyro X axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
// 20,21    Gyro Y axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
// 22,23    Gyro Z axis raw output, 16 bit signed integer with register 22 being the upper 8 bits

// 25       Temperature register

// 30       Calibration register

void CompassCMPS12::begin(){
	Wire.begin();
}

int16_t CompassCMPS12::getBearing()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate full bearing
  bearing = ((_byteHigh<<8) + _byteLow) / 10;
  
  return bearing;
}

byte CompassCMPS12::getPitch()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(PITCH_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}

  // Request 1 byte from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  pitch = Wire.read();

  return pitch;
}

byte CompassCMPS12::getRoll()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ROLL_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 1 byte from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  roll = Wire.read();

  return roll ;
}

int16_t CompassCMPS12::getGyroX()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoX
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS12::getGyroY()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoY
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS12::getGyroZ()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoZ
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS12::getAcceleroX()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS12::getAcceleroY()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS12::getAcceleroZ()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

}

int16_t CompassCMPS12::getMagnetX()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS12::getMagnetY()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS12::getMagnetZ()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

float CompassCMPS12::getTemperature()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(TEMP_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 1 byte from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) return 0;

  // Read the values
  _byteHigh = Wire.read(); 

  // Calculate Temperature
  return (((int16_t)_byteHigh <<8));
}

void CompassCMPS12::ReadCompass()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){bearing = 0; pitch = 0;  roll = 0; return;}
  
  // Request 4 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , FOUR_BYTES);

  // Something went wrong
  if (nReceived != FOUR_BYTES) {bearing = 0; pitch = 0;  roll = 0; return;}
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  bearing = ((_byteHigh<<8) + _byteLow) / 10;

  // Read the values
  pitch = Wire.read();

  // Read the values
  roll = Wire.read();

}

void CompassCMPS12::ReadAccelerator()
{
  // Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){accelX = 0; accelY = 0; accelZ = 0; return;}
  
  // Request 6 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , SIX_BYTES);

  // Something went wrong
  if (nReceived != SIX_BYTES) {accelX = 0; accelY = 0; accelZ = 0; return;}
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelX = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelY = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelZ = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

}

void CompassCMPS12::ReadGyro()
{
  // Begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){gyroX = 0; gyroY = 0; gyroZ = 0; return;}
  
  // Request 6 bytes from CMPS12
  nReceived = Wire.requestFrom(_i2cAddress , SIX_BYTES);

  // Sometime went wrong
  if (nReceived != SIX_BYTES) {accelX = 0; accelY = 0; accelZ = 0; return;}
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  gyroX = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;

    // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  gyroY = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  gyroZ = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;

}

void CompassCMPS12::ReadMagnet()
{
  // Begin communication with CMPS11
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNETX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){magnetX = 0; magnetY = 0; magnetZ = 0; return;}
  
  // Request 6 bytes from CMPS11
  Wire.requestFrom(_i2cAddress , SIX_BYTES);

  // Something went wrong
  if (nReceived != SIX_BYTES) {magnetX = 0; magnetY = 0; magnetZ = 0; return;}
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  magnetX = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * magnetScale;

    // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  magnetY = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * magnetScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  magnetZ = (int16_t)(((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * magnetScale;

}

void CompassCMPS12::changeAddress(byte i2cAddress, byte newi2cAddress)
{
  // Reset the address on the i2c network
  // Ensure that you have only this module connected on the i2c network
  // The 7 bit i2c address must end with a 0. (even numbers please)
  // For example changeAddress(0x60, 0xC4)
  // Notice send 0xC4 if you want the address 0x64

  // Address wanted 0x60, so send 0xC0: 1 long flash, 0 short flashes
  // Address wanted 0x62, so send 0xC2: 1 long flash, 1 short flashes
  // Address wanted 0x64, so send 0xC4: 1 long flash, 2 short flashes
  // Address wanted 0x66, so send 0xC6: 1 long flash, 3 short flashes
  // Address wanted 0x68, so send 0xC8: 1 long flash, 4 short flashes
  // Address wanted 0x6A, so send 0xCA: 1 long flash, 5 short flashes
  // Address wanted 0x6C, so send 0xCC: 1 long flash, 6 short flashes
  // Address wanted 0x6E, so send 0xCE: 1 long flash, 7 short flashes

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(0xA0);
  
  // End the transmission
  Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(0xAA);

  // End the transmission
  Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(0xA5);

  // End the transmission
  Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(newi2cAddress);

}