#include <Wire.h>
#include "CompassCMPS12.h"

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

float CompassCMPS12::getBearing()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(BEARING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return -1;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate full bearing
  _bearing = ((_byteHigh<<8) + _byteLow) / 10;
  
  // Calculate bearing decimal
  //_fine = ((_byteHigh<<8) + _byteLow) % 10; 

  return _bearing;
}

byte CompassCMPS12::getPitch()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(PITCH_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}

  // Request 1 byte from CMPS12
  Wire.requestFrom(_i2cAddress , _ONE_BYTE);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _ONE_BYTE) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _pitch = Wire.read();

  return _pitch;
}

byte CompassCMPS12::getRoll()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ROLL_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 1 byte from CMPS12
  Wire.requestFrom(_i2cAddress , _ONE_BYTE);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _ONE_BYTE) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _roll = Wire.read();

  return _roll ;
}

int16_t CompassCMPS12::getGyroX()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYRO_X_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);
  
  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoX
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS12::getGyroY()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYRO_Y_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;


  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoY
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS12::getGyroZ()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYRO_Z_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;


  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoZ
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS12::getAcceleroX()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELERO_X_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;


  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS12::getAcceleroY()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELERO_Y_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS12::getAcceleroZ()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELERO_Z_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);
  
  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

}

int16_t CompassCMPS12::getMagnetX()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNET_X_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS12::getMagnetY()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNET_Y_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS12::getMagnetZ()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(MAGNET_Z_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _TWO_BYTES) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

float CompassCMPS12::getTemperature()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS12
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(_Register_TEMP);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS12
  Wire.requestFrom(_i2cAddress , _ONE_BYTE);
  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((Wire.available() < _ONE_BYTE) && (timeout-- >0))
	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 

  // Calculate Temperature
  return (((int16_t)_byteHigh <<8));
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
  Wire.write(CONTROLLER_Register);
  Wire.write(0xA0);
  
  // End the transmission
  Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROLLER_Register);
  Wire.write(0xAA);

  // End the transmission
  Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROLLER_Register);
  Wire.write(0xA5);

  // End the transmission
  Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROLLER_Register);
  Wire.write(newi2cAddress);

}