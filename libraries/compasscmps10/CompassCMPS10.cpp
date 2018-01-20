#include <Wire.h>
#include "CompassCMPS10.h"

// Register Function
// 0        Software version
// 1        Compass Bearing as a byte, i.e. 0-255 for a full circle
// 2,3      Compass Bearing as a word, i.e. 0-3599 for a full circle, representing 0-359.9 degrees.
// 4        Pitch angle - signed byte giving angle in degrees from the horizontal plane
// 5        Roll angle - signed byte giving angle in degrees from the horizontal plane
// 6        Unused
// 7        Unused
// 8        Unused
// 9        Unused
// 10,11    Magnetometer X axis raw output, 16 bit signed integer with register 10 being the upper 8 bits
// 12,13    Magnetometer Y axis raw output, 16 bit signed integer with register 12 being the upper 8 bits
// 14,15    Magnetometer Z axis raw output, 16 bit signed integer with register 14 being the upper 8 bits
// 16,17    Accelerometer  X axis raw output, 16 bit signed integer with register 16 being the upper 8 bits
// 18,19    Accelerometer  Y axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
// 20,21    Accelerometer  Z axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
// 22       Command register

void CompassCMPS10::begin(){
	Wire.begin();
}

float CompassCMPS10::getBearing()
{
  // begin communication with CMPS10
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(_REGISTER_BEARING);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS10
  Wire.requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive
  while(Wire.available() < _TWO_BYTES);

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate full bearing
  _bearing = ((_byteHigh<<8) + _byteLow) / 10;
  
  // Calculate bearing decimal
  //_fine = ((_byteHigh<<8) + _byteLow) % 10; 

  return _bearing;
}

char CompassCMPS10::getPitch()
{
  // begin communication with CMPS10
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(_REGISTER_PITCH);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}

  // Request 1 byte from CMPS10
  Wire.requestFrom(_i2cAddress , _ONE_BYTE);

  // Wait for the bytes to arrive
  while(Wire.available() < _ONE_BYTE);

  // Read the values
  _pitch = Wire.read();

  return _pitch;
}

char CompassCMPS10::getRoll()
{
  // begin communication with CMPS10
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(_REGISTER_ROLL);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 1 byte from CMPS10
  Wire.requestFrom(_i2cAddress , _ONE_BYTE);

  // Wait for the bytes to arrive
  while(Wire.available() < _ONE_BYTE);

  // Read the values
  _roll = Wire.read();

  return _roll ;
}

void CompassCMPS10::readAccel()
{
  // begin communication with CMPS10
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(_Register_ACCEL_X);

  //If false, endTransmission() sends a restart message after transmission. The bus will not be released, 
  //which prevents another master device from transmitting between messages. This allows one master device 
  //to send multiple transmissions while in control. The default value is true.
  Wire.endTransmission(false);
  
  // Request 6 bytes from CMPS10
  Wire.requestFrom(_i2cAddress , _SIX_BYTES);

  // Wait for the bytes to arrive
  while(Wire.available() < _SIX_BYTES);

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();
  _accelX = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();
  _accelY = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();
  _accelZ = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

}

void CompassCMPS10::changeAddress(byte i2cAddress, byte newi2cAddress)
{
  // Reset the address on the i2c network
  // Ensure that you have only this module connected on the i2c network
  // The 7 bit i2c address must end with a 0. (even numbers please)
  // For example changeAddress(0x60, 0x64)

  // Address 0x60, 1 long flash, 0 short flashes
  // Address 0x62, 1 long flash, 1 short flashes
  // Address 0x64, 1 long flash, 2 short flashes
  // Address 0x66, 1 long flash, 3 short flashes
  // Address 0x68, 1 long flash, 4 short flashes
  // Address 0x6A, 1 long flash, 5 short flashes
  // Address 0x6C, 1 long flash, 6 short flashes
  // Address 0x6E, 1 long flash, 7 short flashes

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(_REGISTER_CONTROLLER);
  Wire.write(byte(0xA0));
  
  // End the transmission
  int nackCatcher = Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(_REGISTER_CONTROLLER);
  Wire.write(byte(0xAA));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(_REGISTER_CONTROLLER);
  Wire.write(byte(0xA5));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(_REGISTER_CONTROLLER);
  Wire.write(newi2cAddress);

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return;}

}

