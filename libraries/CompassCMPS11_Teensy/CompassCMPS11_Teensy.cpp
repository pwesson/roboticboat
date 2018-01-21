
#include "i2c_t3.h"  // I2C library

#include "CompassCMPS11_Teensy.h"

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

// 24,25    Temperature raw output, 16 bit signed integer with register 24 being the upper 8 bits

// 26       Pitch angle - signed byte giving angle in degrees from the horizontal plane (no Kalman filter)
// 27       Roll angle - signed byte giving angle in degrees from the horizontal plance (no Kalman filter)

void CompassCMPS11_Teensy::begin(uint8_t _addr = 0x60, uint8_t _setbus = 0) {

  i2c_addr = _addr;
  _bus = _setbus;

  // setup the I2C pins and pullups based on bus number if not defined by user
  // setting the I2C pins, pullups, and protecting against _bus out of range

  _pullups = I2C_PULLUP_EXT; // default to external pullups

  if(_bus == 3) {
    	_pins = I2C_PINS_56_57;
  }
  else if(_bus == 2) {
	_pins = I2C_PINS_3_4;
  }
  else if(_bus == 1) {
        _pins = I2C_PINS_37_38;
  }
  else{
        _pins = I2C_PINS_18_19;
	_bus = 0;
  }

  // starting the I2C bus
  i2c_t3(_bus).begin(I2C_MASTER, 0, _pins, _pullups, _i2cRate);

}

int16_t CompassCMPS11_Teensy::getBearing()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_BEARING);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return -1;

  // Read the values
  _byteHigh = i2c_t3(_bus).read(); 
  _byteLow = i2c_t3(_bus).read();

  // Calculate full bearing
  _bearing = ((_byteHigh<<8) + _byteLow) / 10;
  
  // Calculate bearing decimal
  //_fine = ((_byteHigh<<8) + _byteLow) % 10; 

  return _bearing;
}

byte CompassCMPS11_Teensy::getPitch()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_PITCH);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}

  // Request 1 byte from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _ONE_BYTE);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _ONE_BYTE) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _pitch = i2c_t3(_bus).read();

  return _pitch;
}

byte CompassCMPS11_Teensy::getRoll()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_ROLL);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 1 byte from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _ONE_BYTE);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _ONE_BYTE) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _roll = i2c_t3(_bus).read();

  return _roll ;
}

int16_t CompassCMPS11_Teensy::getGyroX()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_GYRO_X);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);
  
  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = i2c_t3(_bus).read(); 
  _byteLow = i2c_t3(_bus).read();

  // Calculate GryoX
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS11_Teensy::getGyroY()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_GYRO_Y);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = i2c_t3(_bus).read(); 
  _byteLow = i2c_t3(_bus).read();

  // Calculate GryoY
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS11_Teensy::getGyroZ()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_GYRO_Z);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = i2c_t3(_bus).read(); 
  _byteLow = i2c_t3(_bus).read();

  // Calculate GryoZ
  return ((_byteHigh<<8) + _byteLow);
}

int16_t CompassCMPS11_Teensy::getAcceleroX()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_ACCELERO_X);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS11_Teensy::getAcceleroY()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_ACCELERO_Y);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  //if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = i2c_t3(_bus).read(); 
  _byteLow = i2c_t3(_bus).read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS11_Teensy::getAcceleroZ()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_ACCELERO_Z);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);
  
  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = i2c_t3(_bus).read(); 
  _byteLow = i2c_t3(_bus).read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

}

int16_t CompassCMPS11_Teensy::getMagnetX()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_MAGNET_X);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  //if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS11_Teensy::getMagnetY()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_MAGNET_Y);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  //if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = i2c_t3(_bus).read(); 
  _byteLow = i2c_t3(_bus).read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t CompassCMPS11_Teensy::getMagnetZ()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_MAGNET_Z);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);

  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  //if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = i2c_t3(_bus).read(); 
  _byteLow = i2c_t3(_bus).read();

  // Calculate value
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

float CompassCMPS11_Teensy::getTemperature()
{
  //Setup timeout parameter
  int timeout = COMMUNICATION_TIMEOUT;

  // begin communication with CMPS11
  i2c_t3(_bus).beginTransmission(_i2cAddress);

  // Tell register you want some data
  i2c_t3(_bus).write(_Register_TEMP);

  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return 0;}
	
  // Request 2 bytes from CMPS11
  i2c_t3(_bus).requestFrom(_i2cAddress , _TWO_BYTES);
  
  // Wait for the bytes to arrive.
  // Don't wait forever as this will hang the whole program
  while((i2c_t3(_bus).available() < _TWO_BYTES) && (timeout-- >0))
  	delay(1);

  if (timeout <= 0) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Temperature
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

void CompassCMPS11_Teensy::changeAddress(byte i2cAddress, byte newi2cAddress)
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
  i2c_t3(_bus).beginTransmission(i2cAddress);
  i2c_t3(_bus).write(_Register_CONTROLLER);
  i2c_t3(_bus).write(byte(0xA0));
  
  // End the transmission
  int nackCatcher = i2c_t3(_bus).endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  i2c_t3(_bus).beginTransmission(i2cAddress);
  i2c_t3(_bus).write(_Register_CONTROLLER);
  i2c_t3(_bus).write(byte(0xAA));

  // End the transmission
  nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  i2c_t3(_bus).beginTransmission(i2cAddress);
  i2c_t3(_bus).write(_Register_CONTROLLER);
  i2c_t3(_bus).write(byte(0xA5));

  // End the transmission
  nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  i2c_t3(_bus).beginTransmission(i2cAddress);
  i2c_t3(_bus).write(_Register_CONTROLLER);
  i2c_t3(_bus).write(newi2cAddress);

  // End the transmission
  nackCatcher = i2c_t3(_bus).endTransmission();

  // Return if we have a connection problem	
  if(nackCatcher != 0){return;}

}