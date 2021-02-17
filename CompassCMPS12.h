#ifndef CompassCMPS12_h
  #define CompassCMPS12_h

  #include "Arduino.h"
  
  //Address of the CMPS12 compass on i2C
  #define _i2cAddress 0x60

  // CMPS12 compass registers
  #define CONTROL_Register 0

  #define BEARING_Register 2 
  #define PITCH_Register 4 
  #define ROLL_Register 5

  #define MAGNETX_Register  6
  #define MAGNETY_Register  8
  #define MAGNETZ_Register 10

  #define ACCELEROX_Register 12
  #define ACCELEROY_Register 14
  #define ACCELEROZ_Register 16

  #define GYROX_Register 18
  #define GYROY_Register 20
  #define GYROZ_Register 22

  #define TEMP_Register 24

  #define ONE_BYTE   1
  #define TWO_BYTES  2
  #define FOUR_BYTES 4
  #define SIX_BYTES  6

  class CompassCMPS12{
	public:
		
	  void begin();
	  void ReadCompass();
	  void ReadAccelerator();
	  void ReadGyro();
	  void ReadMagnet();
	  void changeAddress(byte, byte);

	  int16_t getBearing();
	  byte getPitch();
	  byte getRoll();
		
	  int16_t getAcceleroX();
	  int16_t getAcceleroY();
	  int16_t getAcceleroZ();
                
	  int16_t getGyroX();
	  int16_t getGyroY();
	  int16_t getGyroZ();

	  int16_t getMagnetX();
	  int16_t getMagnetY();
	  int16_t getMagnetZ();
		
	  float getTemperature();

	  int bearing;
	  int nReceived;
	  signed char pitch;
	  signed char roll;

	  float magnetX = 0;
	  float magnetY = 0;
	  float magnetZ = 0;
	  float magnetScale = 2.0f/32768.0f; // 2 Gauss

  	  float accelX = 0;
  	  float accelY = 0;
  	  float accelZ = 0;
	  float accelScale = 0.01f; // 1 mg = 1 LSB

	  float gyroX = 0;
	  float gyroY = 0;
	  float gyroZ = 0;
	  float gyroScale = 1.0f/16.0f;  // 1 Dps = 16 LSB

	private:
		byte _byteHigh;
		byte _byteLow;
  		byte _fine;

  };

#endif