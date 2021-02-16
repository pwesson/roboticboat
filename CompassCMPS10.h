#ifndef CompassCMPS10_h
  #define CompassCMPS10_h

  #include "Arduino.h"
  
  #define _i2cAddress 0x60

  // CMPS10 compass registers
  
  #define BEARING_Register 2 
  #define PITCH_Register 4 
  #define ROLL_Register 5

  #define ACCELEROX_Register 16
  #define ACCELEROY_Register 18
  #define ACCELEROZ_Register 20

  #define CONTROL_Register 22

  #define ONE_BYTE   1
  #define TWO_BYTES  2
  #define FOUR_BYTES 4
  #define SIX_BYTES  6

  class CompassCMPS10{
	public:
	
	  void begin();
	  void ReadCompass();
	  void changeAddress(byte, byte);
		
	  int16_t getBearing();
	  char getPitch();
	  char getRoll();
	  void ReadAccelerator();
		
	  int16_t getAcceleroX();
	  int16_t getAcceleroY();
	  int16_t getAcceleroZ();

	  int bearing;
          int nReceived;
	  signed char pitch;
	  signed char roll;

	  float accelX = 0;
	  float accelY = 0;
	  float accelZ = 0;
	  float accelScale = 9.80592991914f * 2.0f/32767.5f; // setting the accel scale to 2G

	private:
	  byte _byteHigh;
	  byte _byteLow;
	  byte _fine;

  };

#endif