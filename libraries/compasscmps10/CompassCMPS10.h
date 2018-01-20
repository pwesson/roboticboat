#ifndef CompassCMPS10_h
  #define CompassCMPS10_h

  #if ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif  //Address of the CMPS10 compass on i2C
  #define _i2cAddress 0x60

  // CMPS10 compass registers (at least 3 of them)
  #define _REGISTER_BEARING 2 
  #define _REGISTER_PITCH 4 
  #define _REGISTER_ROLL 5
  #define _Register_ACCEL_X 16
  #define _Register_ACCEL_Y 18
  #define _Register_ACCEL_Z 20
  #define _REGISTER_CONTROLLER 22

  #define _ONE_BYTE 1
  #define _TWO_BYTES 2
  #define _SIX_BYTES 6

  class CompassCMPS10{
	public:
		void begin();
		void changeAddress(byte, byte);
		float getBearing();
		char getPitch();
		char getRoll();
                void readAccel();
		int16_t _accelX;
  		int16_t _accelY;
  		int16_t _accelZ;
  		float _accelScale = 2.0f/32767.5f; // setting the accel scale to 2G

	private:
		int _bearing;
  		byte _fine;
		byte _byteHigh;
		byte _byteLow;
  		char _pitch;
		char _roll;

  };

#endif