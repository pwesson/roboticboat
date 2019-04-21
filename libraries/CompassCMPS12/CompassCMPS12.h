#ifndef CompassCMPS12_h
  #define CompassCMPS12_h

  #if ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif

  //Address of the CMPS12 compass on i2C
  #define _i2cAddress 0x60

  // CMPS12 compass registers
  #define BEARING_Register 2 
  #define PITCH_Register 4 
  #define ROLL_Register 5

  #define MAGNET_X_Register  6
  #define MAGNET_Y_Register  8
  #define MAGNET_Z_Register 10

  #define ACCELERO_X_Register 12
  #define ACCELERO_Y_Register 14
  #define ACCELERO_Z_Register 16

  #define GYRO_X_Register 18
  #define GYRO_Y_Register 20
  #define GYRO_Z_Register 22

  #define TEMP_Register 24

  #define CONTROLLER_Register 0

  #define COMMUNICATION_TIMEOUT 1000

  #define _ONE_BYTE 1
  #define _TWO_BYTES 2

  class CompassCMPS12{
	public:
		
		void begin();
		void changeAddress(byte, byte);
		float getBearing();
		byte getPitch();
		byte getRoll();
		
		int16_t getAcceleroX();
		int16_t getAcceleroY();
		int16_t getAcceleroZ();
                float _accelScale = 2.0f/32767.5f; // the accel scale is 2G

                int16_t getGyroX();
                int16_t getGyroY();
                int16_t getGyroZ();
                float _gyroScale = 245.0f/32767.5f;

		int16_t getMagnetX();
		int16_t getMagnetY();
		int16_t getMagnetZ();
		
		float getTemperature();

	private:
		int _bearing;
  		byte _fine;
		byte _byteHigh;
		byte _byteLow;
  		char _pitch;
		char _roll;

  };

#endif