#ifndef CompassCMPS11_t3_h
  #define CompassCMPS11_t3_h

  #include "i2c_t3.h"  // I2C library

  //Address of the CMPS11 compass on i2C
  #define _i2cAddress 0x60

  // CMPS11 compass registers
  #define _Register_BEARING 2 
  #define _Register_PITCH 4 
  #define _Register_ROLL 5

  #define _Register_MAGNET_X  6
  #define _Register_MAGNET_Y  8
  #define _Register_MAGNET_Z 10

  #define _Register_ACCELERO_X 12
  #define _Register_ACCELERO_Y 14
  #define _Register_ACCELERO_Z 16

  #define _Register_GYRO_X 18
  #define _Register_GYRO_Y 20
  #define _Register_GYRO_Z 22

  #define _Register_TEMP 24

  #define _Register_CONTROLLER 0

  #define COMMUNICATION_TIMEOUT 1000

  #define _ONE_BYTE 1
  #define _TWO_BYTES 2

  class CompassCMPS11_t3{
	public:
		
		void begin(uint8_t _addr, uint8_t _setbus);
		void changeAddress(byte, byte);
		int16_t getBearing();
		byte getPitch();
		byte getRoll();
		
		int16_t getAcceleroX();
		int16_t getAcceleroY();
		int16_t getAcceleroZ();
                float _accelScale = 2.0f/32767.5f; // the accel scale is 2G

                int16_t getGyroX();
                int16_t getGyroY();
                int16_t getGyroZ();
                float _gyroScale = 245.0f/32767.5f; // LSM9DS0 documentation

		int16_t getMagnetX();
		int16_t getMagnetY();
		int16_t getMagnetZ();
		
		float getTemperature();

  		uint8_t _address;
  		uint8_t _bus;
  		i2c_pins _pins;
  		i2c_pullup _pullups;

		// i2c bus frequency
  		const uint32_t _i2cRate = 100000;
		//const uint32_t _i2cRate = 400000;
 
	protected:
  		uint8_t i2c_addr;

	private:
		int _bearing;
  		byte _fine;
		byte _byteHigh;
		byte _byteLow;
  		char _pitch;
		char _roll;

  };

#endif