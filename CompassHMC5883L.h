#ifndef CompassHMC5883L_h
#define CompassHMC5883L_h

#include <Arduino.h>
#include <Wire.h>

#define _i2cAddress 0x1E

#define _REGISTER_CONFIG_A 0x00
#define _REGISTER_CONFIG_B 0x01
#define _REGISTER_MODE     0x02
#define _REGISTER_DATA     0x03
#define _REGISTER_STATUS   0x09

#define COMMUNICATION_TIMEOUT 1000
#define TIMEOUT_MICROSECONDS 1000

#define _GN_088_Gauss 0x00 << 5
#define _GN_130_Gauss 0x01 << 5
#define _GN_190_Gauss 0x02 << 5
#define _GN_250_Gauss 0x03 << 5
#define _GN_400_Gauss 0x04 << 5
#define _GN_470_Gauss 0x05 << 5
#define _GN_560_Gauss 0x06 << 5
#define _GN_810_Gauss 0x07 << 5

#define _GN_088_Resolution 0.73
#define _GN_130_Resolution 0.92
#define _GN_190_Resolution 1.22
#define _GN_250_Resolution 1.52
#define _GN_400_Resolution 2.27
#define _GN_470_Resolution 2.56
#define _GN_560_Resolution 3.03
#define _GN_810_Resolution 4.35

#define SIX_BYTES 6

class CompassHMC5883L
{
	public:
	  CompassHMC5883L();
	  void Setup();
	  float getBearing();
          int nReceived;
	  
	private:         
	  int16_t _x;
	  int16_t _y;
 	  int16_t _z;

};
#endif