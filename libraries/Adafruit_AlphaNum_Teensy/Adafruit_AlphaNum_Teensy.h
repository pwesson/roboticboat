
#ifndef Adafruit_AlphaNum_Teensy_h
#define Adafruit_AlphaNum_Teensy_h

#include "i2c_t3.h"  // I2C library
 
#define HT16K33_BLINK_CMD 0x80
#define HT16K33_BLINK_DISPLAYON 0x01
#define HT16K33_BLINK_OFF 0

#define HT16K33_CMD_BRIGHTNESS 0xE0

// this is the raw HT16K33 controller
class Adafruit_AlphaNum_Teensy {

 public:
  	Adafruit_AlphaNum_Teensy(void);

  	void begin(uint8_t _addr, uint8_t _setbus);
  	void setBrightness(uint8_t b);
  	void blinkRate(uint8_t b);
  	void writeDisplay(void);
  	void writeDigitAscii(uint8_t n, uint8_t ascii, boolean dot = false);
  	void clear(void);

  	uint16_t displaybuffer[4]; 

  	void init(uint8_t a);

  	uint8_t _address;
  	uint8_t _bus;
  	i2c_pins _pins;
  	i2c_pullup _pullups;

  	// i2c bus frequency
  	const uint32_t _i2cRate = 100000;
  	//const uint32_t _i2cRate = 400000;

 protected:
  	uint8_t i2c_addr;
};


#endif // Adafruit_AlphaNum_Teensy_h

