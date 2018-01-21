
#include "i2c_t3.h"  // I2C library

#include "Adafruit_AlphaNum_Teensy.h"

static const uint16_t alphafonttable[] PROGMEM =  {

0b0000000000111111, // 0
0b0000000000000110, // 1
0b0000000011011011, // 2
0b0000000011001111, // 3
0b0000000011100110, // 4
0b0000000011101101, // 5
0b0000000011111101, // 6
0b0000000000000111, // 7
0b0000000011111111, // 8
0b0000000011101111, // 9

};

Adafruit_AlphaNum_Teensy::Adafruit_AlphaNum_Teensy(void) {

   // The constructor
}

void Adafruit_AlphaNum_Teensy::begin(uint8_t _addr = 0x70, uint8_t _setbus = 0) {

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

  i2c_t3(_bus).beginTransmission(i2c_addr);
  i2c_t3(_bus).write(0x21);  // turn on oscillator
  i2c_t3(_bus).endTransmission();

  blinkRate(HT16K33_BLINK_OFF);
  
  // max brightness
  setBrightness(1); 
}

void Adafruit_AlphaNum_Teensy::setBrightness(uint8_t b) {

  // The max brightness is 15
  if (b > 15) b = 15;

  // Communicate to the LED module
  i2c_t3(_bus).beginTransmission(i2c_addr);
  i2c_t3(_bus).write(HT16K33_CMD_BRIGHTNESS | b);
  i2c_t3(_bus).endTransmission();  
}

void Adafruit_AlphaNum_Teensy::blinkRate(uint8_t rate) {

  // Turn off if blink rate is wrong
  if (rate > 3) rate = 0;

  // Communicate to the LED module
  i2c_t3(_bus).beginTransmission(i2c_addr);
  i2c_t3(_bus).write(HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (rate << 1)); 
  i2c_t3(_bus).endTransmission();
}

void Adafruit_AlphaNum_Teensy::writeDisplay(void) {

  i2c_t3(_bus).beginTransmission(i2c_addr);
  i2c_t3(_bus).write((uint8_t)0x00); // start at address $00

  for (uint8_t i=0; i<4; i++) {
    i2c_t3(_bus).write(displaybuffer[i] & 0xFF);    
    i2c_t3(_bus).write(displaybuffer[i] >> 8);    
  }

  i2c_t3(_bus).endTransmission();  
}

void Adafruit_AlphaNum_Teensy::writeDigitAscii(uint8_t n, uint8_t a,  boolean d) {

  uint16_t font = pgm_read_word(alphafonttable+a);

  displaybuffer[n] = font;

  if (d) displaybuffer[n] |= (1<<14);
}

void Adafruit_AlphaNum_Teensy::clear(void) {
  for (uint8_t i=0; i<4; i++) {
    displaybuffer[i] = 0;
  }
}
