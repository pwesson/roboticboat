#ifndef ParallaxPMB688_h
  #define ParallaxPMB688_h

  #include "Arduino.h"
  
  // Define byte lengths
  #define _ONE_BYTE 1
  #define _TWO_BYTES 2

  #define MAXLENGTH 120
  #define MAXNUMFIELDS 25
  #define MAXFIELDLENGTH 11

  class ParallaxPMB688{
	public:
		HardwareSerial *gpsSerial;
		ParallaxPMB688(HardwareSerial *serialPort);
		void read(char);
                void checkGPGGA();
		void checkGPRMC();
                bool CheckSum(char*);
		int Hex2Dec(char);
                void parseString(char*);
		void SetGPSWalkMode();
		void SelectSentences();
		void AllSentences();
		void Binary2NMEA();
		void NMEA2Binary();
		float DegreeToDecimal(float, byte);
               
  		float gpstime;
		float gpsdate;
		float latitude;
                float longitude;
		float altitude;

                char latNS, lonEW;
		char gpsstatus;
		int fixquality;
		int numsatelites;
		
	private:
		int _IsBusy;

  };

#endif