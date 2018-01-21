#ifndef AdaGPS_h
  #define AdaGPS_h

  #if ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif

  // Define byte lengths
  #define _ONE_BYTE 1
  #define _TWO_BYTES 2

  #define MAXLENGTH 120
  //If you are getting all NMEA statements, some may have more than 25 fields
  #define MAXNUMFIELDS 25
  #define MAXFIELDLENGTH 11

  class AdaGPS{
	public:
		AdaGPS();
		void read(char);
                void checkGPGGA();
		void checkGPRMC();
                bool CheckSum(char*);
		int Hex2Dec(char);
                void parseString(char*);
		float DegreeToDecimal(float, byte);
               
  		float gpstime;
		float gpsdate;
		float latitude;
                float longitude;
  		float altitude;
		float gpsknots;
                float gpstrack;

                char latNS, lonEW;
		char gpsstatus;
		int fixquality;
		int numsatelites;
                
	private:
	

  };

#endif