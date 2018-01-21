#ifndef Pam7qGPS_h
  #define Pam7qGPS_h

  #if ARDUINO >= 100
    #include "Arduino.h"
  #else
    #include "WProgram.h"
  #endif

  // Define byte lengths
  #define _ONE_BYTE 1
  #define _TWO_BYTES 2

  #define MAXLENGTH 120
  #define MAXNUMFIELDS 25
  #define MAXFIELDLENGTH 21
  //15 and 11

  class Pam7qGPS{
	public:
		Pam7qGPS();
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

		char latNS, lonEW;
		char gpsstatus;
		int fixquality;
		int numsatelites;
                
	private:
		int _IsBusy;

  };

#endif