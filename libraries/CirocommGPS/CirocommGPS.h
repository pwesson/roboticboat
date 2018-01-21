#ifndef CirocommGPS_h
  #define CirocommGPS_h

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
  #define MAXFIELDLENGTH 11

  class CirocommGPS{
	public:
		CirocommGPS();
		void read(char);
                void checkGPGGA();
		void checkGPRMC();
                bool CheckSum(char*);
		int Hex2Dec(char);
                void parseString(char*);
		void SetGPSWalkMode();
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