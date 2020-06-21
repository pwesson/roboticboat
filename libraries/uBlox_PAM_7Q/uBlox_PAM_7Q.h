#ifndef uBlox_PAM_7Q_h
  #define uBlox_PAM_7Q_h

  #include "Arduino.h"

  class uBlox_PAM_7Q{
	public:
		HardwareSerial *gpsSerial;
		uBlox_PAM_7Q(HardwareSerial *serialPort);
		void read(char);
                void checkGPGGA();
		void checkGPRMC();
                bool CheckSum(char*);
		int Hex2Dec(char);
                void parseString(char*);
		void SelectSentences();
		void AllSentences();
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
		volatile int ptr = 0;
		volatile bool flag = true;
		volatile char redbuffer[120];
		volatile char blubuffer[120];
		char gpsfields[25][20];
  };

#endif