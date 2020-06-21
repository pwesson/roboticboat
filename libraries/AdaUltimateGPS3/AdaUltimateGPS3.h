#ifndef AdaUltimateGPS3_h
  #define AdaUltimateGPS3_h

  #include "Arduino.h"

  class AdaUltimateGPS3{
	public:
		HardwareSerial *gpsSerial;
		AdaUltimateGPS3(HardwareSerial *serialPort);
		void read(char);
                void checkGPGGA();
		void checkGPRMC();
                bool CheckSum(char*);
		void SelectSentences();
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
		volatile int ptr = 0;
		volatile bool flag = true;
		volatile char redbuffer[120];
		volatile char blubuffer[120];
		char gpsfields[25][20];
};

#endif