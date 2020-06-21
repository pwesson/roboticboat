#ifndef ParallaxSIM33EAU_h
  #define ParallaxSIM33EAU_h

  #include "Arduino.h"
  
  class ParallaxSIM33EAU{
	public:
		HardwareSerial *gpsSerial;
		ParallaxSIM33EAU(HardwareSerial *serialPort);
		void read(char);
                void checkGNGGA();
		void checkGNRMC();
                bool CheckSum(char*);
		void AllSentences();
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