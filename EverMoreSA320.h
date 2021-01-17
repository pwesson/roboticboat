#ifndef EverMoreSA320_h
  #define EverMoreSA320_h

  #include "Arduino.h"

  class EverMoreSA320{
	public:
		HardwareSerial *gpsSerial;
		EverMoreSA320(HardwareSerial *serialPort);
		void listen();
                void read(char);
                void messageGGA(char*);
		void messageRMC(char*);
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

};

#endif