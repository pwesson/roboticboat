#ifndef ParallaxPMB688_h
  #define ParallaxPMB688_h

  #include "Arduino.h"

  class ParallaxPMB688{
	public:
		HardwareSerial *gpsSerial;
		ParallaxPMB688(HardwareSerial *serialPort);
		void listen();
                void read(char);
                void messageGGA(char*);
		void messageRMC(char*);
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