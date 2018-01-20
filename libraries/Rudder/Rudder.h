#include <Navigation.h>
#include <WeightedLeastSquares.h>

#ifndef Rudder_h
  #define Rudder_h

  class Rudder{
	public:
		Rudder();

		void NewLine(double, double, double);
		void NewLine(double, double, double, double);
		void ClearLine();
		void AddReading(unsigned long, int, float);
		void GyroUpdateMean(float, float);
		void AddGyroReading(int, float);
		void SetGyroRegressionStatic(float, float);
		void SetSensitivity(int);
		
		int Update(double, double, float);
                
		Navigation nav;
		WeightedLeastSquares wls;
		WeightedLeastSquares gyro;

		int minposition;
		int maxposition;

		coordinate waypointX;
		coordinate waypointA;
		coordinate waypointB;
		coordinate waypointD;
		coordinate waypointE;
		coordinate waypointF;

		float reversebearing = 0;
		float alongtrack = 0;
		float alongtrackobjective = 0;
		float crosstrack = 0;
		float localdistance = 10;
		
		float bearing = 0;
		float oldbearing = 0;
		float bearingchanged = 0;
		float requiredBearingChange = 0;
		float targetbearing = 0;

		float rudderma = 0;
		float lambda = 0.1;
		float sensitivity = 2;
		
                int gyroNbearing = 0;
		float gyrolambda = 0.1;
		float gyrooldbearing = 0;
		float gyrozmean = 0;

		unsigned long atime = 0;
		unsigned long btime = 0;
	        
	private:
		        		
  };

#endif