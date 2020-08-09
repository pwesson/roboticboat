// Licensed under the GNU General Public License v3.0

#ifndef Wind_h
  #define Wind_h

  class Wind{
	public:
		Wind();

		void Update(float, signed char);
		void UpdateDistribution(signed char);
                void UpdateRoll(float, signed char);

		int Direction = 0;
		int accelDirection = 0;
		int portTackBearing = 0;
		int starboardTackBearing = 0;
		int rollPort = 0;
		int rollStarboard = 0;
		int rollCentre = 0;

		// Distribution of boat roll
		char msg = 'X';
		float mean = 0;
		float stdev = 0;
		float discount = 0.9999;	        

	private:
		const int n = 36;  //Ensure it is even as it is divided by 2.
		int tmpi = 0;
		int Windyn[36];
		int Windsum[36];
		int rollWind[61];
		
		float tmp = 0;
		float lambda = 0.50;
		float Wind01[36];
		float WindSort[36];

		float WindAccelerator[36];        		
		float sum_accel = 0;
                float min_accel = 0;
                
		// Working variables for distribution of boat roll
		float sum_w = 0;
		float sum_wx = 0;
		float sum_wxx = 0;
		float variance = 0;
		
  };

#endif