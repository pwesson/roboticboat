#ifndef Wind_h
  #define Wind_h

  class Wind{
	public:
		Wind();

		void Update(float, signed char);
                
		int Direction = 0;
	        
	private:
		const int n = 36;  //Ensure it is even as it is divided by 2.
		int Windyn[36];
		int Windsum[36];
		
		float tmp = 0;
		float lambda = 0.50;
		float Wind01[36];
		float WindSort[36];        		
  };

#endif