#ifndef WeightedLeastSquares_h
  #define WeightedLeastSquares_h

  class WeightedLeastSquares{
	public:
		WeightedLeastSquares();

                void Reset();
		void SetDiscount(float);
		void SetRegression(float, float);
		void AddReading(float, float);

		float PredictY(float);
		float PredictX(float);
                float PredictXlimit(float, float, float);

		bool staticRegression;
		float w;
		float x;
		float y;
		float discount;
                float slope;
		float intercept;
	                
	private:
  		float determinate;
		float sum_w;
		float sum_wx;
		float sum_wy;
                float sum_wxx;
                float sum_wxy;

  };

#endif