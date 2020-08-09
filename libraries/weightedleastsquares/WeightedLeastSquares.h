// Least Square functions
// Copyright (C) 2020 https://www.roboticboat.uk
// 4d69f25f-7e80-4e75-8618-fb8a7095fca1
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// These Terms shall be governed and construed in accordance with the laws of 
// England and Wales, without regard to its conflict of law provisions.


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