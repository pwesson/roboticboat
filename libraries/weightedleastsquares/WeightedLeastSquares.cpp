// Least Square functions
// Copyright (C) 2020 https://www.roboticboat.uk
// b06279e0-e969-4f5a-b198-7bb059eda801
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


#include "WeightedLeastSquares.h"

WeightedLeastSquares::WeightedLeastSquares(){

   // Initialise
   Reset();

}

void WeightedLeastSquares::Reset(){

   discount= 0.98;

   staticRegression = false;
   w = 1;
   slope = 0;
   intercept = 0;
   sum_w = 0;
   sum_wx = 0;
   sum_wy = 0;
   sum_wxx = 0;
   sum_wxy = 0;
}

void WeightedLeastSquares::SetRegression(float setslope, float setintercept)
{
   // Set the regression coefficient
   slope = setslope;
   intercept = setintercept;
   staticRegression = true;
}

void WeightedLeastSquares::SetDiscount(float d){

   // The discount rate of old observations
   discount = d;
}

float WeightedLeastSquares::PredictY(float x){

   // Predict using the regression line
   return slope * x + intercept;
}

float WeightedLeastSquares::PredictX(float y){

   // Predict using the regression line
   if (slope == 0) return 0;

   return (y - intercept) / slope;
}

float WeightedLeastSquares::PredictXlimit(float y, float miny, float maxy){

   // Predict using the regression line
   if (slope == 0) return 0;

   double predicty = (y - intercept) / slope;

   if (predicty > maxy) return maxy;
   if (predicty < miny) return miny;

   return (y - intercept) / slope;
}

void WeightedLeastSquares::AddReading(float x, float y)
{
   //Is the regression static. We want it fixed

   if (staticRegression) return;

   //First discount all the old observations

   sum_w =   discount * sum_w;
   sum_wx =  discount * sum_wx;
   sum_wy =  discount * sum_wy;
   sum_wxx = discount * sum_wxx;
   sum_wxy = discount * sum_wxy;

   //Now add in the new observation

   sum_w =   sum_w + w;
   sum_wx =  sum_wx + w * x;
   sum_wy =  sum_wy + w * y;
   sum_wxx = sum_wxx + w * x * x;
   sum_wxy = sum_wxy + w * x * y;

   //Determinate of the matrix

   determinate = (sum_w * sum_wxx) - (sum_wx * sum_wx);

   if (determinate != 0)
   {
      //Update the regression intercept
      intercept = (sum_wy * sum_wxx - sum_wx * sum_wxy) / determinate;
   
      //Update the regression slope
      slope = (sum_w * sum_wxy - sum_wx * sum_wy) / determinate;
   }

}
