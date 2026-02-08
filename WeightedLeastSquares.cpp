// Least Square functions
// Copyright (C) 2026
// 875ffe6a-c0ad-4a07-a75f-01783882d9b8
// Refined in collaboration with Microsoft Copilot
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

//--------------------------------------------------------------
// Constructor
//--------------------------------------------------------------
WeightedLeastSquares::WeightedLeastSquares() {
    Reset();
}

//--------------------------------------------------------------
// Reset()
// Clears all regression accumulators and resets parameters.
//--------------------------------------------------------------
void WeightedLeastSquares::Reset() {
    discount = 0.98;
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

//--------------------------------------------------------------
// SetRegression()
// Locks the regression to a fixed slope/intercept.
// No further learning occurs until Reset() is called.
//--------------------------------------------------------------
void WeightedLeastSquares::SetRegression(float setslope, float setintercept) {
    slope = setslope;
    intercept = setintercept;
    staticRegression = true;
}

//--------------------------------------------------------------
// SetDiscount()
// Controls how quickly old samples fade.
//   1.0 = no fading
//   0.0 = forget everything immediately
//--------------------------------------------------------------
void WeightedLeastSquares::SetDiscount(float d) {
    discount = d;
}

//--------------------------------------------------------------
// PredictY()
// Returns Y = mX + c
//--------------------------------------------------------------
float WeightedLeastSquares::PredictY(float x) {
    return slope * x + intercept;
}

//--------------------------------------------------------------
// PredictX()
// Returns X = (Y - c) / m
//--------------------------------------------------------------
float WeightedLeastSquares::PredictX(float y) {
    if (slope == 0) return 0;   // Avoid divide-by-zero
    return (y - intercept) / slope;
}

//--------------------------------------------------------------
// PredictXlimit()
// Same as PredictX(), but clamps output to [miny, maxy].
// Useful for servo limits, etc.
//--------------------------------------------------------------
float WeightedLeastSquares::PredictXlimit(float y, float miny, float maxy) {
    if (slope == 0) return 0;

    float pred = (y - intercept) / slope;

    if (pred > maxy) return maxy;
    if (pred < miny) return miny;

    return pred;
}

//--------------------------------------------------------------
// AddReading()
// Adds a new (x,y) sample to the regression, applying
// exponential discounting to older samples.
//--------------------------------------------------------------
void WeightedLeastSquares::AddReading(float x, float y)
{
    // If regression is static, ignore new data
    if (staticRegression) return;

    // Discount all previous accumulated values
    sum_w   *= discount;
    sum_wx  *= discount;
    sum_wy  *= discount;
    sum_wxx *= discount;
    sum_wxy *= discount;

    // Add new weighted sample
    sum_w   += w;
    sum_wx  += w * x;
    sum_wy  += w * y;
    sum_wxx += w * x * x;
    sum_wxy += w * x * y;

    // Compute determinant of normal equation matrix
    determinate = (sum_w * sum_wxx) - (sum_wx * sum_wx);

    if (determinate != 0) {
        // Update intercept
        intercept = (sum_wy * sum_wxx - sum_wx * sum_wxy) / determinate;

        // Update slope
        slope = (sum_w * sum_wxy - sum_wx * sum_wy) / determinate;
    }
}