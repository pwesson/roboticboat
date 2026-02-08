// Least Square functions
// Copyright (C) 2026
// 9e13f12b-f5e2-46a5-b0f6-5d28f8ba977b
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


#ifndef WEIGHTED_LEAST_SQUARES_H
#define WEIGHTED_LEAST_SQUARES_H

//--------------------------------------------------------------
// WeightedLeastSquares
//
// Performs an exponentially-discounted weighted linear regression.
// Useful for:
//   - real‑time calibration
//   - smoothing noisy sensor relationships
//   - modelling linear behaviour that slowly drifts over time
//
// Supports:
//   - dynamic regression (default)
//   - fixed/static regression (SetRegression)
//   - predicting Y from X
//   - predicting X from Y (with optional clamping)
//--------------------------------------------------------------

class WeightedLeastSquares {

public:
    WeightedLeastSquares();

    // Reset all internal accumulators and parameters
    void Reset();

    // Fix the regression to a static slope/intercept (disables learning)
    void SetRegression(float setslope, float setintercept);

    // Set exponential discount factor for old samples
    void SetDiscount(float d);

    // Predict Y from X using current regression
    float PredictY(float x);

    // Predict X from Y using current regression
    float PredictX(float y);

    // Predict X from Y, clamped to [miny, maxy]
    float PredictXlimit(float y, float miny, float maxy);

    // Add a new (x,y) sample to the regression
    void AddReading(float x, float y);

private:
    // Regression parameters
    float slope = 0;
    float intercept = 0;

    // Exponential discount factor (0–1)
    float discount = 0.98;

    // Weight of each new sample (constant)
    float w = 1;

    // Whether regression is fixed/static
    bool staticRegression = false;

    // Accumulators for discounted weighted regression
    float sum_w = 0;
    float sum_wx = 0;
    float sum_wy = 0;
    float sum_wxx = 0;
    float sum_wxy = 0;

    // Temporary determinant
    float determinate = 0;
};

#endif