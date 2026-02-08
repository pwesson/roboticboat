// CircularRegression
// Copyright (C) 2026
// ccdd1d4e-502b-45dd-9f59-630314892ef7
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


#ifndef CIRCULAR_REGRESSION_H
#define CIRCULAR_REGRESSION_H

#include <Arduino.h>

class CircularRegression {
public:
    CircularRegression();

    void Reset();
    void SetDecayFactor(float a);

    // Update with new (x, y) pair and return updated intercept (degrees)
    float Update(float x, float y);

    // Predict y = x + intercept
    float PredictY(float x) const;

    // Get current intercept estimate in degrees
    float GetIntercept() const;

private:
    // Wrap angle to [0, 360)
    static float Wrap360(float a);

    float alpha;        // decay factor (0 < alpha < 1)
    float sum_sin;      // EWMA of sin(d)
    float sum_cos;      // EWMA of cos(d)
};

#endif