// CircularRegression
// Copyright (C) 2026
// c8edaef9-9099-4304-9237-e76eddaf0efd
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

// -----------------------------------------------------------------------------
// Streaming Circular Regression Filter (slope = 1.0)
// 
// PURPOSE:
// Maintain an exponentially weighted estimate of the intercept b in:
//      y = x + b   (mod 360)
//
// This version processes one sample at a time in O(1) time.
// No arrays, no history, perfect for real-time Teensy applications.
//
// MATH:
// For each new sample, compute:
//      d = (y - x) mod 360
//      r = radians(d)
//
// Maintain exponentially weighted sums:
//      S = alpha * S + sin(r)
//      C = alpha * C + cos(r)
//
// The intercept is always:
//      b = atan2(S, C)    <-- which is in Radians
// -----------------------------------------------------------------------------

#include "CircularRegression.h"
#include <math.h>

// ------------------------------------------------------------
// Static helper: wrap angle to [0, 360)
// ------------------------------------------------------------
float CircularRegression::Wrap360(float a) {
    while (a < 0.0f)     a += 360.0f;
    while (a >= 360.0f)  a -= 360.0f;
    return a;
}

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
CircularRegression::CircularRegression() {
    Reset();
}

// ------------------------------------------------------------
// Reset all internal state
// ------------------------------------------------------------
void CircularRegression::Reset() {
    alpha   = 0.95f;
    sum_sin = 0.0f;
    sum_cos = 0.0f;
}

// ------------------------------------------------------------
// Set exponential decay factor
// ------------------------------------------------------------
void CircularRegression::SetDecayFactor(float a) {
    // Clamp to valid range
    if (a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;
    alpha = a;
}

// ------------------------------------------------------------
// Update with new observation (x, y)
// Returns updated intercept in degrees
// ------------------------------------------------------------
float CircularRegression::Update(float x, float y) {
    // Angular difference
    float diff_deg = Wrap360(y - x);
    float diff_rad = diff_deg * DEG_TO_RAD;

    // EWMA update
    sum_sin = alpha * sum_sin + sinf(diff_rad);
    sum_cos = alpha * sum_cos + cosf(diff_rad);

    return GetIntercept();
}

// ------------------------------------------------------------
// Predict y = x + intercept
// ------------------------------------------------------------
float CircularRegression::PredictY(float x) const {
    return Wrap360(x + GetIntercept());
}

// ------------------------------------------------------------
// Compute current intercept estimate
// ------------------------------------------------------------
float CircularRegression::GetIntercept() const {
    float angle_rad = atan2f(sum_sin, sum_cos);
    float angle_deg = angle_rad * RAD_TO_DEG;
    return Wrap360(angle_deg);
}

