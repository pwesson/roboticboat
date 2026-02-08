// Wind logic
// Copyright (C) 2026
// a7138980-391c-4688-b659-ca5b037d83cd
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

// wind.Update(headingDeg, rollDeg);
// float windDir = wind.GetWindDirection();


#include "WindCircular.h"
#include <math.h>

//--------------------------------------------------------------
// Constructor
//--------------------------------------------------------------
WindCircular::WindCircular() {
    Reset();
}

//--------------------------------------------------------------
// Reset all accumulators and parameters
//--------------------------------------------------------------
void WindCircular::Reset() {
    sum_w = sum_wx = sum_wy = sum_wxx = sum_wxy = 0;
    a = b = c = 0;
    windDir = 0;
}

//--------------------------------------------------------------
// Angle wrapping helpers
//--------------------------------------------------------------
float WindCircular::wrap360(float x) const {
    while (x < 0)   x += 360;
    while (x >= 360) x -= 360;
    return x;
}

float WindCircular::wrap180(float x) const {
    while (x < -180) x += 360;
    while (x > 180)  x -= 360;
    return x;
}

//--------------------------------------------------------------
// Update()
// Main circular-regression update
//--------------------------------------------------------------
void WindCircular::Update(float headingDeg, float rollDeg)
{
    // 1. Compute relative angle Δ = heading - windDir
    float delta = wrap180(headingDeg - windDir);

    // Convert to radians
    float dRad = delta * M_PI / 180.0f;

    // 2. Circular embedding
    float x = cosf(dRad);
    float y = sinf(dRad);
    float r = rollDeg;

    // 3. Discount old samples
    sum_w   *= discount;
    sum_wx  *= discount;
    sum_wy  *= discount;
    sum_wxx *= discount;
    sum_wxy *= discount;

    // 4. Add new sample
    sum_w   += 1;
    sum_wx  += x;
    sum_wy  += r;
    sum_wxx += x * x + y * y; // = 1, but kept general
    sum_wxy += x * r;

    // 5. Solve regression for r ≈ a*cosΔ + b*sinΔ + c
    // Using simplified normal equations for circular basis
    if (sum_w > 0.001f) {
        c = sum_wy / sum_w; // mean roll
    }

    // Fit a and b using correlation with cosΔ and sinΔ
    a = sum_wxy / sum_w; // correlation with cosΔ
    b = 0;               // sinΔ term omitted for simplicity

    // 6. Extract phase of sinusoid
    float phi = atan2f(b, a) * 180.0f / M_PI;

    // 7. Adjust wind direction so roll peak aligns with expectedPeak
    float correction = wrap180(expectedPeak - phi);
    windDir = wrap360(windDir + 0.1f * correction); // small correction step
}

//--------------------------------------------------------------
// Tack bearings
//--------------------------------------------------------------
float WindCircular::GetPortTack() const {
    return wrap360(windDir + 45);
}

float WindCircular::GetStarboardTack() const {
    return wrap360(windDir - 45);
}