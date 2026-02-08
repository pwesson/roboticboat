// Wind logic
// Copyright (C) 2026
// f6e749ee-02d1-4af0-9c77-381a9296970c
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


#ifndef WIND_CIRCULAR_H
#define WIND_CIRCULAR_H

//--------------------------------------------------------------
// WindCircular
//
// Estimates true wind direction using:
//   - Boat heading (degrees)
//   - Roll angle (degrees)
//
// Method:
//   1. Convert heading into relative angle Δ = heading - windEstimate
//   2. Embed Δ on the unit circle: cosΔ, sinΔ
//   3. Fit roll ≈ a*cosΔ + b*sinΔ + c   (circular regression)
//   4. Extract phase φ = atan2(b, a)
//   5. Adjust wind estimate so φ aligns with expected roll peak
//
// This produces a smooth, continuous wind direction estimate
// without a wind vane.
//--------------------------------------------------------------

class WindCircular {

public:
    WindCircular();

    // Add a new sample (heading in degrees, roll in degrees)
    void Update(float headingDeg, float rollDeg);

    // Reset all accumulators
    void Reset();

    // Get current wind direction estimate (degrees 0–360)
    float GetWindDirection() const { return windDir; }

    // Useful bearings for tacking
    float GetPortTack() const;
    float GetStarboardTack() const;

private:
    // --- Parameters ---
    float discount = 0.98f;     // Exponential decay for old samples
    float expectedPeak = 90.0f; // Roll peaks ~90° off the wind

    // --- Regression accumulators ---
    float sum_w   = 0;
    float sum_wx  = 0;
    float sum_wy  = 0;
    float sum_wxx = 0;
    float sum_wxy = 0;

    // Regression coefficients
    float a = 0;   // cosΔ coefficient
    float b = 0;   // sinΔ coefficient
    float c = 0;   // offset

    // Current wind estimate (degrees)
    float windDir = 0;

    // Helpers
    float wrap360(float x) const;
    float wrap180(float x) const;
};

#endif