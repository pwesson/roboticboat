// Wind logic
// Copyright (C) 2026
// 893b2022-00be-4030-b562-25211c5c5123
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


#ifndef WIND_H
#define WIND_H

//--------------------------------------------------------------
// Wind Class
// Estimates wind direction using:
//   - Roll behaviour over heading bins
//   - Median filtering
//   - Symmetry of roll distribution
//
// Also tracks:
//   - Port/Starboard tack roll clusters
//   - Statistical distribution of roll values
//--------------------------------------------------------------

class Wind {

public:
    Wind();

    // --- Core updates ---
    void Update(float bearingDeg, signed char rollDeg);
    void UpdateDistribution(signed char rollDeg);
    void UpdateRoll(float bearingDeg, signed char rollDeg);

    // --- Public outputs ---
    float Direction = 0;           // Estimated wind direction (degrees)
    float portTackBearing = 0;     // Wind + 50°
    float starboardTackBearing = 0;// Wind - 50°

    // --- Distribution statistics ---
    float mean = 0;
    float variance = 0;
    float stdev = 0;
    char  msg = 'X';               // Encoded roll message

private:
    // --- Constants ---
    static const int n = 36;       // 10° bins (0–350°)
    float lambda = 0.1;            // Smoothing factor
    float discount = 0.99;         // Distribution decay

    // --- Wind estimation arrays ---
    float Wind01[n];               // Smoothed roll values
    float WindSort[n];             // Temporary sorted array
    int   Windyn[n];               // Above/below median flags
    int   Windsum[n];              // Sliding window sums
    float WindAccelerator[n];      // (Unused but preserved)

    // --- Roll distribution (−30° to +30°) ---
    int rollWind[61];              // Histogram
    int rollPort = 0;
    int rollStarboard = 0;
    int rollCentre = 0;

    // --- Distribution accumulators ---
    float sum_w = 0;
    float sum_wx = 0;
    float sum_wxx = 0;

    // --- Temporary variables ---
    float tmp = 0;
    int   tmpi = 0;
};

#endif