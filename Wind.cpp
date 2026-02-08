// Wind logic
// Copyright (C) 2026
// 6d87464e-3e1f-42c3-8bb3-dac59a1a7b75
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


#include "Wind.h"
#include <math.h>

//--------------------------------------------------------------
// Constructor
// Initialises arrays for wind estimation and roll histogram.
//--------------------------------------------------------------
Wind::Wind()
{
    // Initialise wind arrays
    for (int i = 0; i < n; i++) {
        Wind01[i] = i % 2;   // Alternating initial pattern
        Windsum[i] = 0;
        WindAccelerator[i] = 0;
    }

    // Initialise roll histogram (−30° to +30°)
    for (int i = 0; i < 61; i++) {
        rollWind[i] = 0;
    }
}

//--------------------------------------------------------------
// Update()
// Main wind-direction estimator.
// Uses roll values binned by bearing, median filtering,
// and symmetry to estimate wind direction.
//--------------------------------------------------------------
void Wind::Update(float bearing, signed char roll)
{
    // Convert bearing to array index
    int i = (n * bearing) / 360;
    if (i < 0) i = 0;
    if (i >= n) i = n - 1;

    // Exponential smoothing of roll values
    Wind01[i] = lambda * roll + (1 - lambda) * Wind01[(i - 1 + n) % n];

    // --- Median filtering ---
    for (int k = 0; k < n; k++)
        WindSort[k] = Wind01[k];

    // Full sort (n is small, so OK)
    for (int a = 0; a < n - 1; a++) {
        for (int b = a + 1; b < n; b++) {
            if (WindSort[b] < WindSort[a]) {
                float t = WindSort[a];
                WindSort[a] = WindSort[b];
                WindSort[b] = t;
            }
        }
    }

    float median = WindSort[n / 2];

    // Flag values above median
    for (int k = 0; k < n; k++)
        Windyn[k] = (Wind01[k] >= median) ? 1 : 0;

    // --- Sliding window sum over half the circle ---
    int sum_ones = 0;
    for (int k = 0; k < n / 2; k++)
        sum_ones += Windyn[k];

    for (int k = 0; k < n / 2; k++) {
        Windsum[k] = sum_ones;
        Windsum[n / 2 + k] = (n / 2) - sum_ones;

        sum_ones += Windyn[n / 2 + k] - Windyn[k];
    }

    // --- Find maximum sum → wind direction ---
    Direction = 0;
    int maxWindsum = 0;

    for (int k = 0; k < n; k++) {
        if (Windsum[k] > maxWindsum) {
            maxWindsum = Windsum[k];
            Direction = 10 * k; // Convert bin index to degrees
        }
    }

    // --- Compute tack bearings ---
    portTackBearing = Direction + 50;
    if (portTackBearing >= 360) portTackBearing -= 360;

    starboardTackBearing = Direction - 50;
    if (starboardTackBearing < 0) starboardTackBearing += 360;
}

//--------------------------------------------------------------
// UpdateDistribution()
// Maintains a decaying statistical distribution of roll values.
//--------------------------------------------------------------
void Wind::UpdateDistribution(signed char roll)
{
    float r = (float)roll;

    // Update weighted sums
    sum_w   = discount * sum_w   + 1;
    sum_wx  = discount * sum_wx  + r;
    sum_wxx = discount * sum_wxx + r * r;

    // Mean
    mean = sum_wx / sum_w;

    // Variance
    variance = sum_wxx - 2 * sum_wx * mean + sum_w * mean * mean;
    variance /= sum_w;

    stdev = (variance > 0) ? sqrt(variance) : 0;

    // Encode roll message
    msg = 'X';
    if (r < mean - 0.5 * stdev) msg = 'l';
    if (r < mean - 1.0 * stdev) msg = 'L';
    if (r > mean + 0.5 * stdev) msg = 'r';
    if (r > mean + 1.0 * stdev) msg = 'R';
}

//--------------------------------------------------------------
// UpdateRoll()
// Tracks roll distribution to identify port/starboard clusters.
// Only updates when the boat is sailing *into* the wind.
//--------------------------------------------------------------
void Wind::UpdateRoll(float bearing, signed char roll)
{
    // Only update if heading is within ±90° of wind direction
    float diff = bearing - Direction;
    if (diff < -180) diff += 360;
    if (diff > 180) diff -= 360;
    if (diff >= 90 || diff <= -90) return;

    // Only accept roll values in [-30, +30]
    int r = (int)roll;
    if (r < -30 || r > 30) return;

    // Shift to index 0–60
    rollWind[r + 30]++;

    // --- Find port cluster (left side) ---
    rollPort = 0;
    tmpi = rollWind[0];
    for (int i = 0; i <= 20; i++) {
        if (rollWind[i] > tmpi) {
            tmpi = rollWind[i];
            rollPort = i;
        }
    }

    // --- Find starboard cluster (right side) ---
    rollStarboard = 60;
    tmpi = rollWind[60];
    for (int i = 60; i >= 40; i--) {
        if (rollWind[i] > tmpi) {
            tmpi = rollWind[i];
            rollStarboard = i;
        }
    }

    // --- Find minimum between the two peaks (centre) ---
    rollCentre = rollPort;
    tmpi = rollWind[rollPort];

    for (int i = rollPort; i <= rollStarboard; i++) {
        if (rollWind[i] < tmpi) {
            tmpi = rollWind[i];
            rollCentre = i;
        }
    }
}