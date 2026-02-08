// Rudder logic
// Copyright (C) 2026
// 45886ce6-fdb4-47fa-a79c-d34a9543a321
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


#ifndef RUDDER_H
#define RUDDER_H

#include <Navigation.h>
#include <WeightedLeastSquares.h>

//--------------------------------------------------------------
// Rudder Class
// Handles:
//  - Line-following navigation (waypoints A → B)
//  - Rudder control based on bearing error
//  - Gyro calibration and regression
//  - Moving averages and smoothing
//--------------------------------------------------------------

class Rudder {

public:
    Rudder();

    // --- Line creation and clearing ---
    void NewLine(double lat, double lon, double bearingDeg);
    void NewLine(double lat, double lon, double nextLat, double nextLon);
    void ClearLine();

    // --- Data input ---
    void AddReading(unsigned long t, int rudderPos, float bearingChange);
    void AddGyroReading(int rudderPos, float gyroz);
    void GyroUpdateMean(float bearing, float gyroz);
    void SetGyroRegressionStatic(float slope, float intercept);
    void SetSensitivity(int val);

    // --- Rudder control ---
    int Update(double lat, double lon, float bearingDeg);
    int Update(float degreeChange);
    int UpdateTarget(float targetBearing, float currentBearing);
    float ChooseTarget(float windDirection, float currentBearing);

    // --- Public navigation helpers ---
    Navigation nav;
    WeightedLeastSquares wls;   // Bearing regression
    WeightedLeastSquares gyro;  // Gyro regression

    // Rudder limits
    int minposition = 0;
    int maxposition = 180;

    // Waypoints
    coordinate waypointX;
    coordinate waypointA;
    coordinate waypointB;
    coordinate waypointD;
    coordinate waypointE;
    coordinate waypointF;

    // Tracking variables
    float alongtrack = 0;
    float alongtrackobjective = 0;
    float crosstrack = 0;
    float localdistance = 10;

    // Bearing-related values
    float requiredBearingChange = 0;
    float targetbearing = 0;
    float portBearingChange = 0;
    float starBearingChange = 0;

    // Rudder smoothing
    float rudderma = 0;
    float lambda = 0.1;
    float sensitivity = 2;

    // Gyro calibration
    int   gyroNbearing = 0;
    float gyrolambda = 0.01;
    float gyrooldbearing = 0;
    float gyrozmean = 0;

private:

};

#endif