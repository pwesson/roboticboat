// Rudder logic
// Copyright (C) 2026
// af6de076-2c5e-4a1e-ae92-1622619e380c
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

#include "Rudder.h"
#include <Arduino.h>
#include <math.h>

//--------------------------------------------------------------
// Constructor
//--------------------------------------------------------------
Rudder::Rudder() {
    minposition = 0;
    maxposition = 180;
}

//--------------------------------------------------------------
// GyroUpdateMean()
// Updates the mean gyro value when the boat is stationary.
//--------------------------------------------------------------
void Rudder::GyroUpdateMean(float bearing, float gyroznew)
{
    // If bearing hasn't changed, boat may be stationary
    if (bearing == gyrooldbearing) {
        gyroNbearing++;

        // After N stable readings, update mean gyro drift
        if (gyroNbearing > 2) {
            gyrozmean = gyrolambda * gyroznew + (1 - gyrolambda) * gyrozmean;
        }
    } else {
        gyroNbearing = 0;
    }

    gyrooldbearing = bearing;
}

//--------------------------------------------------------------
// AddGyroReading()
// Adds a gyro reading only when rudder is within valid range.
//--------------------------------------------------------------
void Rudder::AddGyroReading(int rudder, float gyroznew)
{
    if (rudder >= minposition && rudder <= maxposition) {
        gyro.AddReading(rudder, gyroznew - gyrozmean);
    }
}

//--------------------------------------------------------------
// SetGyroRegressionStatic()
// Manually sets slope/intercept for gyro regression.
//--------------------------------------------------------------
void Rudder::SetGyroRegressionStatic(float slope, float intercept)
{
    gyro.SetRegression(slope, intercept);
}

//--------------------------------------------------------------
// AddReading()
// Adds bearing-change reading and updates rudder moving average.
//--------------------------------------------------------------
void Rudder::AddReading(unsigned long mytime, int rudder, float bearingchanged)
{
    if (rudder >= minposition && rudder <= maxposition)
        wls.AddReading(rudder, bearingchanged);

    // Update moving average of rudder position
    rudderma = lambda * rudder + (1 - lambda) * rudderma;
}

//--------------------------------------------------------------
// ClearLine()
// Resets all line-following waypoints.
//--------------------------------------------------------------
void Rudder::ClearLine()
{
    waypointA = {0,0};
    waypointB = {0,0};
    alongtrack = 0;
    crosstrack = 0;
}

//--------------------------------------------------------------
// SetSensitivity()
// Controls how aggressively the rudder responds.
//--------------------------------------------------------------
void Rudder::SetSensitivity(int val)
{
    sensitivity = val;
}

//--------------------------------------------------------------
// NewLine() - version 1
// Creates a line based on current position + heading.
//--------------------------------------------------------------
void Rudder::NewLine(double latitude, double longitude, double bearing)
{
    waypointX = {latitude, longitude};

    waypointB = nav.waypoint(waypointX, bearing, 1000);
    waypointA = nav.waypoint(waypointX, nav.clean(bearing - 180), 100);

    alongtrackobjective = 1100; // 100m behind + 1000m ahead
}

//--------------------------------------------------------------
// NewLine() - version 2
// Creates a line based on current position + next waypoint.
//--------------------------------------------------------------
void Rudder::NewLine(double latitude, double longitude, double nextlat, double nextlon)
{
    waypointX = {latitude, longitude};
    waypointF = {nextlat, nextlon};

    double nextbearing = nav.bearing(waypointX, waypointF);

    waypointB = nav.waypoint(waypointX, nextbearing, 1000);
    waypointA = nav.waypoint(waypointX, nav.clean(nextbearing - 180), 100);

    alongtrackobjective = nav.distance(waypointA, waypointF);
}

//--------------------------------------------------------------
// Update()
// Main navigation update: computes target bearing and rudder.
//--------------------------------------------------------------
int Rudder::Update(double latitude, double longitude, float bearing)
{
    waypointX = {latitude, longitude};

    alongtrack = nav.alongtrack(waypointA, waypointB, waypointX);
    crosstrack = nav.crosstrack(waypointA, waypointB, waypointX);
    waypointD = nav.pointontrack(waypointA, waypointB, waypointX);

    double localbearing = nav.bearing(waypointD, waypointB);
    waypointE = nav.waypoint(waypointD, localbearing, localdistance);

    targetbearing = nav.clean(nav.bearing(waypointX, waypointE));

    requiredBearingChange = targetbearing - bearing;

    // Wrap to [-180, 180]
    if (requiredBearingChange > 180) requiredBearingChange -= 360;
    if (requiredBearingChange < -180) requiredBearingChange += 360;

    targetbearing = nav.clean(bearing + requiredBearingChange);

    return (int)gyro.PredictXlimit(-requiredBearingChange / sensitivity,
                                   minposition, maxposition);
}

//--------------------------------------------------------------
// ChooseTarget()
// Chooses port or starboard tack based on wind direction.
//--------------------------------------------------------------
float Rudder::ChooseTarget(float windDirection, float bearing)
{
    float windPort = fmod(windDirection + 60 + 360, 360);
    float windStar = fmod(windDirection - 60 + 360, 360);

    portBearingChange = windPort - bearing;
    starBearingChange = windStar - bearing;

    // Wrap both to [-180, 180]
    if (portBearingChange > 180) portBearingChange -= 360;
    if (portBearingChange < -180) portBearingChange += 360;

    if (starBearingChange > 180) starBearingChange -= 360;
    if (starBearingChange < -180) starBearingChange += 360;

    return (abs(portBearingChange) < abs(starBearingChange))
           ? windPort
           : windStar;
}

//--------------------------------------------------------------
// UpdateTarget()
// Steers toward a specific bearing.
//--------------------------------------------------------------
int Rudder::UpdateTarget(float target, float bearing)
{
    requiredBearingChange = target - bearing;

    if (requiredBearingChange > 180) requiredBearingChange -= 360;
    if (requiredBearingChange < -180) requiredBearingChange += 360;

    targetbearing = nav.clean(bearing + requiredBearingChange);

    return (int)gyro.PredictXlimit(-requiredBearingChange / sensitivity,
                                   minposition, maxposition);
}

//--------------------------------------------------------------
// Update() - simple version
// Applies gyro regression to a known degree change.
//--------------------------------------------------------------
int Rudder::Update(float degreechange)
{
    return (int)gyro.PredictXlimit(-degreechange / sensitivity,
                                   minposition, maxposition);
}
