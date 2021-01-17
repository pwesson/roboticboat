// Teensy 4.0 with Kyosho Seawind
// Copyright (C) 2020 https://www.roboticboat.uk
// 5925b03d-41c7-4051-b890-02b7775383dd
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
#include <math.h>
#include <stdlib.h>
#include "Arduino.h"

// Converts degrees to radians.
//#define radians(angleDegrees) (angleDegrees * M_PI / 180.0)
 
// Converts radians to degrees.
//#define degrees(angleRadians) (angleRadians * 180.0 / M_PI)

Rudder::Rudder(){

   // Initialise the Rudder object

  maxposition = 180;
  minposition = 0;
}

void Rudder::GyroUpdateMean(float bearing, float gyroznew)
{
  // We have received a new bearing. Is it different from the old bearing

  if (bearing == gyrooldbearing)
  {
    // Increase the static bearing counter
    gyroNbearing++;

    // Is the boat stationary? If so can calibrate the gyro
    // Two is an arbitary number at the moment
    if (gyroNbearing > 2)
    {
      // The boat is considered stationary
      gyrozmean = gyrolambda * gyroznew + (1-gyrolambda) * gyrozmean;
    }
  }
  else
  {
    // Reset the static bearing counter
    gyroNbearing = 0;
  }

  // Remember the old bearing
  gyrooldbearing = bearing;
}

void Rudder::AddGyroReading(int rudder, float gyroznew)
{
  // Add new reading of gyro is rudder is within a range
  if (rudder >= minposition && rudder <=maxposition) 
  {
    // Want to add the rotation rate that have been calibrated
    gyro.AddReading(rudder, gyroznew - gyrozmean);
  }
}

void Rudder::SetGyroRegressionStatic(float setslope, float setintercept)
{
  // Set the gyro regression parameters
  gyro.SetRegression(setslope, setintercept);
}

void Rudder::AddReading(unsigned long mytime, int rudder, float bearingchanged)
{
  // Add new reading of bearing change over 1 second 
  if (rudder >=minposition && rudder <=maxposition) wls.AddReading(rudder, bearingchanged);

  // Update the moving average of the rudder position
  rudderma = lambda * rudder + (1 - lambda) * rudderma;
}

void Rudder::ClearLine()
{
  //Clear the waypoint line A and B
  waypointA.latitude = 0;
  waypointA.longitude = 0;
    
  waypointB.latitude = 0;
  waypointB.longitude = 0;

  alongtrack = 0;
  crosstrack = 0;
}


void Rudder::SetSensitivity(int val)
{
   sensitivity = val;
}

void Rudder::NewLine(double latitude, double longitude, double bearing)
{
  // The Human was in control, so now initialise the Robot task
  // The Robot wants to know the line AB to follow. 
      
  // The current position of the boat
  waypointX.latitude = latitude;
  waypointX.longitude = longitude;

  // Calculate the waypoint B that is 1000 meters ahead
  // The current boat heading is bearing (degrees)
  waypointB = nav.waypoint(waypointX, bearing, 1000);

  // Calculate the waypoint A which is 100 meters behind
  // This is required as if the boat slips backwards slightly, it will still be between A and B
  waypointA = nav.waypoint(waypointX, nav.clean(bearing - 180), 100);

  // Target distance to travel along AB
  // We start the line already 100m along it.
  alongtrackobjective = 1100;

}


void Rudder::NewLine(double latitude, double longitude, double nextlat, double nextlon)
{
  // The current position of the boat
  waypointX.latitude = latitude;
  waypointX.longitude = longitude;

  // The next waypoint to move towards
  waypointF.latitude = nextlat;
  waypointF.longitude = nextlon;

  // What is the bearing from X to the waypoint F?
  double nextbearing = nav.bearing(waypointX, waypointF);

  // Calculate the waypoint B that is 1000 meters ahead
  // The current boat heading is bearing (degrees)
  waypointB = nav.waypoint(waypointX, nextbearing, 1000);

  // Calculate the waypoint A which is 100 meters behind
  // This is required as if the boat slips backwards slightly, it will still be between A and B
  waypointA = nav.waypoint(waypointX, nav.clean(nextbearing - 180), 100);

  // The distance to travel along line AB is AF
  // We start the line already 100m along it
  alongtrackobjective = nav.distance(waypointA, waypointF);

}

int Rudder::Update(double latitude, double longitude, float bearing)
{
   // Where is the boat along the line AB?
   // WARNING - this logic does assume the GPS location is known and active.
      
   // The current position of the boat
   waypointX.latitude = latitude;
   waypointX.longitude = longitude;
      
   // Distance along the track between A and B
   alongtrack = nav.alongtrack(waypointA, waypointB, waypointX);
      
   // Distance to the track between A and B
   // Not used in the calculations, but nice to know as its the error
   crosstrack = nav.crosstrack(waypointA, waypointB, waypointX);
      
   // Lat/Lon closest point along the track to the boat
   waypointD = nav.pointontrack(waypointA, waypointB, waypointX);
   
   // What is the bearing of D to the waypoint B?
   double localbearing = nav.bearing(waypointD, waypointB);

   // Create a local waypointE which can be set 10 meters, along the line, from D
   waypointE = nav.waypoint(waypointD, localbearing , localdistance);
   
   // Now calculate bearing from X (current location) to local waypointE
   targetbearing = nav.clean(nav.bearing(waypointX, waypointE));

   // The difference between the boat heading and the targetbearing
   requiredBearingChange = targetbearing - bearing;

   // Ensure the change in bearing is between -180 and 180 degrees
   if (requiredBearingChange > 180) requiredBearingChange -= 360;
   if (requiredBearingChange < -180) requiredBearingChange += 360;

   // Restrict the maximum change in the bearing, so rudder can keep up
   targetbearing = nav.clean(bearing + requiredBearingChange);

   //Restrict the rudder position (within to be between min and max position of servo)
   //Halve the degree error by 2, so hopefully get on target in 2 seconds.
   
   // Negative sign as gyro has positive change as bearing reduce
   return (int)gyro.PredictXlimit(-requiredBearingChange/sensitivity, minposition , maxposition);

}

float Rudder::ChooseTarget(float windDirection, float bearing)
{
   // Want to find the tack bearing to be on
   // windDirection +/- 60 degrees say
  
   float windPort = windDirection + 60;
   if (windPort > 360) {windPort -= 360;}

   float windStar = windDirection - 60;
   if (windStar < 0) {windStar += 360;}

   // The difference between the boat heading and the targetbearing
   portBearingChange  = windPort - bearing;
   starBearingChange  = windStar - bearing;

   // Ensure the change in bearing is between -180 and 180 degrees
   if (portBearingChange > 180) portBearingChange -= 360;
   if (portBearingChange < -180) portBearingChange += 360;

   // Ensure the change in bearing is between -180 and 180 degrees
   if (starBearingChange > 180) starBearingChange -= 360;
   if (starBearingChange < -180) starBearingChange += 360;

   // Choose the bearing that is closest
   if (abs(portBearingChange) < abs(starBearingChange))
   {
      // Have chosen the Port tack
      return windPort;
   }

  // Have chosen the Starboard tack
  return windStar;
}

int Rudder::UpdateTarget(float target, float bearing)
{
   // The difference between the boat heading and the targetbearing
   requiredBearingChange = target - bearing;

   // Ensure the change in bearing is between -180 and 180 degrees
   if (requiredBearingChange > 180) requiredBearingChange -= 360;
   if (requiredBearingChange < -180) requiredBearingChange += 360;

   // Restrict the maximum change in the bearing, so rudder can keep up
   targetbearing = nav.clean(bearing + requiredBearingChange);

   //Restrict the rudder position (within to be between min and max position of servo)
   //Halve the degree error by 2, so hopefully get on target in 2 seconds.
   
   // Negative sign as gyro has positive change as bearing reduce
   return (int)gyro.PredictXlimit(-requiredBearingChange/sensitivity, minposition , maxposition);
}

int Rudder::Update(float degreechange)
{
   // This should compensate for some weather helm
   return (int)gyro.PredictXlimit(-degreechange/sensitivity, minposition , maxposition);
}