// Navigation functions
// Copyright (C) 2020 https://www.roboticboat.uk
// fae5cec2-a0c3-4edf-8008-d6b261ea9cc5
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


#include <math.h> 
#include <arduino.h>
#include "Navigation.h"

double Navigation::distance(coordinate placeA, coordinate placeB)
{
   //The distance between two points on the Earth
   //
   //
   //   A <------distance--------> B
   //
   //

   // Convert degrees to radians
   double phi0 = radians(placeA.latitude);
   double lambda0 = radians(placeA.longitude);

   double phi1 = radians(placeB.latitude);
   double lambda1 = radians(placeB.longitude);

   // The difference between the two places
   double dphi = phi1 - phi0;
   double dlambda = lambda1 - lambda0;	

   double a = pow(sin(dphi/2),2) + cos(phi0)*cos(phi1)*pow(sin(dlambda/2),2);

   double delta = 2 * atan2(sqrt(a), sqrt(1-a));

   // Return distance in meters
   return _earthRadius * delta;
}


double Navigation::bearing(coordinate placeA, coordinate placeB)
{
   // Start bearing to the second waypoint
   //
   //       B
   //      /
   //     /  find the bearing from A to B
   //    /
   //   A

   // Convert degrees to radians
   double phi0 = radians(placeA.latitude);
   double lambda0 = radians(placeA.longitude);

   double phi1 = radians(placeB.latitude);
   double lambda1 = radians(placeB.longitude);

   // The difference between the two places
   //double dphi = phi1 - phi0;
   double dlambda = lambda1 - lambda0;

   double rad = atan2(sin(dlambda)*cos(phi1), cos(phi0)*sin(phi1)-sin(phi0)*cos(phi1)*cos(dlambda));

   // Return bearing in degrees
   return degrees(rad);
}


coordinate Navigation::waypoint(coordinate placeA, double bearing, double distance)
{
   // Lat Lon coordinates in a certain bearing and distance from start point
   //
   //       B
   //      /
   //     /  Start at A. Go distance on bearing to find B
   //    /
   //   A

   // Convert degrees to radians
   double phi0 = radians(placeA.latitude);
   double lambda0 = radians(placeA.longitude);

   // Bearing to the next waypoint
   double radbearing = radians(bearing);

   // Distance in units of Earth's radius. Ensure it is floating arithmetric
   double delta = distance / _earthRadius;

   double phi1 = asin(sin(phi0)*cos(delta)+cos(phi0)*sin(delta)*cos(radbearing));

   double lambda1 = lambda0 + atan2(sin(radbearing)*sin(delta)*cos(phi0), cos(delta)-sin(phi0)*sin(phi1));

   // Build the return coordinate (lon,lat pair)
   coordinate newplace;
   newplace.latitude = degrees(phi1);
   newplace.longitude = degrees(lambda1);
  
   return newplace;
}


double Navigation::crosstrack(coordinate placeA, coordinate placeB, coordinate placeC)
{
   // Find the distance to the track between A and B
   //
   //           C   
   //           |  distance C to D
   //           v
   //   A ------D----------------- B   
   //

   // Convert degrees to radians
   double phi0 = radians(placeA.latitude);
   double lambda0 = radians(placeA.longitude);

   double phi1 = radians(placeB.latitude);
   double lambda1 = radians(placeB.longitude);

   double phi2 = radians(placeC.latitude);
   double lambda2 = radians(placeC.longitude);

   // The difference between the two places
   //double dphi10 = phi1 - phi0;
   double dlambda1 = lambda1 - lambda0;

   double dphi20 = phi2 - phi0;
   double dlambda2 = lambda2 - lambda0;

   double a = pow(sin(dphi20/2),2) + cos(phi0)*cos(phi2)*pow(sin(dlambda2/2),2);

   double delta2 = 2 * atan2(sqrt(a), sqrt(1-a));

   double theta1 = atan2(sin(dlambda1)*cos(phi1), cos(phi0)*sin(phi1)-sin(phi0)*cos(phi1)*cos(dlambda1));

   double theta2 = atan2(sin(dlambda2)*cos(phi2), cos(phi0)*sin(phi2)-sin(phi0)*cos(phi2)*cos(dlambda2));
   
   double crosstrackdelta = sin(delta2) * sin(theta2 - theta1);

   // Return cross track distance in meters
   return _earthRadius * crosstrackdelta;
}


double Navigation::alongtrack(coordinate placeA, coordinate placeB, coordinate placeC)
{
   // Find the distance to the track between A and B
   //
   //           C   
   //           |  distance A to D
   //           |
   //   A ----->D----------------- B   
   //

   // Convert degrees to radians
   double phi0 = radians(placeA.latitude);
   double lambda0 = radians(placeA.longitude);

   double phi1 = radians(placeB.latitude);
   double lambda1 = radians(placeB.longitude);

   double phi2 = radians(placeC.latitude);
   double lambda2 = radians(placeC.longitude);

   // The difference between the two places
   //double dphi10 = phi1 - phi0;
   double dlambda1 = lambda1 - lambda0;

   double dphi20 = phi2 - phi0;
   double dlambda2 = lambda2 - lambda0;

   double a = pow(sin(dphi20/2),2) + cos(phi0)*cos(phi2)*pow(sin(dlambda2/2),2);

   double delta2 = 2 * atan2(sqrt(a), sqrt(1-a));

   double theta1 = atan2(sin(dlambda1)*cos(phi1), cos(phi0)*sin(phi1)-sin(phi0)*cos(phi1)*cos(dlambda1));

   double theta2 = atan2(sin(dlambda2)*cos(phi2), cos(phi0)*sin(phi2)-sin(phi0)*cos(phi2)*cos(dlambda2));
   
   double crosstrackdelta = sin(delta2) * sin(theta2 - theta1);	

   double alongtrackdelta = acos(cos(delta2) / cos(crosstrackdelta));

   // Return along track distance in meters
   return _earthRadius * alongtrackdelta ;	

   return 0;
}


coordinate Navigation::pointontrack(coordinate placeA, coordinate placeB, coordinate placeC)
{
   // Lat Lon coordinates in a certain bearing and distance from start point
   //
   // Find the lat,lon of D which is the projection of C onto the path
   //
   //   A ----->D----------------- B 
   //

   // Calculate the distance AD
   double distance = alongtrack(placeA, placeB, placeC);

   // The start bearing to B
   double angle = bearing(placeA, placeB);

   // Build the return coordinate (lon,lat pair);
   coordinate newplace;
   newplace = waypoint(placeA, angle, distance);
  
   return newplace;
}

float Navigation::clean(float bearing)
{
   if (bearing < 0) bearing += 360;
   if (bearing >= 360) bearing -= 360;

   return bearing;
}