// Teensy 4.0 with Kyosho Seawind
// Copyright (C) 2020 https://www.roboticboat.uk
// 3f37dc03-1122-4b96-b038-f50b2edd909f
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

Wind::Wind(){

   // Initialise the Wind array

   for (int i=0; i<n; i++)
   {
      Wind01[i] = i % 2;
      Windsum[i] = 0;
      WindAccelerator[i] = 0;
   }

   // Initialise the rollWind array. This array counts the number of
   // times a certain roll occurs, from -30 degrees to 30 degrees. 
   // Rolls outside this range are ignored.
   for (int i=0; i<61; i++)
   {
      rollWind[i] = 0;
   }
}

void Wind::Update(float bearing, signed char roll)
{
  //Want to update the Wind array
  //Will range from 0 to n
  int i = (n * bearing)/360; 
  
  if (i<0) { i = 0;}
  if (i>=n) { i = n-1;}

  Wind01[i] = lambda * roll + (1-lambda) * Wind01[i-1]; 

  //Want to find the median of the Wind array
   
  //Copy the array first
  for (int i=0; i<n; i++)
  {
    WindSort[i] = Wind01[i];    
  }

  //Sort into order - looking for the median
  //So could do 1/2 the sort
  for (int i=0; i<n-1; i++)
  {
    for (int j=i+1; j<n; j++)
    {
      if (WindSort[j] < WindSort[i])
      {
        tmp = WindSort[i];
        WindSort[i] = WindSort[j];
        WindSort[j] = tmp;          
      }
    }
  }

  float median = WindSort[n/2];

  //Now flag the values above the median
  for (int i=0; i<n; i++)
  {
    Windyn[i] = 0;
    if (Wind01[i] >= median) Windyn[i] = 1;
  }

  int sum_ones = 0;
  for (int i=0; i<n/2; i++)
  {
     sum_ones = sum_ones + Windyn[i];
  }

  for (int i=0; i<n/2; i++)
  {
    //Save the result
     Windsum[i] = sum_ones;

     //From symmetry we also know
     Windsum[n/2+i] = n/2 - Windsum[i];

     //Update the sum. Add the end one and take off the first
     sum_ones = sum_ones + Windyn[n/2+i] - Windyn[i];
  }

  //What is the max Windsum
  Direction = 0;
  int maxWindsum = 0;

  for (int i=0; i<n; i++)
  {
     if (Windsum[i] > maxWindsum)
     {
        maxWindsum = Windsum[i];

	// Wind direction in degrees
        // Convert from 0 to 35, to 0 to 350
        Direction = 10 * i;  
     }
  }

  // Starboard & Port tacks
  portTackBearing = Direction + 50;
  if (portTackBearing > 360) portTackBearing = portTackBearing - 360;

  starboardTackBearing = Direction - 50;
  if (starboardTackBearing < 0) starboardTackBearing = starboardTackBearing + 360;

}

void Wind::UpdateDistribution(signed char roll)
{
    float r = (float)roll;

    // Update the cumulative sums
    sum_w = discount * sum_w + 1;
    sum_wx = discount * sum_wx + r;
    sum_wxx = discount * sum_wxx + r * r;

    // Mean of the distribution
    mean = sum_wx / sum_w;

    // Standard deviation of the distribution
    variance = sum_wxx - 2 * sum_wx * mean + sum_w * mean*mean;
    variance = variance / sum_w;
    if (variance>0)
    {
      stdev = sqrt(variance);
    }
    else
    {
      stdev = 0;
    }

    // Update the message
    msg = 'X';
    if (r < mean - 0.5 * stdev) {msg = 'l';}
    if (r < mean - 1.0 * stdev) {msg = 'L';}

    if (r > mean + 0.5 * stdev) {msg = 'r';}
    if (r > mean + 1.0 * stdev) {msg = 'R';}

}

void Wind::UpdateRoll(float bearing, signed char roll)
{
    // The idea of this function is that the boat is either on starboard tack or port tack.
    // Thus if counting the occurance of each roll degree that it should be one lump to the port
    // and one lump to starboard.

    // Only do this calculation if the boat is actually heading towards the wind, thus need to
    // know the boat heading and the estimated wind direction

    tmp = bearing - (float)Direction;
    if (tmp < -180) {tmp = tmp + 360;}
    if (tmp > 180) {tmp = tmp - 360;}
    if (tmp >= 90 || tmp <= -90) {return;}

    // Only interest in integer roll numbers
    int r = (int)roll;

    // Not interest in high roll numbers
    if (r < -30) {return;}
    if (r > 30) {return;}
    
    // So we have a r between [-30 and 30]. Add r+30 thus gives the count array entry
    // Increase the accurancy by 1.
    rollWind[r+30]++;

    // Update the statistics rollPort, rollStarboard, and rollCentre;

    // Update rollPort from bottom of array towards the middle
    rollPort = 0;
    tmpi = rollWind[rollPort];
    for (int i=0; i<=20; i++)
    {
       if (rollWind[i] > tmpi)
       {
         // Found a new maximum
         tmpi = rollWind[i];
         rollPort = i;
       }
    }

    // Update rollStarboard from top of array towards middle
    rollStarboard = 60;
    tmp = rollWind[rollStarboard];
    for (int i=60; i>=40; i--)
    {
       if (rollWind[i] > tmp)
       {
         // Found a new maximum
         tmpi = rollWind[i];
         rollStarboard = i;
       }
    }

    // Now find lowest between rollPort and rollStarboard
    tmpi = rollWind[rollPort];
    for (int i=rollPort; i<=rollStarboard; i++)
    {
       if (rollWind[i] < tmpi)
       {
          // Found a new minimum
          tmpi = rollWind[i];
          rollCentre = i;     
       }
    }    
    
}
