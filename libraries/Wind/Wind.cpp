#include "Wind.h"

Wind::Wind(){

   // Initialise the Wind array

   for (int i=0; i<n; i++)
   {
      Wind01[i] = i % 2;
      Windsum[i] = 0;
   }
}

void Wind::Update(float bearing, signed char roll)
{
   //Want to update the Wind array
  int i = (n * bearing)/360; //Will range from 0 to n
  
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

}