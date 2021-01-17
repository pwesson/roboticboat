// Navigation functions
// Copyright (C) 2020 https://www.roboticboat.uk
// c75b2bac-9edf-419f-bd4a-3ae8dd930f3e
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


#ifndef Navigation_h
  #define Navigation_h

  struct coordinate {
    double latitude;
    double longitude;
  };

  class Navigation{
	public:
		double distance(coordinate, coordinate);

		double bearing(coordinate, coordinate);

 		double crosstrack(coordinate, coordinate, coordinate);

		double alongtrack(coordinate, coordinate, coordinate);

		coordinate waypoint(coordinate, double, double);

		coordinate pointontrack(coordinate, coordinate, coordinate);

		float clean(float);

	private:
		double _earthRadius = 6371000;

  };

#endif