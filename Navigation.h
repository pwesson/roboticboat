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