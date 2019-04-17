

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <limits>

#ifndef Geo_Info
#define Geo_Info


using namespace geographic_msgs;
using namespace geometry_msgs;
using namespace std;


namespace eDrone
{

	class GeoInfo
	{
		private:
			GeoInfo(){};
			~GeoInfo(){};
						
			static GeoInfo* instance;
			
			Point position_local;
			GeoPoint position_geo;
			GeoPoint homePosition;

		public:

			static GeoInfo* getInstance();
		
			void setLocalPosition(Point pos_local);
			Point getLocalPosition();			

			void setGlobalPosition(GeoPoint pos_geo);
			GeoPoint getGlobalPosition();

			void setHomePosition(GeoPoint home_geo); 
			GeoPoint getHomePosition();
			
			

	};

} // namespace eDrone

#endif

