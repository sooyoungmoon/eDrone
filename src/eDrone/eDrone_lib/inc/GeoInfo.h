

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


namespace Mission_API
{

	class GeoInfo
	{
		private:
			GeoInfo(){printf("GeoInfo(): cnt is %d", cnt); };
			~GeoInfo(){instanceFlag = false;};

			static bool instanceFlag;

			//static GeoInfo* instance;
			static GeoInfo instance;

			static int cnt;
			
			Point position_local;

			GeoPoint position_geo;

			GeoPoint homePosition;

		public:

			static GeoInfo& getInstance();

			static void add();
			static void sub();
		
			void setLocalPosition(Point pos_local);
			Point getLocalPosition();			

			void setGlobalPosition(GeoPoint pos_geo);
			GeoPoint getGlobalPosition();

			void setHomePosition(GeoPoint home_geo); 
			GeoPoint getHomePosition();
			
			

	};

} // namespace Mission_API

#endif

