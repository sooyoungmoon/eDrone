

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/SetMode.h>

using namespace geographic_msgs;
using namespace geometry_msgs;


namespace Mission_API
{

typedef struct str_state
{
	bool connected;
	bool armed;
	std::string flight_mode;

} VehicleState;

class Vehicle
{
  private:  

	// (2017.11.29) singleton 패턴 적용 - 외부에서 임의로 Vehicle 객체를 생성할 수 없도록 제한
	Vehicle(){};
	
	//// 현재 위치        

        // global position
/*
	double latitude;
	double longitude;
	double altitude;

	// local position 

	double local_x;
	double local_y;
	double local_z;
	
        // home position
	GeoPoint homePosition;
        double  home_lat;
        double  home_lon;
	*/
	// 무인기 현재 상태
      
	VehicleState cur_state;
 
	// (2017.11.29) singleton 패턴 적용

	static Vehicle* instance;

  public:

	// 인스턴스 획득

	static Vehicle* getInstance();
	
	// 현재 위치 get/set
	/*
	void setGlobalPosition(double lat, double lon, double alt);
	GeoPoint getGlobalPosition();

	void setLocalPosition(double x, double y, double z);
	Point getLocalPosition();
	
	// home position get/set
	
	void setHomePosition(GeoPoint home);
	GeoPoint getHomePosition ();
*/
	// 무인기 현재 상태 get/set


	void setState(mavros_msgs::State newState);
	VehicleState getState(); 		 

};

} // namespace Mission_API
