
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include <vector>
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/HomePosition.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/CommandLong.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointPull.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "Vehicle.h"
#include "mission_lib/Arming.h" // 시동 서비스 헤더 파일 포함                                                 
#include "mission_lib/Takeoff.h" // 이륙 서비스 
#include "mission_lib/Landing.h" // 착륙 서비스 	"
#include "mission_lib/WaypointAdd.h" // 웨이포인트 추가 서비스 헤더 파일 포함
#include "mission_lib/WaypointPush.h" // 웨이포인트 목록 업로드 서비스 헤더 파일 포함
#include "mission_lib/WaypointPull.h" // 웨이포인트 목록 업로드 서비스 헤더 파일 포함
#include "mission_lib/Goto.h" // 무인기 위치 이동 서비스 헤더 파일 포함
#include "mission_lib/Target.h" // 현재 목적지 topic 메시지가 선언된 헤더 파일 포함
#include "mission_lib/Geofence.h" // Geofence 서비스 헤더 파일
#include "mission_lib/Noflyzone.h" // Noflyzone 서비스 헤더 파일


#define PI 3.14159265358979323846



using namespace std;
using namespace Mission_API; 
using namespace mavros_msgs;


//// 위도/경도/고도 <=> NED 좌표 변환 코드 (QGC에서 발췌)

/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/


#include <cmath>
#include <limits>

//#include "QGCGeo.h"

// These defines are private
#define M_DEG_TO_RAD (M_PI / 180.0)

#define M_RAD_TO_DEG (180.0 / M_PI)

#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* °C			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/

static const float epsilon = std::numeric_limits<double>::epsilon();

Point convertGeoToPoint(double coord_lat, double coord_long, double coord_alt, double home_lat, double home_long, double home_alt) {

    double lat_rad = coord_lat * M_DEG_TO_RAD;
    double lon_rad = coord_long* M_DEG_TO_RAD;

    double ref_lon_rad = home_long * M_DEG_TO_RAD;
    double ref_lat_rad = home_lat * M_DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double c = acos(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon);
    double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

    Point point;

    point.x = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
    point.y = k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;

    point.z = (coord_alt - home_alt);

    return point;
}

GeoPoint convertPointToGeo(double x, double y, double z, double home_lat, double home_long, double home_altitude) {
    double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
    double sin_c = sin(c);
    double cos_c = cos(c);

    double ref_lon_rad = home_long * M_DEG_TO_RAD;
    double ref_lat_rad = home_lat * M_DEG_TO_RAD;

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double lat_rad;
    double lon_rad;

    if (fabs(c) > epsilon) {
        lat_rad = asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c);
        lon_rad = (ref_lon_rad + atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c));

    } else {
        lat_rad = ref_lat_rad;
        lon_rad = ref_lon_rad;
    }

    GeoPoint geoPoint;

   

    double latitude = lat_rad * M_RAD_TO_DEG;
    
    double longitude = lon_rad * M_RAD_TO_DEG;

    double altitude = z + home_altitude;

    cout << fixed;

	
    cout << "(" << latitude << ", " << longitude << ", " << altitude << ") " << endl;

    geoPoint.latitude  = latitude;
    geoPoint.longitude = longitude;
    geoPoint.altitude = altitude;

    return geoPoint;
}




//// 좌표 변환 코드




//// 거리 계산 함수 (http://www.geodatasource.com/developers/c )


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double deg2rad(double deg) {
  return (deg * PI / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
double rad2deg(double rad) {
  return (rad * 180 / PI);
}

double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
  double theta, dist;
  theta = lon1 - lon2;
  dist = sin(deg2rad(lat1)) * sin(deg2rad(lat2)) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta));
  dist = acos(dist);
  dist = rad2deg(dist);
  dist = dist * 60 * 1.1515;
  switch(unit) {
    case 'm': // meter
      dist = dist * 1609.344;
    break;
	
    case 'M':
      break;
    case 'K':
      dist = dist * 1.609344;
      break;
    case 'N':
      dist = dist * 0.8684;
      break;
  }
  return (dist);
}


typedef struct _str_target_position
{
	int target_seq_no; // 목적지 순번 (1, 2, 3, ...)
	bool is_global; // 좌표 유형 (true: global coord, false: local coord)
	
	Point pos_local; // (x, y, z)
	GeoPoint pos_global; // (lat, long, alt)

	bool reached; // 목적지 도달 여부 (true: 목적지 도착, false: 목적지로 이동 중)
	bool geofence_breach; // geofence 영역 위반 여부 (true: geofence 영역 밖, false: 영역 내) 
} Target_Position;


typedef struct _str_nofly_zone
{
	double min_x_lat;
	double min_y_long;

	double max_x_lat;
	double max_y_long;
	
	bool isSet; // true: 비행 금지 구역 설정, false: 비행 금지 구역 미 설정
} Nofly_Zone;

//// 목적지 변수

Target_Position target_position;
int num_targets; // 목적지 개수 (goto service마다 1 씩 증가)
bool autonomous_flight = false; // 자율 비행 여부  (goto service가 호출되면 true로 변경됨)


//// Vehicle 객체 선언

Vehicle vehicle;

float Geofence_Radius = 400;
enum Geofence_Policy {Warning, RTL, Loiter, Landing};
enum Geofence_Policy geofence_policy = RTL;

Nofly_Zone nofly_zone;


//// 메시지 변수 선언

// (무인기 상태 정보 수신 목적)
mavros_msgs::State current_state; // 무인기 상태 정보

// (무인기 위치 확인 목적)
geometry_msgs::PoseStamped current_pos_local; // 현재 위치 및 자세 정보 (지역 좌표)
sensor_msgs::NavSatFix current_pos_global; // 현재 위치 정보 (전역 좌표)


// (홈 위치 획득 목적)
mavros_msgs::HomePosition home_position; // home position

// (웨이포인트 요청 메시지 수신(응용프로그램)  및 송신 (mavros) 목적
vector<mavros_msgs::Waypoint> waypoints; // 웨이포인트 정보
mavros_msgs::WaypointList waypointList; // 웨이포인트 목록


// (무인기 위치 이동 목적)
geometry_msgs::PoseStamped target_pos_local; // 목적지 위치 정보 (지역 좌표)
mavros_msgs::GlobalPositionTarget target_pos_global; // 목적지 위치 정보 (전역 좌표)
mission_lib::Target cur_target; // 현재 목적지 정보 (publisher: mission_lib_node, subscriber: 응용 프로그램)



//// 서비스 요청 메시지 선언 (mavros)

mavros_msgs::CommandBool arm_cmd; // 시동 명령에 사용될 서비스 선언 
mavros_msgs::CommandTOL takeoff_cmd; // 이륙 명령에 사용될 서비스 선언 
mavros_msgs::CommandTOL landing_cmd; // 착륙 명령에 사용될 서비스 선언 
mavros_msgs::WaypointPush waypointPush_cmd; // 웨이포인트 업로드에 사용될 서비스 선언
mavros_msgs::WaypointPull waypointPull_cmd; // 웨이포인트 다운로드에 사용될 서비스 선언
mavros_msgs::CommandLong commandLong_cmd;// 무인기 제어에 사용될 서비스 선언
mavros_msgs::SetMode modeChange_cmd; // 모드 변경에 사용될 서비스 요청 메시지

// publisher 선언

//ros::Publisher state_pub;
ros::Publisher pos_pub_local;
ros::Publisher pos_pub_global;
ros::Publisher cur_target_pub; // (offboard control에 필요한) 현재 목적지 정보 (도착 여부 포함)


// subscriber 선언

ros::Subscriber state_sub;
ros::Subscriber pos_sub_local;
ros::Subscriber pos_sub_global;
ros::Subscriber home_sub; 
ros::Subscriber wpList_sub;

// 서비스 서버 선언
ros::ServiceServer arming_srv_server;
ros::ServiceServer takeoff_srv_server;
ros::ServiceServer landing_srv_server;
ros::ServiceServer waypointAdd_srv_server;
ros::ServiceServer waypointPush_srv_server;
ros::ServiceServer waypointPull_srv_server;
ros::ServiceServer goto_srv_server;
ros::ServiceServer geofence_srv_server;
ros::ServiceServer noflyzone_srv_server;



//서비스 클라이언트 선언
ros::ServiceClient arming_client; // 서비스 클라이언트 선언
ros::ServiceClient takeoff_client; // 서비스 클라이언트 선언
ros::ServiceClient landing_client; // 서비스 클라이언트 선언
ros::ServiceClient waypointAdd_client; // 	"
ros::ServiceClient waypointPush_client; // 	"
ros::ServiceClient waypointPull_client; // 	"
ros::ServiceClient commandLong_client; // "
ros::ServiceClient modeChange_client; // 모드 변경 서비스 클라이언트 


// home position


 float HOME_LAT ;
 float HOME_LON;
 float HOME_ALT;



void ref_system_conversion_test()
{
	printf("reference system conversion test: \n");

	printf("GeoPoint => Point\n");
	
//	Point point = convertGeoToPoint(HOME_LAT, HOME_LON, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );

	Point point = convertGeoToPoint(36.3847751, 127.3689272, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );
	cout << "(" << HOME_LAT  << ", " << HOME_LON << ", " << HOME_ALT << ") =  " << endl;

	cout << "(" << point.x << ", " << point.y << ", " << point.z << ") " << endl;

	printf("Point => GeoPoint\n");

	GeoPoint geoPoint = convertPointToGeo( 0.0, 0.0, 0.0, HOME_LAT, HOME_LON, HOME_ALT);

	
	cout << "(0, 0, 0) =  " << endl;
	cout << fixed;
	cout << "(" << geoPoint.latitude << ", " << geoPoint.longitude << ", " << geoPoint.altitude << ") " << endl;

}

void print_waypoints (vector<mavros_msgs::Waypoint> waypoints) // 웨이포인트 정보
{
	for (int i = 0; i < waypoints.size(); i++)
//	for (vector<mavros_msgs::Waypoint>::iterator it = std::begin(waypoints); it! = std::end(waypoints); it++)
	{
		cout << "waypoint[" << i << "]: ";

		cout << endl ;

		cout << waypoints[i] << endl;
	}
}



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;

	vehicle.setState (current_state);

	VehicleState cState = vehicle.getState();


	if (cState.flight_mode.compare("AUTO.RTL") ==0)
	{
		ROS_ERROR("state_cb(): FLIGHT MODE = RTL");
	}
/*
	if (cState.connected)
	{
 	  printf("A UAV is connected\n");
	}


	else
	{
  	  printf("A UAV is not connected\n");
	}

	if (cState.armed)
	{
	  printf("A UAV is armed\n");
	} 
	else
	{
  	  printf("A UAV is not armed\n");
	}
	
	cout << "flight mode:" <<  cState.flight_mode << endl;
*/
}


void pos_cb_local(const geometry_msgs::PoseStamped::ConstPtr& msg){

	current_pos_local = *msg;

	double DIST_RANGE = 0.5;	

	vehicle.setLocalPosition(current_pos_local.pose.position.x, current_pos_local.pose.position.y, current_pos_local.pose.position.z);

//					printf("current_position: (%f, %f, %f \n)", current_pos_local.pose.position.x, current_pos_local.pose.position.y, current_pos_local.pose.position.z);	


	// geofence 기능 추가

	double distance_home = 0;

	distance_home = current_pos_local.pose.position.x * current_pos_local.pose.position.x + current_pos_local.pose.position.y * current_pos_local.pose.position.y;

	distance_home = sqrt(distance_home);

	if (distance_home > Geofence_Radius)
	{
		ROS_ERROR("Geofence: boundary breach");
		printf("Geofence_Radius: %f\n", Geofence_Radius);

		if (geofence_policy == Warning)
		{		
			printf("Don't cross the line!!\n");
		}
		else if (geofence_policy == RTL)
		{
			modeChange_cmd.request.base_mode = 0;
			modeChange_cmd.request.custom_mode = "AUTO.RTL";
			modeChange_client.call(modeChange_cmd);
		}
		
		else if (geofence_policy == Loiter)
		{

			modeChange_cmd.request.base_mode = 0;
			modeChange_cmd.request.custom_mode = "AUTO.LOITER";
			modeChange_client.call(modeChange_cmd);
		}

		else if (geofence_policy == Landing)
		{

			modeChange_cmd.request.base_mode = 0;
			modeChange_cmd.request.custom_mode = "AUTO.LAND";
			modeChange_client.call(modeChange_cmd);
		}

	}


	// geofence (end)


	if (!autonomous_flight)
		return;

	if(  (target_position.reached == false) && (target_position.is_global == false))
	{
		// 목적지 도착 여부 확인
		double distance;

		if ( ((target_position.pos_local.x - DIST_RANGE) < current_pos_local.pose.position.x) &&
			 (current_pos_local.pose.position.x <(target_position.pos_local.x + DIST_RANGE) ) )
		{
			
			if ( ((target_position.pos_local.y - DIST_RANGE) < current_pos_local.pose.position.y) &&
			 (current_pos_local.pose.position.y <(target_position.pos_local.y + DIST_RANGE) ) )
			{
					
				if ( ((target_position.pos_local.z - 0.5) < current_pos_local.pose.position.z) && 
				(current_pos_local.pose.position.z <(target_position.pos_local.z +0.5) ) )
				{
					target_position.reached = true;
					ROS_ERROR("pos_cb_local(): The UAV reached to the target position");	

					printf("current_position: (%f, %f, %f \n)", current_pos_local.pose.position.x, current_pos_local.pose.position.y, current_pos_local.pose.position.z);	
					
			//		modeChange_cmd.request.base_mode = 0;
			//		modeChange_cmd.request.custom_mode = MAV_MODE_AUTO_ARMED;
			//		modeChange_cmd.request.custom_mode = "Hold";
			//		modeChange_client.call(modeChange_cmd);
				}

				
			}

		}
	}
}

void pos_cb_global(const sensor_msgs::NavSatFix::ConstPtr& msg){

	

	current_pos_global = *msg;

	vehicle.setGlobalPosition(current_pos_global.latitude, current_pos_global.longitude, current_pos_global.altitude);

//					printf("current_position: (%f, %f, %f \n)", current_pos_global.latitude, current_pos_global.longitude, current_pos_global.altitude-HOME_ALT);
	
	if (!autonomous_flight)
		return;

	if( (target_position.reached == false) && (target_position.is_global == true))
	

	{
		// 목적지 도착 여부 확인
		double distance;

		if ( ((target_position.pos_global.latitude - 0.0001) < current_pos_global.latitude) &&

			 (current_pos_global.latitude <(target_position.pos_global.latitude +0.0001) ) )
		{
			
			if ( ((target_position.pos_global.longitude - 0.0001) < current_pos_global.longitude) &&
			 (current_pos_global.longitude <(target_position.pos_global.longitude +0.0001) ) )
			{
					
				if ( ((target_position.pos_global.altitude - 0.5) < current_pos_global.longitude) && 
				(current_pos_global.longitude <(target_position.pos_global.longitude +0.5) ) )
				{
					target_position.reached = true;
					ROS_ERROR("pos_cb_global(): The UAV reached to the target position");	
					
					
					
				//	modeChange_cmd.request.base_mode = 0;
				//	modeChange_cmd.request.custom_mode = "Hold";
				//	modeChange_client.call(modeChange_cmd);
					//printf("current_position: (%f, %f, %f \n)", current_pos_global.latitude, current_pos_global.longitude, current_pos_global.altitude);	
				}

				
			}

		}
	}
}



void homePosition_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
	home_position = *msg;
	
//	printf("home position: (%f, %f, %f) \n", home_position.position.x, home_position.position.y, home_position.position.z);

	HOME_LAT = home_position.geo.latitude;
	HOME_LON = home_position.geo.longitude;		
	HOME_ALT = home_position.geo.altitude;
	vehicle.setHomePosition(home_position.geo);

	GeoPoint home = vehicle.getHomePosition(); 
	
//	printf("home position: (%f, %f, %f) \n", home.latitude, home.longitude, home.altitude);	
}

void wpList_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
	printf("wpList_cb(): waypointList was received \n");	
	waypointList = *msg;
	//print_waypoints(waypointList.waypoints);
}

// callback 함수 (서비스 제공) 정의

bool srv_arming_cb(mission_lib::Arming::Request &req, mission_lib::Arming::Response &res )
{

	ROS_ERROR("ARMing request received\n");
	arm_cmd.request.value = true; // 서비스 요청 메시지 필드 설정

	//// Arming

  	while (ros::ok() ) // 서비스 요청 메시지 전달
  	{
  	      printf("send Arming command ...\n");

		if (!arming_client.call(arm_cmd))
        	 {
	 	  ros::spinOnce();
 //   	  	 rate.sleep();
        	}
        	else break;

  	} 
	 
	ROS_ERROR("ARMing command was sent\n");

}


bool srv_takeoff_cb(mission_lib::Takeoff::Request &req, mission_lib::Takeoff::Response &res)
{

	ROS_ERROR("Takeoff request received\n");
	// 서비스 요청 메시지 필드 선언
	 takeoff_cmd.request.altitude = 15;

  	takeoff_cmd.request.latitude = HOME_LAT; // 자동으로 home position 값을 얻어 와서 설정되도록 변경 필요

  	takeoff_cmd.request.longitude = HOME_LON;
  
 	 takeoff_cmd.request.yaw = 0;

  	takeoff_cmd.request.min_pitch = 0;

	// 서비스 요청 메시지 전달 
  
	while (ros::ok() )
  	{
 		printf("send Takeoff command ...\n");
 	
		if (!takeoff_client.call(takeoff_cmd))
		{
		  ros::spinOnce();
                //  rate.sleep();
		}
		else break;
  	}

 	 ROS_ERROR("Takeoff command was sent\n");
}

bool srv_landing_cb(mission_lib::Landing::Request &req, mission_lib::Landing::Response &res)
{
	
	ROS_ERROR("Landing request received\n");
	//// 서비스 요청 메시지 필드 설정 

	landing_cmd.request.altitude = 10;

  	landing_cmd.request.latitude = HOME_LAT;

  	landing_cmd.request.longitude = HOME_LON;

  	landing_cmd.request.min_pitch = 0;

  	landing_cmd.request.yaw = 0;

	//// 서비스 요청 메시지 전달 

	//// Landing

 	 while (ros::ok() )
  	{
		printf("send Landing command ...\n");

		if (!landing_client.call(landing_cmd))
		{
	 	 ros::spinOnce();
	 	 // rate.sleep();
		}
		else break;
  	
 	} 
 	ROS_ERROR("Landing command was sent\n");
}

bool srv_waypointAdd_cb(mission_lib::WaypointAdd::Request &req, mission_lib::WaypointAdd::Response &res)
{
	ROS_ERROR("WaypointAdd request received\n");

	// 웨이포인트 추가 
	
	mavros_msgs::Waypoint waypoint;
	
	waypoint.frame = req.frame;
	waypoint.command = req.command;
	waypoint.is_current = req.is_current;
	waypoint.autocontinue = req.autocontinue;
	waypoint.param1 = req.param1;
	waypoint.param2 = req.param2;
	waypoint.param3 = req.param3;
	waypoint.param4 = req.param4;
	waypoint.x_lat = req.x_lat;
	waypoint.y_long = req.y_long;
	waypoint.z_alt = req.z_alt;

	double distance_home;


	bool noflyZone_violation = false;

	switch (req.command)
	{
		case MAV_CMD_NAV_TAKEOFF:
			waypoint.x_lat = home_position.geo.latitude;
			waypoint.y_long = home_position.geo.longitude;

		break;


		case MAV_CMD_NAV_WAYPOINT:
		

		waypoint.frame = req.frame;
		waypoint.command = req.command;

		if (req.is_global) // 전역 좌표인 경우
		{

			// (2017.11.23) geofence 기능 추가 - 

			distance_home = distance(HOME_LAT, HOME_LON, waypoint.x_lat, waypoint.y_long, 'm');

			cout << "(" << waypoint.x_lat <<", " << waypoint.y_long << ")" << endl;
			cout <<"- home distance = " << distance_home << endl;
			
		//	distance_home = pow (waypoint.x_lat, 2.0) + pow (waypoint.y_long, 2.0);

		//	distance_home = sqrt(distance_home);
	


			// (2017.11.27) noflyZone 기능 추가 - 

			if (nofly_zone.isSet == true)
			{
				if ( (waypoint.x_lat >= nofly_zone.min_x_lat) && (waypoint.x_lat <= nofly_zone.max_x_lat) )
				{
					if ( (waypoint.y_long >= nofly_zone.min_y_long) && (waypoint.y_long <= nofly_zone.max_y_long) )
					{
						cout << "(" << waypoint.x_lat <<", " << waypoint.y_long << ")" << endl;
						noflyZone_violation = true;
					}
				}
		
			}

	
		}
		else
		{
	
			// (x, y, z) -> (lat, long, alt) 변환 필요
		}
	
		break;
		
		case MAV_CMD_NAV_LAND:

			waypoint.x_lat = home_position.geo.latitude;
			waypoint.y_long = home_position.geo.longitude;
		break;		


		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		

	
		break;

	}


	
	if (distance_home > Geofence_Radius)
	{	
		ROS_ERROR("WaypointAdd service rejected: new WP is out of the geofence boundary\n");
		res.value = false;
		
	}else if (noflyZone_violation == true)
	{
		ROS_ERROR("WaypointAdd service rejected: new WP is inside the nofly-Zone\n");
		res.value = false;
	}
	else
	{
		waypoints.push_back(waypoint);
	
		ROS_ERROR("WaypointAdd service accepted: new WP was added to the wp list.\n");
	
		res.value = true;
	}

	
	return true;
}


bool srv_waypointPush_cb(mission_lib::WaypointPush::Request &req, mission_lib::WaypointPush::Response &res)
{
	ROS_ERROR("WaypointPush request received\n");

	// 웨이포인트 업로드 메시지 설정 
	waypointPush_cmd.request.start_index = 0;
	waypointPush_cmd.request.waypoints = waypoints;

	//// 서비스 요청 메시지 전달

	//// waypointPush

	printf("send WaypointPush command ...\n");

	waypointPush_client.call(waypointPush_cmd);

 	ROS_ERROR("WaypointPush command was sent\n");
}

bool srv_waypointPull_cb(mission_lib::WaypointPull::Request &req, mission_lib::WaypointPull::Response &res)
{
	ROS_ERROR("WaypointPull request received\n");

	// do something ...
}

bool srv_goto_cb(mission_lib::Goto::Request &req, mission_lib::Goto::Response &res)
{

	double distance_home = 0;

	target_position.reached = false;
	cur_target.reached = false;

	target_position.is_global = req.is_global;
        target_position.target_seq_no = req.target_seq_no; 
        autonomous_flight = true;

	target_position.geofence_breach = false;

	ROS_ERROR("srv_goto_cb()");
	printf(": target_seq_no: %d\n", target_position.target_seq_no);


	ROS_ERROR("Goto request received\n");
	
	if (req.is_global) // 전역 좌표인 경우
	{
		//
		target_position.pos_global.latitude = req.x_lat;
		target_position.pos_global.longitude = req.y_long;
		target_position.pos_global.altitude = req.z_alt;

		target_pos_global.latitude = req.x_lat;
		target_pos_global.longitude = req.y_long;
		target_pos_global.altitude = req.z_alt;

	}
	else // 지역 좌표인 경우
	{
		//


		printf("target position: (%f, %f, %f)\n", target_pos_local.pose.position.x, target_pos_local.pose.position.y, target_pos_local.pose.position.z);
		

		distance_home = pow(req.x_lat, 2.0) + pow(req.y_long, 2.0);
		distance_home = sqrt(distance_home);

		if (distance_home > Geofence_Radius)
		{
			printf("goto service was rejected: the target is out of the geofence boundary\n");
			target_position.geofence_breach = true;
			res.value = false;
		}
		else
		{
			printf("goto the target: (%f, %f, %f)\n", req.x_lat, req.y_long, req.z_alt);
		//	target_position.reached = false;
		//	cur_target.reached = false;
			target_position.geofence_breach = false;
			
			target_position.pos_local.x = req.x_lat;
			target_position.pos_local.y = req.y_long;
			target_position.pos_local.z = req.z_alt;
		
			target_pos_local.pose.position.x = req.x_lat;
			target_pos_local.pose.position.y = req.y_long;
			target_pos_local.pose.position.z = req.z_alt;
			res.value = true;
		}
	}
//	sleep(10);
/*
	commandLong_cmd;// 무인기 제어에 사용될 서비스 선언

	ROS_ERROR("Goto request received\n");
	// target_system 설정
	//commandLong_cmd.request.target_system = 1;


	// target_component 설정
	//commandLong_cmd.request.target_component = 1; 

	// broadcast 설정

	commandLong_cmd.request.broadcast = true;
*/
/*	
	commandLong_cmd.request.command = MAV_CMD_NAV_LAND; 
	commandLong_cmd.request.param5 = req.x_lat;
	commandLong_cmd.request.param6 = req.y_long;
	commandLong_cmd.request.param7 = req.z_alt;
*/
/*	
	commandLong_cmd.request.command = MAV_CMD_NAV_LOITER_UNLIM;
	commandLong_cmd.request.param3 = 5;
	
	commandLong_cmd.request.param5 = req.x_lat;
	commandLong_cmd.request.param6 = req.y_long;
	commandLong_cmd.request.param7 = req.z_alt;
*/
/*	
	commandLong_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	commandLong_cmd.request.param5 = req.x_lat;
	commandLong_cmd.request.param6 = req.y_long;
	commandLong_cmd.request.param7 = req.z_alt;
	printf("send Goto command ...\n");
	commandLong_client.call(commandLong_cmd); // "
 	ROS_ERROR("Goto command was sent\n");
	
	// 요청 메시지 필드 설정

	// 요청 메시지 전달

*/ 
	return true;
}

bool srv_geofence_cb(mission_lib::Geofence::Request &req, mission_lib::Geofence::Response &res)
{
	Geofence_Radius = req.radius;

	switch (req.action)
	{
	case Warning:
		
		geofence_policy = Warning;	
		break;

	case RTL:
		geofence_policy = RTL;
		break;
	
	case Loiter:
		geofence_policy = Loiter;
		break;

	case Landing:

		geofence_policy = Landing;
		break;

	default:
		geofence_policy = RTL;
		break;
	}	
	return true;
}

bool srv_noflyZone_cb(mission_lib::Noflyzone::Request &req, mission_lib::Noflyzone::Response &res)
{
	ROS_INFO ("nofly_zone service was called:\n");
	
	nofly_zone.isSet = true;

	nofly_zone.min_x_lat = req.min_x_lat;
		
	nofly_zone.min_y_long = req.min_y_long;

	nofly_zone.max_x_lat = req.max_x_lat;

	nofly_zone.max_y_long = req.max_y_long;


	cout << "x_lat: " << nofly_zone.min_x_lat << " ~ " << nofly_zone.max_x_lat << endl;

	cout << "y_long: " << nofly_zone.min_y_long << " ~ " << nofly_zone.max_y_long << endl;


	return true;

}


int main(int argc, char** argv)

{

	ros::init(argc, argv, "mission_lib_node");
 	ros::NodeHandle nh; 
	
	ros::Rate rate (20.0); 
	
	// stream rate 설정
	printf("setting stream rate!!\n");
	ros::ServiceClient client = nh.serviceClient<mavros_msgs::StreamRate>("mavros/set_stream_rate");
	mavros_msgs::StreamRate srv;
	srv.request.stream_id = 0;
	srv.request.on_off = 1;

	int retry_cnt = 0;
	while (ros::ok() && retry_cnt <100)
	{
		retry_cnt++;

		 if (client.call(srv))
       		 {
			ROS_ERROR("Service called");
			break;	
		}
		else	
       		{
       		         ROS_ERROR("Failed to call service");
			rate.sleep();	
        	}	
	}

	// publisher 초기화
//	state_pub = nh.advertise<mavros_msgs::State>("/mavros/state", 10);
	pos_pub_local = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	pos_pub_global = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);
	cur_target_pub = nh.advertise<mission_lib::Target>("mission_lib/current_target", 10);

	// subscriber 초기화
	state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
	pos_sub_local = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose",10,  pos_cb_local); 
	pos_sub_global = nh.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global",10,  pos_cb_global); 
	home_sub = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, homePosition_cb);
	wpList_sub = nh.subscribe<mavros_msgs::WaypointList> ("mavros/mission/waypoints", 10, wpList_cb);

	//// 서비스 서버 선언
	
	
	arming_srv_server = nh.advertiseService("srv_arming", srv_arming_cb);
	takeoff_srv_server = nh.advertiseService("srv_takeoff", srv_takeoff_cb);
	landing_srv_server = nh.advertiseService("srv_landing", srv_landing_cb);
	waypointAdd_srv_server = nh.advertiseService("srv_waypointAdd", srv_waypointAdd_cb);
	waypointPush_srv_server = nh.advertiseService("srv_waypointPush", srv_waypointPush_cb);
	waypointPull_srv_server = nh.advertiseService("srv_waypointPull", srv_waypointPull_cb);
	goto_srv_server = nh.advertiseService("srv_goto", srv_goto_cb); 
	geofence_srv_server = nh.advertiseService("srv_geofence", srv_geofence_cb);		
	noflyzone_srv_server = nh.advertiseService("srv_noflyZone", srv_noflyZone_cb);	

	//// 서비스 클라이언트 선언

        arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming"); // service client 선언
	takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/takeoff"); // 서비스 클라이언트 선언
	landing_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/land"); // 서비스 클라이언트 선언
	waypointPush_client = nh.serviceClient<mavros_msgs::WaypointPush> ("mavros/mission/push"); // 서비스 클라이언트 선언
	waypointPull_client = nh.serviceClient<mavros_msgs::WaypointPull> ("mavros/mission/pull"); // 서비스 클라이언트 선언
	commandLong_client = nh.serviceClient<mavros_msgs::CommandLong> ("mavros/cmd/command");	

	modeChange_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
	target_position.reached = false; // 목적지 도착 여부를 false로 초기화
	
	while ( ros::ok() )
	{
		VehicleState cState = vehicle.getState();

		if (cState.flight_mode.compare("AUTO.RTL") == 0) // RTL 모드일 경우 바로 종료
		{
			break;
		}

		// 현재 목적지 정보 publish
		
		// 1) Target 토픽 publish (응용 프로그램 측에 현재 진행 중인 target 위치 및 도착 여부 알림)

		// 2) 위치 이동을 위한 setpoint_position/local 또는 setpoint_raw/global 토픽 출판

		if (autonomous_flight == true && target_position.geofence_breach == false)
		{

			cur_target.target_seq_no = target_position.target_seq_no;
		
			cur_target.is_global = target_position.is_global;

			if (target_position.is_global == false)
			{
				cur_target.x_lat = target_position.pos_local.x;
			
				cur_target.y_long = target_position.pos_local.y;

				cur_target.z_alt = target_position.pos_local.z;
			}

			else
			{
				cur_target.x_lat  = target_position.pos_global.latitude;
				cur_target.y_long  = target_position.pos_global.longitude;
				cur_target.z_alt  = target_position.pos_global.altitude;
			}
		
			cur_target.reached = target_position.reached;


			cur_target_pub.publish(cur_target);

			// 현재 목적지로 이동

			if (target_position.reached == false)
			{
				if (target_position.is_global == false)
				{
				//	pos_pub_local.
					ROS_INFO("Moving to the target point (local coord.)");
				/*
				printf("target position: (%f, %f, %f)\n", target_position.pos_local.x,
				 target_position.pose_local.y, target_position.pos_local.z);
				*/
					printf("target position: (%f, %f, %f)\n", target_pos_local.pose.position.x,
					 target_pos_local.pose.position.y, target_pos_local.pose.position.z);
				
					pos_pub_local.publish(target_pos_local); // 목적지 위치 정보 (지역 좌표) 출판 - type: geometry_msgs::poseStamped
				}

				else
				{
				//	pos_pub_global

					ROS_INFO("Moving to the target point (global coord)");
					/*printf("target position: (%f, %f, %f)\n", target_position.pos_global.latitude, 
					target_position.pos_global.longitude, 	target_position.pos_global.altitude);*/
 					//mavros_msgs::GlobalPositionTarget target_pos_global; // 목적지 위치 정보 (전역 좌표)
				
					printf("target position: (%f, %f, %f)\n", target_pos_global.latitude, 
					target_pos_global.longitude, 	target_pos_global.altitude);

					pos_pub_global.publish(target_pos_global);// 목적지 위치 정보 (전역 좌표 출판) - type: mavros_msgs/GlobalPositionTarget
				}

					modeChange_cmd.request.base_mode = 0;
					modeChange_cmd.request.custom_mode = "OFFBOARD";
					modeChange_client.call(modeChange_cmd);
			}
		  	
			//ros::spinOnce();
      		  	//rate.sleep();

			//printf("home position: (%f, %f, %f) \n", home_position.position.x, home_position.position.y, home_position.position.z);	
	
		}// if (autonomous_flight)

			ros::spinOnce();
      		  	rate.sleep();

		ref_system_conversion_test();
	}

	return 0;
}
