
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
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mission_lib_msgs/Arming.h" // 시동 서비스 헤더 파일 포함                                                 
#include "mission_lib_msgs/Takeoff.h" // 이륙 서비스 
#include "mission_lib_msgs/Landing.h" // 착륙 서비스 	"
#include "mission_lib_msgs/MissionAddItem.h" // 미션 아이템 추가 서비스 헤더 파일 포함
#include "mission_lib_msgs/MissionUpload.h" // 미션 업로드 서비스 헤더 파일 포함
#include "mission_lib_msgs/MissionDownload.h" // 미션 다운로드 서비스 헤더 파일 포함
#include "mission_lib_msgs/MissionClear.h" // 미션 제거 서비스 헤더 파일 포함
#include "mission_lib_msgs/CheckHome.h"  // 홈 위치 확인 서비스 헤더 파일 포함

#include "mission_lib_msgs/Geofence.h" // Geofence 서비스 헤더 파일
#include "mission_lib_msgs/NoflyZone.h" // Noflyzone 서비스 헤더 파일
#include "mission_lib_msgs/CheckNFZone.h" // NoflyZone 확인 서비스 헤더 파일





using namespace std;
using namespace mavros_msgs;
using namespace geographic_msgs;
using namespace geometry_msgs;

//// 메시지 변수 선언
GeoPoint home_position;



// (웨이포인트 요청 메시지 수신(응용프로그램)  및 송신 (mavros) 목적
vector<mavros_msgs::Waypoint> waypoints; // 웨이포인트 정보
mavros_msgs::WaypointList waypointList; // 웨이포인트 목록


//// 서비스 요청 메시지 선언 (mavros)

mavros_msgs::WaypointPush waypointPush_cmd; // 미션 업로드 요청 메시지
mavros_msgs::WaypointPull waypointPull_cmd; // 미션 다운로드 요청 메시지
mavros_msgs::WaypointClear waypointClear_cmd; // 미션 제거 요청 메시지
mavros_msgs::SetMode modeChange_cmd;  

mission_lib_msgs::CheckHome checkHome_cmd; // home 위치 확인 서비스 요청 메시지
mission_lib_msgs::CheckNFZone checkNFZone_cmd; // noflyZone 확인 서비스 요청 메시지	
// publisher 선언



// subscriber 선언

ros::Subscriber wpList_sub;

// 서비스 서버 선언
ros::ServiceServer missionAddItem_srv_server;
ros::ServiceServer missionUpload_srv_server;
ros::ServiceServer missionDownload_srv_server;
ros::ServiceServer missionClear_srv_server;


// 서비스 클라이언트 선언
ros::ServiceClient waypointAdd_client; // 	"
ros::ServiceClient waypointPush_client; // 	"
ros::ServiceClient waypointPull_client; // 	"
ros::ServiceClient waypointClear_client; //
ros::ServiceClient commandLong_client; // "
ros::ServiceClient modeChange_client; // 모드 변경 서비스 클라이언트
ros::ServiceClient checkHome_client; // home 위치 확인 서비스 클라이언트 
ros::ServiceClient checkNFZone_client; // noflyZone 확인 서비스 클라이언트



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

void wpList_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
	printf("wpList_cb(): waypointList was received \n");	
	waypointList = *msg;
	//print_waypoints(waypointList.waypoints);
}



// callback 함수 (서비스 제공) 정의


bool srv_missionAddItem_cb(mission_lib_msgs::MissionAddItem::Request &req, mission_lib_msgs::MissionAddItem::Response &res)
{
	ROS_ERROR("MissionAddItem request received\n");

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

	checkNFZone_cmd.request.value = true;

	if ( checkNFZone_client.call(checkNFZone_cmd)==true)

	{
		cout << " checkNFZone service was called " << endl;

		if (checkNFZone_cmd.response.isSet == true)
		{
			double min_x_lat = checkNFZone_cmd.response.min_x_lat;
			double min_y_long = checkNFZone_cmd.response.min_y_long;
			double max_x_lat = checkNFZone_cmd.response.max_x_lat;
			double max_y_long = checkNFZone_cmd.response.max_y_long;


			if( ( min_x_lat <= waypoint.x_lat  ) && ( waypoint.x_lat <= max_x_lat ) )
			{
				if( ( min_y_long <= waypoint.y_long  ) && ( waypoint.y_long <= max_y_long ) )
				{
					cout << "noFlyZone_viloation!! " << endl;
					noflyZone_violation = true;
				} 
			}
		}

	}

	

	

	switch (req.command)
	{
		case MAV_CMD_NAV_TAKEOFF:

		waypoint.x_lat = home_position.latitude;
		waypoint.y_long = home_position.longitude;

		break;


		case MAV_CMD_NAV_WAYPOINT:
		

		waypoint.frame = req.frame;
		waypoint.command = req.command;

		if (req.is_global) // 전역 좌표인 경우
		{

			// (2017.11.23) geofence 기능 추가 - 
		/*
			distance_home = distance(HOME_LAT, HOME_LON, waypoint.x_lat, waypoint.y_long, 'm');

			cout << "(" << waypoint.x_lat <<", " << waypoint.y_long << ")" << endl;
			cout <<"- home distance = " << distance_home << endl;
			
		//	distance_home = pow (waypoint.x_lat, 2.0) + pow (waypoint.y_long, 2.0);

		//	distance_home = sqrt(distance_home);
	

		*/
			// (2017.11.27) noflyZone 기능 추가 - 
		
		/*
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

		*/
		}
		else
		{
	
			// (x, y, z) -> (lat, long, alt) 변환 필요
		}
	
		break;
		
		case MAV_CMD_NAV_LAND:

			waypoint.x_lat = home_position.latitude;
			waypoint.y_long = home_position.longitude;
		break;		


		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		

	
		break;

	}


	/*	
	if (distance_home > Geofence_Radius)
	{	
		ROS_ERROR("WaypointAdd service rejected: new WP is out of the geofence boundary\n");
		res.value = false;
		
	}else
		*/
	 if (noflyZone_violation == true)
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


bool srv_missionUpload_cb(mission_lib_msgs::MissionUpload::Request &req, mission_lib_msgs::MissionUpload::Response &res)
{
	ROS_ERROR("MissionAddItem request received\n");

	// 웨이포인트 업로드 메시지 설정 
	waypointPush_cmd.request.start_index = 0;
	waypointPush_cmd.request.waypoints = waypoints;

	//// 서비스 요청 메시지 전달

	//// waypointPush

	printf("send WaypointPush command ...\n");

	waypointPush_client.call(waypointPush_cmd);

 	ROS_ERROR("WaypointPush command was sent\n");
}

bool srv_missionDownload_cb(mission_lib_msgs::MissionDownload::Request &req, mission_lib_msgs::MissionDownload::Response &res)
{
	ROS_ERROR("MissionDownload request received\n");

	// do something ...

	res.waypoints = waypointList.waypoints;
}


bool srv_missionClear_cb(mission_lib_msgs::MissionClear::Request &req, mission_lib_msgs::MissionClear::Response &res)
{
	ROS_ERROR("MIssionClear request received\n");
	waypoints.clear();
	waypointClear_client.call(waypointClear_cmd);
	
	ROS_ERROR("WaypointClear command was sent\n");
	// 

}

int main(int argc, char** argv)

{

	ros::init(argc, argv, "mission_lib_mission");
 	ros::NodeHandle nh; 
	
	ros::Rate rate (20.0); 
	


	// subscriber 초기화
	wpList_sub = nh.subscribe<mavros_msgs::WaypointList> ("mavros/mission/waypoints", 10, wpList_cb);

	//// 서비스 서버 선언
	
	
	missionAddItem_srv_server = nh.advertiseService("srv_missionAddItem", srv_missionAddItem_cb);
	missionUpload_srv_server = nh.advertiseService("srv_missionUpload", srv_missionUpload_cb);
	missionDownload_srv_server = nh.advertiseService("srv_missionDownload", srv_missionDownload_cb);
	missionClear_srv_server = nh.advertiseService("srv_missionClear", srv_missionClear_cb);	


	//// 서비스 클라이언트 초기화

	waypointPush_client = nh.serviceClient<mavros_msgs::WaypointPush> ("mavros/mission/push"); // 서비스 클라이언트 선언
	waypointPull_client = nh.serviceClient<mavros_msgs::WaypointPull> ("mavros/mission/pull"); // 서비스 클라이언트 선언
	waypointClear_client = nh.serviceClient<mavros_msgs::WaypointClear> ("mavros/mission/clear"); //
	modeChange_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
	
	checkHome_client = nh.serviceClient<mission_lib_msgs::CheckHome> ("srv_chkHome"); 
	checkNFZone_client = nh.serviceClient<mission_lib_msgs::CheckNFZone> ("srv_checkNFZone");

	while ( ros::ok() )
	{

			ros::spinOnce();
      		  	rate.sleep();

			checkHome_cmd.request.value = true;
			
			if (checkHome_client.call (checkHome_cmd) == true)
			{
			//	ROS_INFO("checkHome service called...");

				home_position.latitude = checkHome_cmd.response.latitude;
				home_position.longitude = checkHome_cmd.response.longitude;
				home_position.altitude = checkHome_cmd.response.altitude;
				
			}
	/*		

			cout << "mission_lib_mission: " ;
			printf(" home position: (%lf, %lf, %lf) \n", home_position.latitude, home_position.longitude, home_position.altitude);	
	
	*/
	}	
	return 0;
}
