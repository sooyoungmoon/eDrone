
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <eDrone_msgs/Arming.h> // 시동 서비스 헤더 파일 포함                                                 
#include <eDrone_msgs/Takeoff.h> // 이륙 서비스 
#include <eDrone_msgs/Landing.h> // 착륙 서비스 	"
#include <eDrone_msgs/MissionAddItem.h> // 미션 아이템 추가 서비스 헤더 파일 포함
#include <eDrone_msgs/MissionUpload.h> // 미션 업로드 서비스 헤더 파일 포함
#include <eDrone_msgs/MissionDownload.h> // 미션 다운로드 서비스 헤더 파일 포함
#include <eDrone_msgs/MissionClear.h> // 미션 제거 서비스 헤더 파일 포함
#include <eDrone_msgs/CheckHome.h>  // 홈 위치 확인 서비스 헤더 파일 포함

#include <eDrone_msgs/GeofenceSet.h> // 가상 울타리 설정 
#include <eDrone_msgs/GeofenceReset.h> // 가상 울타리 해제 
#include <eDrone_msgs/GeofenceCheck.h> // 가상 울타리 확인
//#include <eDrone_msgs/NoflyZone.h> // Noflyzone 서비스 헤더 파일
//#include <eDrone_msgs/CheckNFZone.h> // NoflyZone 확인 서비스 헤더 파일


#include <eDrone_msgs/NoflyZoneSet.h> // 비행 금지 구역 설정
#include <eDrone_msgs/NoflyZoneReset.h> // 비행 금지 구역 해제
#include <eDrone_msgs/NoflyZoneCheck.h> // 비행 금지 구역 확인

#include <eDrone_lib/GeoUtils.h> // 좌표 변환 라이브러리 헤더 파일 





using namespace std;
using namespace mavros_msgs;
using namespace geographic_msgs;
using namespace geometry_msgs;


// node handle 포인터 변수

ros::NodeHandle* nh_ptr; 


//// 메시지 변수 선언
//GeoPoint home_position;
mavros_msgs::HomePosition home_position; // Home 위치 획득에 필요한 메시지 변수

// (웨이포인트 요청 메시지 수신(응용프로그램)  및 송신 (mavros) 목적
vector<mavros_msgs::Waypoint> waypoints; // 웨이포인트 정보
mavros_msgs::WaypointList waypointList; // 웨이포인트 목록


//// 서비스 요청 메시지 선언 (mavros)

mavros_msgs::WaypointPush waypointPush_cmd; // 미션 업로드 요청 메시지
mavros_msgs::WaypointPull waypointPull_cmd; // 미션 다운로드 요청 메시지
mavros_msgs::WaypointClear waypointClear_cmd; // 미션 제거 요청 메시지
mavros_msgs::SetMode modeChange_cmd;  

eDrone_msgs::CheckHome checkHome_cmd; // home 위치 확인 서비스 요청 메시지
//eDrone_msgs::CheckNFZone checkNFZone_cmd; // noflyZone 확인 서비스 요청 메시지	
// publisher 선언



// subscriber 선언

ros::Subscriber wpList_sub;
ros::Subscriber home_sub;

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
//ros::ServiceClient checkNFZone_client; // noflyZone 확인 서비스 클라이언트
ros::ServiceClient noflyZone_client; // 비행금지구역 확인 서비스 클라이언트

// Home 위치 변수
float HOME_LAT;
float HOME_LON;
float HOME_ALT;

void homePosition_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
	static int print_cnt = 0;

        home_position = *msg;

//      printf("home position: (%f, %f, %f) \n", home_position.position.x, home_position.position.y, home_position.position.z);

        HOME_LAT = home_position.geo.latitude;
        HOME_LON = home_position.geo.longitude;
        HOME_ALT = home_position.geo.altitude;

	if (print_cnt < 10)
	{
        	ROS_INFO( "eDrone_utility_node: home position: (%lf, %lf, %lf) \n", HOME_LAT,  HOME_LON, HOME_ALT); 
		print_cnt++;
	}
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

void wpList_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
	printf("wpList_cb(): waypointList was received \n");	
	waypointList = *msg;
	//print_waypoints(waypointList.waypoints);
}



// callback 함수 (서비스 제공) 정의


bool srv_missionAddItem_cb(eDrone_msgs::MissionAddItem::Request &req, eDrone_msgs::MissionAddItem::Response &res)
{
	ROS_INFO("eDrone_utility_node: MissionAddItem request received\n");

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

/*
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

*/	

	

	switch (req.command)
	{
		case MAV_CMD_NAV_TAKEOFF:

		//waypoint.x_lat = home_position.latitude;
		//waypoint.y_long = home_position.longitude;
		waypoint.x_lat = HOME_LAT;
		waypoint.y_long = HOME_LON;
		waypoint.z_alt = req.z_alt;
		break;		

		case MAV_CMD_NAV_WAYPOINT:
		waypoint.frame = req.frame;
		waypoint.command = req.command;
		waypoint.z_alt = req.z_alt;

		if (req.is_global) // 전역 좌표인 경우
		{
			;
		}
		else // 지역 좌표인 경우 
		{
			;
			// (x, y, z) -> (lat, long, alt) 변환 필요

			GeoPoint geoPoint = convertENUToGeo( waypoint.x_lat, waypoint.y_long, waypoint.z_alt, HOME_LAT, HOME_LON, HOME_ALT);

			waypoint.x_lat = geoPoint.latitude;
			waypoint.y_long = geoPoint.longitude;
			//waypoint.z_alt = geoPoint.altitude;

		}
	
		break;
		
		case MAV_CMD_NAV_LAND:

			waypoint.x_lat = HOME_LAT;
			waypoint.y_long = HOME_LON;
		break;		


		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		

	
		break;

	}


	/*	
	if (distance_home > Geofence_Radius)
	{	
		ROS_INFO("WaypointAdd service rejected: new WP is out of the geofence boundary\n");
		res.value = false;
		
	}else
		*/

        /* 비행 금지 구역 check */

	bool noflyZone_violation = false;

	eDrone_msgs::NoflyZoneCheck noflyZoneCheck_cmd; // noflyZone 확인 서비스 요청 메시지	

	ros::ServiceClient noflyZoneCheck_client = nh_ptr->serviceClient<eDrone_msgs::NoflyZoneCheck>("srv_noflyZoneCheck" );; // noflyZone 확인 서비스 클라이언트 


	noflyZoneCheck_cmd.request.ref_system = "WGS84";

	noflyZoneCheck_cmd.request.arg1 = waypoint.x_lat;
	noflyZoneCheck_cmd.request.arg2 = waypoint.y_long;	

	ROS_INFO("eDrone_utility_node: trying to call NoflyZoneCheck service... ");
	
		if (noflyZoneCheck_client.call (noflyZoneCheck_cmd)==true)
		{
			if (noflyZoneCheck_cmd.response.violation == true)
			{
				noflyZone_violation = true; // 비행 금지 구역 위반 
				ROS_INFO ("eDrone_utility_node: missionAddItem service rejected: noflyZone violation!");
				res.value = false;
				return true;
			}
			else
			{
				noflyZone_violation = false; 
				//res.value = true;
			}
	
		}
	
	/* Geofence check */

	bool geofence_violation = false;
	
	eDrone_msgs::GeofenceCheck geofenceCheck_cmd;
	ros::ServiceClient geofenceCheck_client = nh_ptr-> serviceClient<eDrone_msgs::GeofenceCheck> ("srv_geofenceCheck"); // geofence 확인 서비스 클라이언트

	geofenceCheck_cmd.request.ref_system = "WGS84";
	geofenceCheck_cmd.request.arg1= waypoint.x_lat;
        geofenceCheck_cmd.request.arg2= waypoint.y_long;
		
	ROS_INFO("eDrone_utility_node: trying to call GeofenceCheck service");

	if (geofenceCheck_client.call (geofenceCheck_cmd) == true)
	{

		if (geofenceCheck_cmd.response.value == true )
		{
			if (geofenceCheck_cmd.response.violation == true)
			{
				geofence_violation = true;				
				ROS_INFO("eDrone_utility_node: missionAddItem service rejected: geofence violation!\n");
				res.value = false;
				return true;

			}
			else
			{
				geofence_violation = false; 
				
			}
		
		}
		
	}

	 if (noflyZone_violation != true && geofence_violation!=true)
	{	
		waypoints.push_back(waypoint);	
		ROS_INFO("eDrone_utility_node: missionAddItem service accepted: new WP was added to the wp list.\n");
		res.value = true;

	}

	
	return true;
}


bool srv_missionUpload_cb(eDrone_msgs::MissionUpload::Request &req, eDrone_msgs::MissionUpload::Response &res)
{
	ROS_INFO("MissionUpload request received\n");

	// 웨이포인트 업로드 메시지 설정 
	waypointPush_cmd.request.start_index = 0;
	waypointPush_cmd.request.waypoints = waypoints;

	//// 서비스 요청 메시지 전달

	//// waypointPush

	printf("send WaypointPush command ...\n");

	waypointPush_client.call(waypointPush_cmd);

 	ROS_INFO("WaypointPush command was sent\n");
}

bool srv_missionDownload_cb(eDrone_msgs::MissionDownload::Request &req, eDrone_msgs::MissionDownload::Response &res)
{
	ROS_INFO("MissionDownload request received\n");

	// do something ...

	res.waypoints = waypointList.waypoints;
}


bool srv_missionClear_cb(eDrone_msgs::MissionClear::Request &req, eDrone_msgs::MissionClear::Response &res)
{
	ROS_INFO("MIssionClear request received\n");
	waypoints.clear();
	waypointClear_client.call(waypointClear_cmd);
	
	ROS_INFO("WaypointClear command was sent\n");
	// 

}

int main(int argc, char** argv)

{

	ros::init(argc, argv, "eDrone_msgs");
 	ros::NodeHandle nh; 

	nh_ptr = &nh;
	
	ros::Rate rate (20.0); 
	


	// subscriber 초기화
	wpList_sub = nh.subscribe<mavros_msgs::WaypointList> ("mavros/mission/waypoints", 10, wpList_cb);
	home_sub = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, homePosition_cb);



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
	
//	checkHome_client = nh.serviceClient<eDrone_msgs::CheckHome> ("srv_chkHome"); 
//	checkNFZone_client = nh.serviceClient<eDrone_msgs::CheckNFZone> ("srv_checkNFZone");

	while ( ros::ok() )
	{

			ros::spinOnce();
      		  	rate.sleep();
			/*
			checkHome_cmd.request.value = true;
			
			if (checkHome_client.call (checkHome_cmd) == true)
			{
			//	ROS_INFO("checkHome service called...");

				home_position.latitude = checkHome_cmd.response.latitude;
				home_position.longitude = checkHome_cmd.response.longitude;
				home_position.altitude = checkHome_cmd.response.altitude;
				
			}
			*/
	}	
	return 0;
}
