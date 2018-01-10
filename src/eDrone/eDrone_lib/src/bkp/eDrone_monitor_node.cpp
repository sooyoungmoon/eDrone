
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
#include <mavros_msgs/GlobalPositionTarget.h>
#include "Vehicle.h"
#include "GeoInfo.h"
#include <eDrone_msgs/Arming.h> // 시동 서비스 헤더 파일 포함                                                 
#include <eDrone_msgs/Takeoff.h> // 이륙 서비스 
#include <eDrone_msgs/Landing.h> // 착륙 서비스 	"
#include <eDrone_msgs/CheckState.h> // 무인기 상태 확인 서비스
#include <eDrone_msgs/CheckPosition.h> // 무인기 위치 확인 서비스
#include <eDrone_msgs/CheckHome.h> // home 위치 확인 서비스




using namespace std;
using namespace Mission_API; 
using namespace mavros_msgs;
using namespace sensor_msgs;

//// Vehicle 객체 선언

Vehicle* vehicle = Vehicle::getInstance();
//GeoInfo* geoInfo = GeoInfo::getInstance();


//// 메시지 변수 선언

// (무인기 상태 정보 수신 목적)
mavros_msgs::State current_state; // 무인기 상태 정보

// (무인기 위치 확인 목적)
geometry_msgs::PoseStamped current_pos_local; // 현재 위치 및 자세 정보 (지역 좌표)
sensor_msgs::NavSatFix current_pos_global; // 현재 위치 정보 (전역 좌표)


// (홈 위치 획득 목적)
//mavros_msgs::HomePosition home_position; // home position




//// 서비스 요청 메시지 선언 (mavros)

mavros_msgs::CommandBool arm_cmd; // 시동 명령에 사용될 서비스 선언 
mavros_msgs::CommandTOL takeoff_cmd; // 이륙 명령에 사용될 서비스 선언 
mavros_msgs::CommandTOL landing_cmd; // 착륙 명령에 사용될 서비스 선언 

// publisher 선언



// subscriber 선언

ros::Subscriber state_sub;
ros::Subscriber pos_sub_local;
ros::Subscriber pos_sub_global;
ros::Subscriber home_sub;

// 서비스 서버 선언
ros::ServiceServer arming_srv_server;
ros::ServiceServer takeoff_srv_server;
ros::ServiceServer landing_srv_server;
ros::ServiceServer chkState_srv_server;
ros::ServiceServer chkPosition_srv_server;
ros::ServiceServer chkHome_srv_server;

//서비스 클라이언트 선언
ros::ServiceClient arming_client; // 서비스 클라이언트 선언
ros::ServiceClient takeoff_client; // 서비스 클라이언트 선언
ros::ServiceClient landing_client; // 서비스 클라이언트 선언


// home position


 double HOME_LAT ;
 double HOME_LON;
 double HOME_ALT;




void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;

	vehicle->setState (current_state);

	VehicleState cState = vehicle->getState();


	if (cState.flight_mode.compare("AUTO.RTL") ==0)
	{
		ROS_INFO("state_cb(): FLIGHT MODE = RTL");
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

	GeoInfo&  geoInfo = GeoInfo::getInstance();

	current_pos_local = *msg;

	double DIST_RANGE = 0.5;	

//	vehicle->setLocalPosition(current_pos_local.pose.position.x, current_pos_local.pose.position.y, current_pos_local.pose.position.z);

//					printf("current_position: (%f, %f, %f \n)", current_pos_local.pose.position.x, current_pos_local.pose.position.y, current_pos_local.pose.position.z);	

//	PoseStamped temp_pose = *msg;


	geoInfo.setLocalPosition(msg->pose.position);

	
}

void pos_cb_global(const sensor_msgs::NavSatFix::ConstPtr& msg){

	

	current_pos_global = *msg;

//	vehicle->setGlobalPosition(current_pos_global.latitude, current_pos_global.longitude, current_pos_global.altitude);

//					printf("current_position: (%f, %f, %f \n)", current_pos_global.latitude, current_pos_global.longitude, current_pos_global.altitude-HOME_ALT);



	GeoInfo& geoInfo = GeoInfo::getInstance();	
	GeoPoint temp_geo;

	temp_geo.latitude = msg->latitude;
	temp_geo.longitude = msg->longitude;
	temp_geo.altitude = msg->altitude;
		
	geoInfo.setGlobalPosition (temp_geo);

}



void homePosition_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
	GeoInfo& geoInfo = GeoInfo::getInstance();	

	geoInfo.setHomePosition(msg->geo);

	GeoPoint home = geoInfo.getHomePosition(); 

	HOME_LAT = home.latitude;
	HOME_LON = home.longitude;		
	HOME_ALT = home.altitude;
	
	printf("home position: (%f, %f, %f) \n", home.latitude, home.longitude, home.altitude);	
}

// callback 함수 (서비스 제공) 정의

bool srv_arming_cb(eDrone_msgs::Arming::Request &req, eDrone_msgs::Arming::Response &res )
{

	ROS_INFO("ARMing request received\n");
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
	 
	ROS_INFO("ARMing command was sent\n");

}


bool srv_takeoff_cb(eDrone_msgs::Takeoff::Request &req, eDrone_msgs::Takeoff::Response &res)
{

	ROS_INFO("Takeoff request received\n");
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

 	 ROS_INFO("Takeoff command was sent\n");
}

bool srv_landing_cb(eDrone_msgs::Landing::Request &req, eDrone_msgs::Landing::Response &res)
{
	
	ROS_INFO("Landing request received\n");
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
 	ROS_INFO("Landing command was sent\n");
}

bool srv_chkState_cb(eDrone_msgs::CheckState::Request &req, eDrone_msgs::CheckState::Response &res)
{
	
	VehicleState cState = vehicle->getState();

	res.armed = cState.armed;
	res.connected = cState.connected;
	res.mode.assign(cState.flight_mode);

	res.value = true;	

	return true;

}

bool srv_chkPosition_cb(eDrone_msgs::CheckPosition::Request &req, eDrone_msgs::CheckPosition::Response &res)
{
	GeoInfo& geoInfo = GeoInfo::getInstance();
	GeoPoint geoPoint = geoInfo.getGlobalPosition();

	Point point = geoInfo.getLocalPosition();

	res.latitude = geoPoint.latitude;
	res.longitude = geoPoint.longitude;
	res.altitude = geoPoint.altitude;

	res.x = point.x;
	res.y = point.y;
	res.z = point.z;

	res.value = true;
	
	return true;
}

bool srv_chkHome_cb(eDrone_msgs::CheckHome::Request& req, eDrone_msgs::CheckHome::Response& res)
{
	GeoInfo& geoInfo = GeoInfo::getInstance();
	GeoPoint home_position = geoInfo.getHomePosition();

	res.latitude = home_position.latitude;
	res.longitude = home_position.longitude;
	res.altitude = home_position.altitude;

	res.value = true;
	return true;
}

int main(int argc, char** argv)

{

	ros::init(argc, argv, "eDrone_monitor_node");
 	ros::NodeHandle nh; 
	
	ros::Rate rate (20.0); 
	

	// publisher 초기화

	// subscriber 초기화
	state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
	pos_sub_local = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose",10,  pos_cb_local); 
	pos_sub_global = nh.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global",10,  pos_cb_global); 
	home_sub = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, homePosition_cb);

	//// 서비스 서버 선언
	
	
	arming_srv_server = nh.advertiseService("srv_arming", srv_arming_cb);
	takeoff_srv_server = nh.advertiseService("srv_takeoff", srv_takeoff_cb);
	landing_srv_server = nh.advertiseService("srv_landing", srv_landing_cb);
	chkState_srv_server = nh.advertiseService("srv_chkState", srv_chkState_cb);
	chkPosition_srv_server = nh.advertiseService("srv_chkPosition", srv_chkPosition_cb);
	chkHome_srv_server = nh.advertiseService("srv_chkHome", srv_chkHome_cb);

	//// 서비스 클라이언트 선언

        arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming"); // service client 선언
	takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/takeoff"); // 서비스 클라이언트 선언
	landing_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/land"); // 서비스 클라이언트 선언
	
	while ( ros::ok() )
	{
		VehicleState cState = vehicle->getState();
	
	
		ros::spinOnce();
      		 rate.sleep();

		GeoInfo& geoInfo = GeoInfo::getInstance();
		GeoPoint home = geoInfo.getHomePosition();
/*
		cout << "eDrone_monitor_node: " ;
		printf(" home position: (%lf, %lf, %lf) \n", home.latitude, home.longitude, home.altitude);	
*/
	}


	return 0;
}
