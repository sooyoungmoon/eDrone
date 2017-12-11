
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
#include "mavros_msgs/GlobalPositionTarget.h"
#include "Vehicle.h"
#include "GeoInfo.h"
#include "mission_lib_msgs/Arming.h" // 시동 서비스 헤더 파일
#include "mission_lib_msgs/Goto.h" // 무인기 위치 이동 서비스 헤더 파일 포함
#include "mission_lib_msgs/ModeChange.h" // 비행 모드 변경 서비스 헤더 파일
#include "mission_lib_msgs/RTL.h" // RTL
#include "mission_lib_msgs/Target.h" // 현재 목적지 topic 메시지가 선언된 헤더 파일 포함
#include "mission_lib_msgs/Geofence.h" // Geofence 서비스 헤더 파일
#include "mission_lib_msgs/NoflyZone.h" // Noflyzone 서비스 헤더 파일
#include "mission_lib_msgs/CheckNFZone.h" // CheckNFZone 서비스 헤더 파일 (noflyZone 확인)
#include "GeoUtils.h"





using namespace std;
using namespace Mission_API; 
using namespace mavros_msgs;


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



// (무인기 위치 이동 목적)
geometry_msgs::PoseStamped target_pos_local; // 목적지 위치 정보 (지역 좌표)
mavros_msgs::GlobalPositionTarget target_pos_global; // 목적지 위치 정보 (전역 좌표)
mission_lib_msgs::Target cur_target; // 현재 목적지 정보 (publisher: mission_lib_node, subscriber: 응용 프로그램)



//// 서비스 요청 메시지 선언 (mavros)
mavros_msgs::CommandBool arming_cmd;
mavros_msgs::CommandLong commandLong_cmd;// 무인기 제어에 사용될 서비스 선언
mavros_msgs::SetMode modeChange_cmd; // 모드 변경에 사용될 서비스 요청 메시지
mavros_msgs::SetMode rtl_cmd; // 복귀 명령에 사용될 서비스 요청 메시지
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

// 서비스 서버 선언

ros::ServiceServer modeChange_srv_server;
ros::ServiceServer rtl_srv_server;
ros::ServiceServer goto_srv_server;
ros::ServiceServer geofence_srv_server;
ros::ServiceServer noflyzone_srv_server;
ros::ServiceServer checkNFZone_srv_server;


//서비스 클라이언트 선언
ros::ServiceClient arming_client; // 서비스 클라이언트 선언
ros::ServiceClient modeChange_client; // 모드 변경 서비스 클라이언트 
ros::ServiceClient rtl_client; // 모드 변경 서비스 클라이언트 


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




void state_cb(const mavros_msgs::State::ConstPtr& msg){

	


	
	current_state = *msg;

		

	if (current_state.mode.compare("AUTO.RTL") ==0)
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

	
//	printf("home position: (%f, %f, %f) \n", home.latitude, home.longitude, home.altitude);	
}


// callback 함수 (서비스 제공) 정의

bool srv_arming_cb(mission_lib_msgs::Arming::Request &req, mission_lib_msgs::Arming::Response &res )
{

	ROS_ERROR("ARMing request received\n");
	arming_cmd.request.value = true; // 서비스 요청 메시지 필드 설정

	//// Arming

  	while (ros::ok() ) // 서비스 요청 메시지 전달
  	{
  	      printf("send Arming command ...\n");

		if (!arming_client.call(arming_cmd))
        	 {
	 	  ros::spinOnce();
 //   	  	 rate.sleep();
        	}
        	else break;

  	} 
	 
	ROS_ERROR("ARMing command was sent\n");

}

bool srv_modeChange_cb(mission_lib_msgs::ModeChange::Request &req, mission_lib_msgs::ModeChange::Response &res)
{
	
	std::cout << "srv_modeChange_cb(): change the mode to " << req.mode << endl; 

	modeChange_cmd.request.base_mode = 0;
        
	modeChange_cmd.request.custom_mode.assign(req.mode);

       if (modeChange_client.call(modeChange_cmd)==true)
	{
		std::cout << " modeChange cmd was sent!\n " << endl;
	}

	


	return true;
}

bool srv_rtl_cb(mission_lib_msgs::RTL::Request &req, mission_lib_msgs::RTL::Response &res)
{
	std::cout << "srv_rtl_cb():return to home"  << endl; 

	modeChange_cmd.request.base_mode = 0;
        
	modeChange_cmd.request.custom_mode.assign("AUTO.RTL");

       if (modeChange_client.call(modeChange_cmd)==true)
	{
		std::cout << " modeChange cmd was sent!\n " << endl;
	}

	return true;
}


bool srv_goto_cb(mission_lib_msgs::Goto::Request &req, mission_lib_msgs::Goto::Response &res)
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
		/*
		target_position.pos_global.latitude = req.x_lat;
		target_position.pos_global.longitude = req.y_long;
		target_position.pos_global.altitude = req.z_alt;

		target_pos_global.latitude = req.x_lat;
		target_pos_global.longitude = req.y_long;
		target_pos_global.altitude = req.z_alt;
		*/
		
		Point point = convertGeoToPoint(req.x_lat, req.y_long, req.z_alt, HOME_LAT, HOME_LON, HOME_ALT);

		
		target_position.pos_local.x = point.x;
		target_position.pos_local.y = point.y;
		target_position.pos_local.z = point.z;

		target_pos_local.pose.position = point;

		printf("target position: (%f, %f, %f)\n", point.x, point.y, point.z);
	 
	}
	else // 지역 좌표인 경우
	{
		//
		
		target_pos_local.pose.position.x = req.x_lat;

		target_pos_local.pose.position.y = req.y_long;
		target_pos_local.pose.position.z = req.z_alt;

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
	return true;
}

bool srv_geofence_cb(mission_lib_msgs::Geofence::Request &req, mission_lib_msgs::Geofence::Response &res)
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

bool srv_noflyZone_cb(mission_lib_msgs::NoflyZone::Request &req, mission_lib_msgs::NoflyZone::Response &res)
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

bool srv_checkNFZone_cb (mission_lib_msgs::CheckNFZone::Request &req, mission_lib_msgs::CheckNFZone::Response &res)
{
	ROS_INFO ("checkNFZone service was called:\n");

	if (nofly_zone.isSet == true)
	{
		res.isSet = true;

		res.min_x_lat = nofly_zone.min_x_lat;
		res.min_y_long = nofly_zone.min_y_long;
		res.max_x_lat = nofly_zone.max_x_lat;
		res.max_y_long = nofly_zone.max_y_long;
	}

	else
	{
		res.isSet = false;
	}
	
	
}


int main(int argc, char** argv)

{

	ros::init(argc, argv, "mission_lib_control");

 	ros::NodeHandle nh; 
	
	ros::Rate rate (20.0); 
	
	
	// publisher 초기화
	pos_pub_local = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        pos_pub_global = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);
	cur_target_pub = nh.advertise<mission_lib_msgs::Target>("mission_lib/current_target", 10);

	// subscriber 초기화
	state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
	pos_sub_local = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose",10,  pos_cb_local); 
	pos_sub_global = nh.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global",10,  pos_cb_global); 
	home_sub = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, homePosition_cb);

	//// 서비스 서버 선언
	
	modeChange_srv_server = nh.advertiseService("srv_modeChange", srv_modeChange_cb);
	rtl_srv_server = nh.advertiseService("srv_rtl", srv_rtl_cb);
	goto_srv_server = nh.advertiseService("srv_goto", srv_goto_cb); 
	geofence_srv_server = nh.advertiseService("srv_geofence", srv_geofence_cb);		
	noflyzone_srv_server = nh.advertiseService("srv_noflyZone", srv_noflyZone_cb);	
	checkNFZone_srv_server = nh.advertiseService("srv_checkNFZone", srv_checkNFZone_cb);


	//// 서비스 클라이언트 선언

	arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");	
	rtl_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");

	modeChange_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
	target_position.reached = false; // 목적지 도착 여부를 false로 초기화
	
	while ( ros::ok() )
	{
/*
		if (current_state.mode.compare("AUTO.RTL") == 0) // RTL 모드일 경우 바로 종료
		{
			break;
		}
*/
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

				if (current_state.mode.compare("AUTO.RTL") !=0)
				{
					//ROS_ERROR("state_cb(): FLIGHT MODE = RTL");
				
					modeChange_cmd.request.base_mode = 0;
					modeChange_cmd.request.custom_mode = "OFFBOARD";
					modeChange_client.call(modeChange_cmd);
				}
			}
		  	
			//ros::spinOnce();
      		  	//rate.sleep();

			//printf("home position: (%f, %f, %f) \n", home_position.position.x, home_position.position.y, home_position.position.z);	
	
		}// if (autonomous_flight)

			ros::spinOnce();
      		  	rate.sleep();

	}

	return 0;
}
