
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
#include <mavros_msgs/GlobalPositionTarget.h>
#include "Vehicle.h"
#include "GeoInfo.h"
#include <eDrone_msgs/Arming.h> // 시동 서비스 헤더 파일
#include <eDrone_msgs/Goto.h> // 무인기 위치 이동 서비스 헤더 파일 포함
#include <eDrone_msgs/ModeChange.h> // 비행 모드 변경 서비스 헤더 파일
#include <eDrone_msgs/RTL.h> // RTL
#include <eDrone_msgs/Target.h> // 현재 목적지 topic 메시지가 선언된 헤더 파일 포함
#include <eDrone_msgs/Geofence.h> // Geofence 서비스 헤더 파일
#include <eDrone_msgs/NoflyZone.h> // Noflyzone 서비스 헤더 파일
#include <eDrone_msgs/CheckNFZone.h> // CheckNFZone 서비스 헤더 파일 (noflyZone 확인)
#include <eDrone_msgs/Survey.h> // Survey 서비스 헤더 파일
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




//// 경로 변수

std::vector<eDrone_msgs::Target> path; // 무인기 자율 비행 경로 


//// Survey 서비스 피요청 여부
bool survey_srv_called = false;


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
eDrone_msgs::Target cur_target; // 현재 목적지 정보 (publisher: eDrone_control_node, subscriber: eDrone_application_node)

eDrone_msgs::Target next_target; // 다음  목적지 정보 (goto 서비스 요청에 사용)
int cur_target_seq_no = -1; // survey 기능 수행 시 목적지 순번 (0, 1, 2, ...)


//// 서비스 요청 메시지 선언 (mavros)
mavros_msgs::CommandBool arming_cmd;
mavros_msgs::CommandLong commandLong_cmd;// 무인기 제어에 사용될 서비스 선언
mavros_msgs::SetMode modeChange_cmd; // 모드 변경에 사용될 서비스 요청 메시지
mavros_msgs::SetMode rtl_cmd; // 복귀 명령에 사용될 서비스 요청 메시지i


//// 서비스 요청 메시지 선언 (eDrone_msgs)
eDrone_msgs::Goto goto_cmd; // goto 요청 메시지


// publisher 선언

//ros::Publisher state_pub;
ros::Publisher pos_pub_local;
ros::Publisher pos_pub_global;


// subscriber 선언

ros::Subscriber state_sub;
ros::Subscriber pos_sub_local;
ros::Subscriber pos_sub_global;
ros::Subscriber home_sub; 
ros::Subscriber cur_target_sub; // 현재 목적지 정보 구독 

// 서비스 서버 선언

ros::ServiceServer survey_srv_server;

//서비스 클라이언트 선언
ros::ServiceClient arming_client; // 서비스 클라이언트 선언
ros::ServiceClient modeChange_client; // 모드 변경 서비스 클라이언트 
ros::ServiceClient rtl_client; // 모드 변경 서비스 클라이언트 
ros::ServiceClient goto_client; // goto 서비스 클라이언트 

// home position


 float HOME_LAT ;
 float HOME_LON;
 float HOME_ALT;

void cur_target_cb (const eDrone_msgs::Target::ConstPtr& msg)
{
	cur_target = *msg;

		
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	
	current_state = *msg;

		

	if (current_state.mode.compare("AUTO.RTL") ==0)
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

	current_pos_local = *msg;

}

void pos_cb_global(const sensor_msgs::NavSatFix::ConstPtr& msg){

	

	current_pos_global = *msg;

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

bool srv_arming_cb(eDrone_msgs::Arming::Request &req, eDrone_msgs::Arming::Response &res )
{

	ROS_INFO("ARMing request received\n");
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
	 
	ROS_INFO("ARMing command was sent\n");

}

bool srv_modeChange_cb(eDrone_msgs::ModeChange::Request &req, eDrone_msgs::ModeChange::Response &res)
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

bool srv_rtl_cb(eDrone_msgs::RTL::Request &req, eDrone_msgs::RTL::Response &res)
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


bool srv_survey_cb(eDrone_msgs::Survey::Request &req, eDrone_msgs::Survey::Response &res)
{

	std::cout << "srv_survey_cb(): survey a target area"  << endl; 
	
	survey_srv_called = true;

	cur_target_seq_no = 0;

	// 변수 선언

	double row_col_width = 0;

	double min_x_lat = 0;

	double min_y_long = 0;

	double max_x_lat = 0;

	double max_y_long = 0;

	double x_lat = 0;

	double y_long = 0;

	int row = 0; // 행 번호

	int col = 0;	// 열 번호

	int num_rows = 0; // 행 개수 

	int num_cols = 0; // 열 개수 

	int target_seq_no = 0; // 목표 지점 순번

	min_x_lat = req.min_x_lat;
	
	min_y_long = req.min_y_long;
	
	max_x_lat = req.max_x_lat;

	max_y_long = req.max_y_long;
	
	row_col_width = req.row_col_width;

	num_rows = (max_x_lat - min_x_lat) / row_col_width;

	num_cols = (max_y_long - min_y_long) / row_col_width;

	
	ROS_INFO("min_x_lat: %lf", min_x_lat);
	ROS_INFO("min_y_long: %lf", min_y_long);
	ROS_INFO("max_x_lat: %lf", max_x_lat);
	ROS_INFO("max_y_long: %lf", max_y_long);

	ROS_INFO("row_col_width: %lf", row_col_width);

	ROS_INFO("num_rows: %d", num_rows);
	ROS_INFO("num_cols: %d", num_cols);

	//sleep(10);


	// 최초 목적지 설정

	next_target.target_seq_no = 0;

	next_target.is_global = req.is_global;

	next_target.x_lat = req.min_x_lat;
	
	next_target.y_long = req.min_y_long;
		
	if (next_target.is_global == true) // 절대 고도일 경우 높이 지정
	{
		next_target.z_alt = HOME_ALT + 10;
	}
	else // 상대 고도일 경우 높이 지정
	{
		next_target.z_alt = 10;
	}


	next_target.reached = false;
		

	// 경로 좌표 계산 & 경로 변수 값  설정

			
	for (col = 0; col < num_cols; col++)
	{

		x_lat = min_x_lat + col * row_col_width; 


		for (row = 0; row < num_rows; row++)
		{
			if (col %2 ==0)
			{
				y_long = min_y_long + row * row_col_width;
			}
			else
			{
				y_long = max_y_long - row * row_col_width;
			}
	

			eDrone_msgs::Target target;

			target.is_global = req.is_global;
			target.x_lat = x_lat;
			target.y_long = y_long;
			
			if (target.is_global == true) // 절대 고도일 경우 높이 지정
			{
				target.z_alt = HOME_ALT + 10;
			}
			else // 상대 고도일 경우 높이 지정
			{
				target.z_alt = 10;
			}
			
			target.reached = false;
		
			path.push_back(target);
			
			ROS_INFO("target seq no. %d (%lf, %lf)", target_seq_no, x_lat, y_long);			

			target_seq_no++;
		}
	}



	return true;

}




int main(int argc, char** argv)

{

	ros::init(argc, argv, "eDrone_application_node");

 	ros::NodeHandle nh; 
	
	ros::Rate rate (20.0); 
	
	
	// publisher 초기화
	pos_pub_local = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        pos_pub_global = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);

	// subscriber 초기화
	state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
	pos_sub_local = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose",10,  pos_cb_local); 
	pos_sub_global = nh.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global",10,  pos_cb_global); 
	home_sub = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, homePosition_cb);
	
	cur_target_sub = nh.subscribe<eDrone_msgs::Target> ("eDrone_lib/current_target", 10, cur_target_cb);


	//// 서비스 서버 선언

	survey_srv_server = nh.advertiseService("srv_survey", srv_survey_cb);

//	int cur_target_seq_no = -1; // survey 기능 수행 시 목적지 순번 (0, 1, 2, ...)

	//modeChange_srv_server = nh.advertiseService("srv_modeChange", srv_modeChange_cb);

	//// 서비스 클라이언트 선언

	arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming");	
	rtl_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
	goto_client = nh.serviceClient<eDrone_msgs::Goto>("srv_goto");
		
	while ( ros::ok() ) // 탐색 서비스 요청 메시지 처리
	{

		if (survey_srv_called == true)
		{
			// 현재 목적지로 이동하기 위해 goto 서비스 호출
		
			goto_cmd.request.value = true;
			
			goto_cmd.request.target_seq_no = next_target.target_seq_no;

			goto_cmd.request.is_global = next_target.is_global;

			goto_cmd.request.x_lat = next_target.x_lat;
	
			goto_cmd.request.y_long = next_target.y_long;

			goto_cmd.request.z_alt = next_target.z_alt;

			goto_client.call(goto_cmd);


			// 현재 목적지에 도착했으면 다음 목적지 정보를 갱신 => 

			if (cur_target.reached == true)
			{

				cout << "eDrone_application_node: cur_target.reached == true" << endl;
				if (cur_target.target_seq_no < path.size()-1 )
				{
				//	cur_target_seq_no = cur_target.target_seq_no +1;
					cur_target_seq_no++;

					next_target = path[cur_target_seq_no];

					ROS_INFO("next_target %d: (%lf, %lf)", next_target.target_seq_no, next_target.x_lat, next_target.y_long);
				}
				else break;
			}
			
		}		


		ros::spinOnce();
      	  	rate.sleep();

	}

	return 0;
}
