


/* include */

#ifndef goto1024_3
// 기본 header (ROS & C/C++)
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <vector>
#include <stdlib.h>
#include <geometry_msgs/Point.h> 
#include <geographic_msgs/GeoPoint.h> 
#include <string>

// 토픽 선언 header 
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/Phase.h>

// 파라미터 초기값 선언 header
#include <goto1024_3/params.h>

// 서비스 선언 header
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Goto.h>
#include <eDrone_msgs/RTL.h>

/* namespace */
#endif
using namespace std;
using namespace geographic_msgs;
using namespace geometry_msgs;
using namespace eDrone_msgs;

/* 포인터 변수 선언 */
ros::NodeHandle* nh_ptr; // node handle pointer (서버/클라이언트 또는 퍼블리셔/서브스크라이버 선언에 사용)
eDrone_msgs::Target* cur_target_ptr; // cur_target 변수 접근을 위한 포인터 변수 
eDrone_msgs::Phase* cur_phase_ptr; // cur_phase		"


/* 콜백 함수 정의 */

// 토픽 콜백 함수

void cur_target_cb(const eDrone_msgs::Target::ConstPtr& msg)
{
	*cur_target_ptr = *msg;

	// 현재 목적지 도달 여부 확인
	ROS_INFO("cur_target_cb(): \n");
	ROS_INFO("current target: %d \n", cur_target_ptr->target_seq_no);
	
	if (cur_target_ptr->reached == true)
	{
		ROS_INFO("we reached at the current target\n");
	} 
}

void cur_phase_cb(const eDrone_msgs::Phase::ConstPtr& msg)
{
	*cur_phase_ptr = *msg;

	// 현재 목적지 도달 여부 확인
	ROS_INFO("cur_phase_cb(): \n");
	ROS_INFO("current phase: %s \n", cur_phase_ptr->phase.c_str()); 
}

/* 기타 함수 정의 */

/* main 함수 */
int main (int argc, char** argv)
{

// ROS node 초기화

	ROS_INFO("==goto1024_3==\n");	
	ros::init(argc, argv, "goto1024_3"); 
	ros::NodeHandle nh;
	nh_ptr = &nh; // node handle 주소 저장 	


	for (int arg_index = 0; arg_index < argc; arg_index++)
	{
		ROS_INFO("main arg[%d]: %s", arg_index, argv[arg_index] );
	}

// Topic 메시지 변수 
	eDrone_msgs::Target cur_target; // 무인기가 현재 향하고 있는 목적지 (경유지)
	eDrone_msgs::Phase cur_phase; // 무인기의 현재 동작 단계 (ex. UNARMED, ARMED, TAKEOFF, GOTO, ...)
	cur_target_ptr = &cur_target; // cur_target 변수 주소 저장 
	cur_phase_ptr = &cur_phase;

// Service 메시지 변수
	

eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
	eDrone_msgs::Goto goto_cmd;
	eDrone_msgs::RTL rtl_cmd;

// 기타 변수


// Topic Subscriber 객체 
	
	// rate 설정 
	ros::Rate rate(20.0);

	// 토픽 subscriber 선언 & 초기화 

	ros::Subscriber cur_target_sub = nh.subscribe("eDrone_msgs/current_target", 10, cur_target_cb); // 
	ros::Subscriber cur_phase_sub = nh.subscribe("eDrone_msgs/current_phase", 10, cur_phase_cb);	

// ROS Service Client 객체 
	
ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  
ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition"); 
ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");
ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
ros::ServiceClient goto_client = nh.serviceClient<eDrone_msgs::Goto>("srv_goto");
	ros::ServiceClient rtl_client = nh.serviceClient<eDrone_msgs::RTL>("srv_rtl");

// ROS Service 호출 

	    // 연결 상태 확인

		sleep(10); // (수정)
		ROS_INFO("Send checkState command ... \n");
		ROS_INFO("Checking the connection ... \n");
		
		if (checkState_client.call(checkState_cmd))
		{
			ROS_INFO ("CheckState service was requested");
			
			while (checkState_cmd.response.connected == false)
			{
				if (checkState_client.call(checkState_cmd))
				{
					ROS_INFO ("Checking state...");
				}

				ros::spinOnce();
				rate.sleep();
			}

			ROS_INFO("UAV connection established!");

			if (checkState_cmd.response.connected == true)
			{
				cout << "UAV connected: " << endl;
			}
			
			if (checkState_cmd.response.armed == true )
			{
				cout << "UAV armed: " << endl;
			}

			cout << "flight mode: " << checkState_cmd.response.mode << endl;

			cout << "remaining battery(%): " << checkState_cmd.response.battery_remain << endl;
		}

	    // 무인기 위치 확인

		 ROS_INFO("Send checkPosition command ... \n");
		 ROS_INFO("Checking the position ... \n");

		 if (checkPosition_client.call(checkPosition_cmd))
		 {
			ROS_INFO ("CheckPosition service was requested");

			while (checkPosition_cmd.response.value == false)
			{
				if (checkPosition_client.call(checkPosition_cmd));
				{
					ROS_INFO ("Checking position...");
				}

				ros::spinOnce();
				sleep(10);
			}


			cout <<"global frame (WGS84): (" << checkPosition_cmd.response.latitude << ", ";
			cout << checkPosition_cmd.response.longitude << ", ";
			cout << checkPosition_cmd.response.altitude << ") " << endl << endl;

			cout <<"local frame (ENU): (" << checkPosition_cmd.response.x << ", ";
			cout << checkPosition_cmd.response.y << ", " << checkPosition_cmd.response.z << ") " << endl;
			
			}
			
			ROS_INFO("UAV position was checked!");
	// Arming

		ROS_INFO("Send arming command ... \n"); 
		arming_client.call(arming_cmd);
		
		if (arming_cmd.response.value == true)
		{
			ROS_INFO("Arming command was sent to FC");
		}
  // Takeoff
	
	double takeoff_altitude = 0;

	takeoff_altitude = atof (argv[1]);
	// takeoff_altitude = (double) argv[1]; // 도구에서 생성된 코드
	// 1) 상수에 의한 초기화: takeoff_altitude = TAKEOFF_ALTITUDE;
	// 2) 명령줄 인자에 의한 초기화: takeoff_altitude = atof (argv[1]); 

	ROS_INFO("takeoff_altitude: %lf", takeoff_altitude);

	ROS_INFO("Send takeoff command ... \n");
	takeoff_cmd.request.takeoff_altitude = takeoff_altitude; // 서비스 파라미터 설정
	takeoff_client.call(takeoff_cmd); // 서비스 호출

	if (takeoff_cmd.response.value == true) // 서비스 호출 결과 확인 
	{
		ROS_INFO("Takeoff command was sent to FC\n");
	}
		
	sleep(10);
	// Goto

	while (cur_phase.phase.compare ("READY")!=0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}

	string goto_ref_system = GOTO_REF_SYSTEM;
	goto_cmd.request.goto_ref_system = goto_ref_system;
	
	Target goto_point; // template에 포함	
	string goto_point_str = argv[2]; // template에 포함
	//string goto_point_str = GOTO_POINT_STR; // 도구에서 생성된 코드
	
	// Target이 구조체이므로 goto_point을 초기화 하는데 필요한 문자열 변수 (goto_point_str)가 선언되었다는 가정 하에 아래 코드 생성  	

	// 1) 상수에 의한 초기화의 경우 	   
	  // (params.h에 선언) string GOTO_POINT_STR = "30,30,10";	
	  // goto_point_str = GOTO_POINT_STR;			
	
	  // 2) 명령줄 인자에 의한 초기화의 경우 
	  // goto_point_str = argv[i]; 


	// 1), 2) 공통  
	{
		// 도구에서 자동 생성 요 (시작)
			int numCnt = 0;
			vector<string>  strVector;

			string token;
			size_t delimiter_pos = 0;
			delimiter_pos = goto_point_str.find(",");

			while (delimiter_pos != string::npos)
			{
				numCnt++;
				token = goto_point_str.substr(0, delimiter_pos);
				goto_point_str = goto_point_str.substr (delimiter_pos+1);
				cout << "token: " << token << endl;
				strVector.push_back(token);
				cout << "goto_point_str: " << goto_point_str << endl;
				delimiter_pos = goto_point_str.find(",");
			}

			token = goto_point_str.substr(0, delimiter_pos);
			strVector.push_back(token);
			numCnt++;
			cout << "token: " << token << endl;
			cout << "numCnt: " << numCnt << endl;

		// 도구에서 자동 생성 요 (종료)

			string x_lat_str = strVector[0];
			goto_point.x_lat = atof (x_lat_str.c_str());

			string y_long_str = strVector[1];
			goto_point.y_long = atof (y_long_str.c_str());

			string z_alt_str = strVector[2];
			goto_point.z_alt = atof (z_alt_str.c_str());

			goto_cmd.request.goto_point = goto_point;
	} 
	

	 // goto_point parameter에 데이터 저장 완료 

	if (goto_client.call(goto_cmd) == true)
	{
		ROS_INFO("Goto command was sent to FC\n");
	}

	 // goto_point parameter에 데이터 저장 완료

	// 초기화 루틴 종료

	while (cur_phase.phase.compare ("READY")==0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}
	// RTL

	while (cur_phase.phase.compare ("READY")!=0)
	{
		ros::spinOnce();
		rate.sleep();
		cout << "cur_phase: " << cur_phase_ptr->phase << endl;
	}

	rtl_client.call(rtl_cmd); // rtl service 호출 (복귀)
	
	if (rtl_cmd.response.value == true)
	{
		ROS_INFO("RTL command was sent\n");
        }


return 0;

}
