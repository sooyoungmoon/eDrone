
/* include 문 */ 
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Goto.h>
#include <eDrone_msgs/SprayStart.h>
#include <eDrone_msgs/SprayStop.h>
#include <eDrone_msgs/RTL.h>
#include <eDrone_msgs/Disarming.h>
#include <eDrone_msgs/Landing.h>
#include <params.h>

using namespace std;

std::vector<eDrone_msgs::Target> path; // 무인기 자율 비행 경로

/* 토픽 메시지 선언부 */

eDrone_msgs::Target cur_target;
eDrone_msgs::Target next_target; 

/* 서비스 선언부 */

eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Goto goto_cmd;
eDrone_msgs::SprayStart sprayStart_cmd;
eDrone_msgs::SprayStop sprayStop_cmd;
eDrone_msgs::RTL rtl_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::Disarming disarming_cmd;


/* 콜백 함수 (토픽 구독) 정의부 */


/* void ${ }_cb (const eDrone_msgs::${}::ConstPtr& msg)
 
}
*/ 

void cur_target_cb (const eDrone_msgs::Target::ConstPtr& msg)
{
	cur_target = *msg;
	
}


/* 콜백 함수 (서비스 요청 처리) 정의부 */

/*

bool srv_${SERVICE_FILE_NAME_LOWERCASE}}_cb(eDrone_msgs::${SERVICE_FILE_NAME}::Request &req, eDrone_msgs::${SERVICE_FILE_NAME}::Response &res)
{

}

*/


/* main 함수*/

int main(int argc, char** argv)
{
   printf("==mission==\n");
   /* ros::init() 함수 호출 */ 
   ros::init(argc, argv, "mission");

  /*   ros::NodeHandle 객체 선언*/
   ros::NodeHandle nh;

 /* 파라미터 설정 */
 // $(PARAMETER_TYPE) $(PARAMETER_VARIABLE) = $(PARAMETER_NAME);

 double altitude = ALTITUDE;
  
 nh.setParam("ALTITUDE", altitude);
  
  /* subscriber 선언 */

  // ros::subscriber ${MSG_VAR_NAME}_sub = nh.subscribe("${TOPIC_NAME}, ${MSG_QUEUE_SIZE}, ${CALLBACK_FUNCTION_NAME});

  
  ros::Subscriber cur_target_sub = nh.subscribe("eDrone_msgs/current_target", 10, cur_target_cb);


  /* publishring rate 설정 */
  ros::Rate rate(20.0);

  /* service client 선언 */ 
  //ros::ServiceClient $(SERVICE_CLIENT_NAME) =nh.serviceClient<eDrone_msgs::$(SERVICE_FILE_NAME)>("$(SERVICE_NAME)");  

  
  ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  

  ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition"); 

 
  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");  
 
  ros::ServiceClient takeoff_client = nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");  

  ros::ServiceClient goto_client =nh.serviceClient<eDrone_msgs::Goto>("srv_goto");  
  
  ros::ServiceClient sprayStart_client = nh.serviceClient<eDrone_msgs::SprayStart>("srv_sprayStart");  

  ros::ServiceClient sprayStop_client  = nh.serviceClient<eDrone_msgs::SprayStop>("srv_sprayStop"); 

  ros::ServiceClient rtl_client  = nh.serviceClient<eDrone_msgs::RTL>("srv_rtl");
 
  ros::ServiceClient disarming_client  = nh.serviceClient<eDrone_msgs::Disarming>("srv_disarming"); 

  /* service 요청 메시지 필드 설정 */
 // $(SERVICE_FILE_NAME)_cmd.$(PARAMETER_NAME) = $(PARAMETER_VALUE)
  

  // path 설정

  
int cur_target_seq_no = -1; // 현재 target 순번 (0, 1, 2, ...)

	// 초기 부분 경로 설정 (사각형 경로)
	  // % mission 명령에서 사용되는 waypoint와 구분하기 위해 
	  // 위치 이동 명령에서 사용되는 경유지는 target으로 지칭

	// Starting position
	
	next_target.is_global = false;
	next_target.x_lat = STARTING_POS_X_LAT;
        next_target.y_long = STARTING_POS_Y_LON;
	next_target.z_alt = ALTITUDE;
	next_target.reached = false;
	path.push_back(next_target);

	 // target#1
	
	next_target.is_global = false;
	next_target.x_lat = WP_1_X_LAT;     
        next_target.y_long = WP_1_Y_LON;
	next_target.z_alt = ALTITUDE;
	next_target.reached = false;
	path.push_back(next_target);

	 // target#2

	
	next_target.is_global = false;
	next_target.x_lat = WP_2_X_LAT;     
        next_target.y_long = WP_2_Y_LON;
	next_target.z_alt = ALTITUDE;
	next_target.reached = false;
	path.push_back(next_target);

	 // target#3 	

	
	next_target.is_global = false;
	next_target.x_lat = WP_3_X_LAT;     
        next_target.y_long = WP_3_Y_LON;
	next_target.z_alt = ALTITUDE;
	next_target.reached = false;
	path.push_back(next_target);


	 // target#4 	

	
	next_target.is_global = false;
	next_target.x_lat = WP_4_X_LAT;     
        next_target.y_long = WP_4_Y_LON;
	next_target.z_alt = ALTITUDE;
	next_target.reached = false;
	path.push_back(next_target);

	 // target#5 	

	
	next_target.is_global = false;
	next_target.x_lat = WP_5_X_LAT;     
        next_target.y_long = WP_5_Y_LON;
	next_target.z_alt = ALTITUDE;
	next_target.reached = false;
	path.push_back(next_target);
	cur_target_seq_no = 0; // 0번 index의 요소를 현재 target으로 지정
	

	sleep(10);
  
	


  /* 서비스 호출 */

  /* $(SERVICE_FILE_NAME)
   if ($(SERVICE_CLIENT_NAME).call($(SERVICE_FILE_NAME)_cmd))
	ROS_INFO("$(SERVICE_FILE_NAME) command was sent\n"); */

        //// CheckState

        
        // flight controller 연결 확인

	ROS_INFO("Send checkState command ... \n");
	ROS_INFO("Checking the connection ... \n");

	
	if (checkState_client.call(checkState_cmd))
	{
		ROS_INFO ("CheckState service was requested");
		
		while (checkState_cmd.response.connected == false)
		{
			if (checkState_client.call(checkState_cmd));
			{
				ROS_INFO ("Checking state...");
			}

			ros::spinOnce();
			rate.sleep();
		}

		ROS_INFO("UAV connection established!");
	}



	// 현재 위치 확인 

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
			rate.sleep();
		}
	

		cout <<"global frame: (" << checkPosition_cmd.response.latitude << ", " << checkPosition_cmd.response.longitude << ", " << checkPosition_cmd.response.altitude << ") " << endl << endl;

	        cout <<"local frame: (" << checkPosition_cmd.response.x << ", " << checkPosition_cmd.response.y << ", " << checkPosition_cmd.response.z << ") " << endl;
	

		ROS_INFO("UAV position was checked!");
	}

	while (checkPosition_cmd.response.value == false)
	{
		if (checkPosition_client.call(checkPosition_cmd));
		{
			ROS_INFO ("Checking position...");
		}

		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("UAV position was checked!");

	


        //// Arming

	printf("Send arming command ... \n");
	if (arming_client.call(arming_cmd));
	{
		ROS_INFO ("Arming service was requested");

	}




	//// Takeoff

	printf("Send takeoff command ... \n");

	nh.getParam("ALTITUDE", altitude );
	takeoff_cmd.request.altitude  = altitude;
	if (takeoff_client.call(takeoff_cmd))
	{
		ROS_INFO("Takeoff service was requested");
	}

	sleep(10);
	
	

	//// Goto


	printf("Send goto command ...\n");
	printf("let's start a mission! \n");

	goto_cmd.request.is_global = false;
	goto_cmd.request.x_lat = STARTING_POS_X_LAT;
	goto_cmd.request.y_long =STARTING_POS_Y_LON;
	goto_cmd.request.z_alt = ALTITUDE;
	
	if (goto_client.call(goto_cmd))
	{
		ROS_INFO("Goto service was requested\n");
	}

	//// SprayStart
	
	
	printf("Send SprayStart command ...\n");
	if (sprayStart_client.call(sprayStart_cmd))
	{
		ROS_INFO("SprayStart service was requested\n");
	}

	//// Goto requests (path planning) 	
	// 주기적으로 자율 비행 경로에 새로운 목적지 (경유지) 정보 추가
	// 현재 목적지에 도달한 경우 더 남아 있는 경유지가 있다면 새로운 목적지로 goto service 호출

	while(ros::ok())
	{

		if (cur_target.reached== true)
		{
			if (cur_target.target_seq_no < path.size())
			{

				ROS_INFO("we reached at the current target. Go to the next target\n");

				//int next_target_seq_no = cur_target.target_seq_no +1;
				
				cur_target_seq_no = cur_target.target_seq_no+1;
					
				if (cur_target_seq_no >= path.size() )
				{
					break;
			 	}	
				
				next_target = path[cur_target_seq_no];

				goto_cmd.request.target_seq_no = cur_target_seq_no; // target seq no 설정
				goto_cmd.request.is_global = next_target.is_global;
				goto_cmd.request.x_lat = next_target.x_lat;
				goto_cmd.request.y_long = next_target.y_long;
				goto_cmd.request.z_alt = next_target.z_alt;
				
				if (goto_client.call(goto_cmd))  // 새로운 경유지로 goto service 호출	
				{
					ROS_INFO("Goto service was requested");
				}
			} 

			else
				break;
		}


		ros::spinOnce();
		rate.sleep();

		// 경유지 추가 (필요 시)

	}

	// SprayStop

	ROS_INFO("Send SprayStop command...");

	if (sprayStop_client.call (sprayStop_cmd))
	{
		ROS_INFO("SprayStop service was requested");
	}

	
	// RTL

	
	ROS_INFO("Send RTL command...");

	if (rtl_client.call (rtl_cmd))
	{
		ROS_INFO("RTL service was requested");
	}

	// Disarming


	ROS_INFO("Send Disarming command...");

	if (disarming_client.call (disarming_cmd))
	{
		ROS_INFO("Disarming service was requested");
	}

	

  return 0;
}

