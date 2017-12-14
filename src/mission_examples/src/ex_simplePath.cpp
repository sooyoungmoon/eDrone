
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include <mavros_msgs/SetMode.h>
#include "mission_lib_msgs/Arming.h"
#include "mission_lib_msgs/Takeoff.h"
#include "mission_lib_msgs/Landing.h"
#include "mission_lib_msgs/Goto.h"
#include "mission_lib_msgs/Target.h"



std::vector<mission_lib_msgs::Target> path; // 무인기 자율 비행 경로


mission_lib_msgs::Arming arming_cmd;
mission_lib_msgs::Takeoff takeoff_cmd;
mission_lib_msgs::Landing landing_cmd;
mission_lib_msgs::Goto goto_cmd;



// 메시지 변수 선언
mission_lib_msgs::Target cur_target; // 무인기가 현재 향하고 있는 목적지 (경유지)
mission_lib_msgs::Target next_target; // 다음 목적지 



// 콜백 함수 
void cur_target_cb(const mission_lib_msgs::Target::ConstPtr& msg)
{
	cur_target = *msg;

	// 현재 목적지 도달 여부 확인
	printf("cur_target_cb(): \n");
	printf("current target: %d \n", cur_target.target_seq_no);
	
	if (cur_target.reached == true)
	{
		printf("we reached at the current target\n");
	} 
}


int main(int argc, char** argv)
{
	printf("==ex_simplePath==\n");

	ros::init(argc, argv, "ex_simplePath");
	ros::NodeHandle nh;

		
	// subscriber 선언
	ros::Subscriber cur_target_sub= nh.subscribe("mission_lib/current_target", 10, cur_target_cb); // 
	
	ros::Rate rate(20.0);

	// service client 선언
	
	ros::ServiceClient arming_client =nh.serviceClient<mission_lib_msgs::Arming>("srv_arming");
	ros::ServiceClient takeoff_client =nh.serviceClient<mission_lib_msgs::Takeoff>("srv_takeoff");
	ros::ServiceClient landing_client =nh.serviceClient<mission_lib_msgs::Landing>("srv_landing");
	ros::ServiceClient goto_client = nh.serviceClient<mission_lib_msgs::Goto>("srv_goto");
	

	int cur_target_seq_no = -1; // 현재 target 순번 (0, 1, 2, ...)

	// 초기 부분 경로 설정 (사각형 경로)
	  // % mission 명령에서 사용되는 waypoint와 구분하기 위해 
	  // 위치 이동 명령에서 사용되는 경유지는 target으로 지칭
	
	 // target#1
	
	next_target.is_global = false;
	next_target.x_lat = 0;                                                                                                                next_target.y_long = 100;
	next_target.z_alt = 50;
	next_target.reached = false;
	path.push_back(next_target);

	 // target#2

	
	next_target.is_global = false;
	next_target.x_lat = 100;                                                                                                              next_target.y_long = 100;
	next_target.z_alt = 50;
	next_target.reached = false;
	path.push_back(next_target);

	 // target#3 	

	
	next_target.is_global = false;
	next_target.x_lat = 100;                                                                                
        next_target.y_long = 0;
	next_target.z_alt = 50;
	next_target.reached = false;
	path.push_back(next_target);


	cur_target_seq_no = 0; // 0번 index의 요소를 현재 target으로 지정
	

	//// Arming

	printf("Send arming command ... \n");
	arming_cmd.request.value = true;
	arming_client.call(arming_cmd);
	ROS_ERROR("Arming command was sent\n");



	//// Takeoff

	printf("Send takeoff command ... \n");
	takeoff_cmd.request.value = true;
	takeoff_client.call(takeoff_cmd);
	ROS_ERROR("Takeoff command was sent\n");

	sleep(10);
	
	

	//// Goto


	printf("Send goto command ...\n");
	printf("let's start a mission! \n");


	goto_cmd.request.value = true;
	goto_cmd.request.is_global = false;
	goto_cmd.request.x_lat = 0;
	goto_cmd.request.y_long = 100;
	goto_cmd.request.z_alt = 50;
	
	goto_client.call(goto_cmd);
	ROS_ERROR("Goto command was sent\n");

	// 주기적으로 자율 비행 경로에 새로운 목적지 (경유지) 정보 추가
	// 현재 목적지에 도달한 경우 더 남아 있는 경유지가 있다면 새로운 목적지로 goto service 호출

	while(ros::ok())
	{

		if (cur_target.reached== true)
		{
			if (cur_target.target_seq_no < path.size())
			{

				ROS_ERROR("we reached at the current target. Go to the next target\n");

				//int next_target_seq_no = cur_target.target_seq_no +1;
				
				cur_target_seq_no = cur_target.target_seq_no+1;

				next_target = path[cur_target_seq_no];

				goto_cmd.request.value = true;

				goto_cmd.request.target_seq_no = cur_target_seq_no; // target seq no 설정
				
				goto_cmd.request.is_global = next_target.is_global;

				goto_cmd.request.x_lat = next_target.x_lat;
				
				goto_cmd.request.y_long = next_target.y_long;
				
				goto_cmd.request.z_alt = next_target.z_alt;
				
				goto_client.call(goto_cmd); // 새로운 경유지로 goto service 호출	
			} 

			else
				break;
		}


		ros::spinOnce();
		rate.sleep();

		// 경유지 추가 (필요 시)

	}


	// 더 이상 남은 목적지가 없으면 HOME 위치로 복귀
	
		


	return 0;
} 
