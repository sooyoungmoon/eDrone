
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_msgs/Goto.h>
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/Survey.h>


eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::Goto goto_cmd;
eDrone_msgs::Survey survey_cmd;


// 메시지 변수 선언

eDrone_msgs::Target cur_target;

// 콜백 함수 
void cur_target_cb(const eDrone_msgs::Target::ConstPtr& msg)
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
	printf("==ex_simpleSurvey==\n");

	ros::init(argc, argv, "ex_simpleSurvey");
	ros::NodeHandle nh;

		
	// subscriber 선언
	ros::Subscriber cur_target_sub= nh.subscribe("eDrone_msgs/current_target", 10, cur_target_cb); // 
	
	ros::Rate rate(20.0);

	// service client 선언
	
	ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");
	ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
	ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");


	ros::ServiceClient survey_client =nh.serviceClient<eDrone_msgs::Survey>("srv_survey");

	sleep(20);
	

	//// Arming

	printf("Send arming command ... \n");
	arming_cmd.request.value = true;
	arming_client.call(arming_cmd);
	ROS_INFO("Arming command was sent\n");



	//// Takeoff

	printf("Send takeoff command ... \n");
	takeoff_cmd.request.value = true;
	takeoff_client.call(takeoff_cmd);
	ROS_INFO("Takeoff command was sent\n");

	sleep(10);
	
	//// Survey

	printf("Send survey command ... \n");
	survey_cmd.request.value = true;
	survey_cmd.request.is_global = false;
	survey_cmd.request.min_x_lat = 0;
	
	survey_cmd.request.min_y_long = 0;
	survey_cmd.request.max_x_lat = 100;
	survey_cmd.request.max_y_long =100;
	survey_cmd.request.row_col_width = 10;

	survey_client.call(survey_cmd);	
	ROS_INFO("Survey command was sent\n");

	//// Goto

	/*
	printf("Send goto command ...\n");
	printf("let's start a mission! \n");


	goto_cmd.request.value = true;
	goto_cmd.request.is_global = false;
	goto_cmd.request.x_lat = 0;
	goto_cmd.request.y_long = 100;
	goto_cmd.request.z_alt = 50;
	
	goto_client.call(goto_cmd);
	ROS_INFO("Goto command was sent\n");
	*/
	// 주기적으로 자율 비행 경로에 새로운 목적지 (경유지) 정보 추가
	// 현재 목적지에 도달한 경우 더 남아 있는 경유지가 있다면 새로운 목적지로 goto service 호출

	while(ros::ok())
	{



		ros::spinOnce();
		rate.sleep();

		// 경유지 추가 (필요 시)

	}



	return 0;
} 
