
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_msgs/Goto.h>
#include <eDrone_msgs/RTL.h>

using namespace std;


eDrone_msgs::Target cur_target;
eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::Goto goto_cmd;
eDrone_msgs::RTL rtl_cmd;

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
  printf("==ex_goto==\n");
  
  ros::init(argc, argv, "ex_goto");
  ros::NodeHandle nh;

  // service client

  ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  
  ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition"); 
  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");  
  ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
  ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
  ros::ServiceClient goto_client = nh.serviceClient<eDrone_msgs::Goto>("srv_goto");
  ros::ServiceClient rtl_client = nh.serviceClient<eDrone_msgs::RTL>("srv_rtl");

  ros::Rate rate(20.0);

  // service 요청 메시지 필드 설정


  sleep(10);

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


	
 sleep(10); 
 
  //// Arming

  {
	printf("Send arming command ... \n");
	
	if (!arming_client.call(arming_cmd))
	{
	  ros::spinOnce();
	  rate.sleep();
	}
  }
   
  ROS_INFO("Arming command was sent\n");

 {
	printf("send takeoff command ... \n");
	
	if (!takeoff_client.call(takeoff_cmd))
	{
	  ros::spinOnce();
	  rate.sleep();
	}
	

  } 

  ROS_INFO("Takeoff command was sent\n");

  sleep(10);
  
  {
	printf("send goto command ...\n");


	goto_cmd.request.is_global = false;

	goto_cmd.request.x_lat = 50;

	goto_cmd.request.y_long = 50;

	goto_cmd.request.z_alt = 20;        

	goto_client.call(goto_cmd);
  } 
  

  ROS_INFO("Goto command was sent\n");
  
  while (ros::ok())
  {
	if (cur_target.reached == true)
	{
		// 복귀 서비스 호출
		ROS_INFO ("Return-to-launch\n");
		rtl_client.call(rtl_cmd);

	}


  	ros::spinOnce();
  	rate.sleep(); 
  }
  return 0;
}

