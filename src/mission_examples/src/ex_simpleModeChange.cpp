
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
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
#include "mission_lib_msgs/ModeChange.h"



mission_lib_msgs::Arming arming_cmd;
mission_lib_msgs::Takeoff takeoff_cmd;
mission_lib_msgs::Landing landing_cmd;
mission_lib_msgs::Goto goto_cmd;
mission_lib_msgs::ModeChange modeChange_cmd;


int main(int argc, char** argv)
{
  printf("==ex_simpleModeChange==\n");
  
  ros::init(argc, argv, "ex_ModeChange");
  ros::NodeHandle nh;

  
  // service client
  ros::ServiceClient arming_client =nh.serviceClient<mission_lib_msgs::Arming>("srv_arming");

  
  ros::ServiceClient takeoff_client =nh.serviceClient<mission_lib_msgs::Takeoff>("srv_takeoff");

  ros::ServiceClient landing_client =nh.serviceClient<mission_lib_msgs::Landing>("srv_landing");

  ros::ServiceClient goto_client = nh.serviceClient<mission_lib_msgs::Goto>("srv_goto");

  ros::ServiceClient modeChange_client = nh.serviceClient<mission_lib_msgs::ModeChange>("srv_modeChange");
  
  ros::Rate rate(20.0);

  // service 요청 메시지 필드 설정
  arming_cmd.request.value = true;
  takeoff_cmd.request.value = true;
  landing_cmd.request.value = true; 
  goto_cmd.request.value = true;
  modeChange_cmd.request.value = true;
  
 
  //// Arming
  


  {
	printf("Send arming command ... \n");
	
	if (!arming_client.call(arming_cmd))
	{
	  ros::spinOnce();
	  rate.sleep();
	}
  }
   
  ROS_ERROR("Arming command was sent\n");


 

 {
	printf("send takeoff command ... \n");
	
	if (!takeoff_client.call(takeoff_cmd))
	{
	  ros::spinOnce();
	  rate.sleep();
	}
	

  } 

  ROS_ERROR("Takeoff command was sent\n");

  sleep(10);
 

  modeChange_cmd.request.mode.assign("AUTO.MISSION"); 
//  modeChange_cmd.request.mode.assign("OFFBOARD"); 
//  modeChange_cmd.request.mode.assign("AUTO.RTL"); 
  
  if ( modeChange_client.call(modeChange_cmd) == true)
  { 
	ROS_INFO("modeChange command was sent");
  }

/*

  {
	printf("send goto command ...\n");

	goto_cmd.request.value = true;

	goto_cmd.request.is_global = false;

	goto_cmd.request.x_lat = 0;

	goto_cmd.request.y_long = 100;

	goto_cmd.request.z_alt = 50;        

	goto_client.call(goto_cmd);
  } 
  

  ROS_ERROR("Goto command was sent\n");
 */ 
  while (ros::ok())
  {
  	ros::spinOnce();
  	rate.sleep(); 
  }
  return 0;
}

