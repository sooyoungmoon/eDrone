

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
#include "mission_lib_msgs/CheckState.h"
#include "mission_lib_msgs/CheckPosition.h"
#include "mission_lib_msgs/CheckState.h"


//using namespace Mission_API;
using namespace std;

mission_lib_msgs::Arming arming_cmd;
mission_lib_msgs::Takeoff takeoff_cmd;
mission_lib_msgs::Landing landing_cmd;
mission_lib_msgs::CheckState chkState_cmd;
mission_lib_msgs::CheckPosition chkPosition_cmd;

int main(int argc, char** argv)
{
  printf("==ex_simpleCheck==\n");
  
  ros::init(argc, argv, "ex_simpleCheck");
  ros::NodeHandle nh;

  
  // service client
  ros::ServiceClient arming_client =nh.serviceClient<mission_lib_msgs::Arming>("srv_arming");

  
  ros::ServiceClient takeoff_client =nh.serviceClient<mission_lib_msgs::Takeoff>("srv_takeoff");

  ros::ServiceClient landing_client =nh.serviceClient<mission_lib_msgs::Landing>("srv_landing");

  
  ros::ServiceClient chkState_client =nh.serviceClient<mission_lib_msgs::CheckState>("srv_chkState");

  ros::ServiceClient chkPosition_client =nh.serviceClient<mission_lib_msgs::CheckPosition>("srv_chkPosition");
  ros::Rate rate(20.0);

  // service 요청 메시지 필드 설정
  arming_cmd.request.value = true;

  takeoff_cmd.request.value = true;

  landing_cmd.request.value = true; 
 
  //// Arming
  

  int loopCnt = 0;

  while (ros::ok()&& loopCnt <1 )
 {
	printf("Send arming command ... \n");
	
	if (!arming_client.call(arming_cmd))
	{
	  ros::spinOnce();
	  rate.sleep();
	}
	else break;
	loopCnt++;
  } 
  ROS_ERROR("Arming command was sent\n");
/*
  loopCnt=0; 
  while (ros::ok()&& loopCnt <1 )
 {
	printf("send takeoff command ... \n");
	
	if (!takeoff_client.call(takeoff_cmd))
	{
	  ros::spinOnce();
	  rate.sleep();
	}
	else break;
	loopCnt++;
  } 
  ROS_ERROR("Takeoff command was sent\n");
  */
  // Do something (mission)
  sleep(10);
	
  loopCnt = 0;
  while (ros::ok()&& loopCnt <1 )
 {
	printf("send landing  command ... \n");
	
	if (!landing_client.call(landing_cmd))
	{
	  ros::spinOnce();
	  rate.sleep();
	}
	else break;
	loopCnt++;
  } 
  ROS_ERROR("Landing command was sent\n");


  // check UAV state

  printf("send chkState command...\n");

  chkState_cmd.request.value = true;

  if (chkState_client.call(chkState_cmd))
  {
 	ROS_INFO("ChkState command was sent\n");
	
	if (chkState_cmd.response.armed == true)
	{
		printf("uav is armed\n");
	}
	else
	{
		printf("uav is unarmed\n");
	}

	if (chkState_cmd.response.connected == true)

	{
		printf("uav is connected\n");
	}
	else
	{
		printf("uav is connected\n");
	}
	if (chkState_cmd.response.mode.empty() == false)
	{
		std::cout << "flight mode: " << chkState_cmd.response.mode << endl;
	}
	else
	{
		printf("flight mode: unknown\n") ;
	}
  }

  

  // check UAV position

  
  printf("send chkPosition command...\n");

  chkPosition_cmd.request.value = true;

  if (chkPosition_client.call(chkPosition_cmd))
  {
	cout <<"(UAV position)" << endl;

	cout <<"global frame: (" << chkPosition_cmd.response.latitude << ", " << chkPosition_cmd.response.longitude << ", " << chkPosition_cmd.response.altitude << ") " << endl << endl;

	cout <<"local frame: (" << chkPosition_cmd.response.x << ", " << chkPosition_cmd.response.y << ", " << chkPosition_cmd.response.z << ") " << endl; 
  }
  ros::spinOnce();
  rate.sleep(); 

  return 0;
}

