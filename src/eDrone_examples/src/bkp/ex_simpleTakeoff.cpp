

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>


//using namespace Mission_API;


eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;


int main(int argc, char** argv)
{
  printf("==ex_simpleTakeoff==\n");
  
  ros::init(argc, argv, "ex_simpleTakeoff");
  ros::NodeHandle nh;

  //setup(argc, argv, "ex_simpleTakeoff");
  
  // service client
  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");

  
  ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");

  ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");

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
  ROS_INFO("Arming command was sent\n");

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
  ROS_INFO("Takeoff command was sent\n");
  
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
  ROS_INFO("Landing command was sent\n");


  ros::spinOnce();
  rate.sleep(); 

  return 0;
}

