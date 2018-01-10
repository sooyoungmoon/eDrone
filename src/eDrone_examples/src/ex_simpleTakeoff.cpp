

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>


eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;


int main(int argc, char** argv)
{
  printf("==ex_simpleTakeoff==\n");

  if (argc <2)
  {
   ROS_ERROR("usage: ex_simpleTakeoff <altitude> " );
   return -1; 
  }
  
  double altitude = atof (argv[1]);
  
  ros::init(argc, argv, "ex_simpleTakeoff");
  ros::NodeHandle nh;
  
  // service client
  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");  
  ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
  ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
  

  // service 요청 메시지 필드 설정
  takeoff_cmd.request.altitude = altitude;  

 
  //// Arming 

  if (arming_client.call(arming_cmd))
	ROS_INFO("Arming command was sent\n");

  //// Takeoff

  if (takeoff_client.call(takeoff_cmd))
	ROS_INFO("Takeoff command was sent\n");

  // Do something (mission)
  sleep(10);

  //// Landing
  
  if (landing_client.call(landing_cmd))
	ROS_INFO("Landing command was sent\n");  

  return 0;
}

