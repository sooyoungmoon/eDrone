

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
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/CheckState.h>


using namespace std;

eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::CheckState chkState_cmd;
eDrone_msgs::CheckPosition chkPosition_cmd;

int main(int argc, char** argv)
{
  printf("==ex_simpleCheck==\n");
  
  ros::init(argc, argv, "ex_simpleCheck");
  ros::NodeHandle nh;

  
  // service client
  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");

  
  ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");

  ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");

  
  ros::ServiceClient chkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");

  ros::ServiceClient chkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition");
  ros::Rate rate(20.0);

  // service 요청 메시지 필드 설정
 
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
  ROS_INFO("Takeoff command was sent\n");
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
  ROS_INFO("Landing command was sent\n");


  // check UAV state

  printf("send chkState command...\n");


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

