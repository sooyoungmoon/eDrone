

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
//#include <eDrone_lib/GeoUtils.h>
//#include <eDrone_lib/GeoInfo.h>

using namespace std;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;

int main(int argc, char** argv)
{
  printf("==ex_check==\n");
  
  ros::init(argc, argv, "ex_check");
  ros::NodeHandle nh;

  
  // service client
  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");

  
  ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");

  ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");

  
  ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");

  ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition");
  ros::Rate rate(20.0);

  // service 요청 메시지 필드 설정

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
                        sleep(10);
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
                        sleep(10);
                }


                cout <<"global frame: (" << checkPosition_cmd.response.latitude << ", " << checkPosition_cmd.response.longitude << ", " << checkPosition_cmd.response.altitude << ") " << endl << endl;

                cout <<"local frame: (" << checkPosition_cmd.response.x << ", " << checkPosition_cmd.response.y << ", " << checkPosition_cmd.response.z << ") " << endl;


        }

	ROS_INFO("UAV position was checked!");

  sleep(10);
 
  //// Arming
  

  int loopCnt = 0;

  while (ros::ok()&& loopCnt <1 )
 {
	printf("ex_check: Send arming command ... \n");
	
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

  printf("send checkState command...\n");

 

  if (checkState_client.call(checkState_cmd))
  {
 	ROS_INFO("checkState command was sent\n");

	printf("(UAV Status)\n\n");
	
	if (checkState_cmd.response.armed == true)
	{
		printf("uav is armed\n");
	}
	else
	{
		printf("uav is unarmed\n");
	}

	if (checkState_cmd.response.connected == true)

	{
		printf("uav is connected\n");
	}
	else
	{
		printf("uav is connected\n");
	}
	if (checkState_cmd.response.mode.empty() == false)
	{
		std::cout << "flight mode: " << checkState_cmd.response.mode << endl;
	}
	else
	{
		printf("flight mode: unknown\n") ;
	}
  }

  

  // check UAV position

  
  printf("\nsend checkPosition command...\n");

 

  if (checkPosition_client.call(checkPosition_cmd))
  {
	cout << endl<< "(UAV position)" << endl << endl;

	cout <<"global frame: (" << checkPosition_cmd.response.latitude << ", " << checkPosition_cmd.response.longitude << ", " << checkPosition_cmd.response.altitude << ") " << endl ;

	cout <<"local frame: (" << checkPosition_cmd.response.x << ", " << checkPosition_cmd.response.y << ", " << checkPosition_cmd.response.z << ") " << endl; 
  }
  
  
  ros::spinOnce();
  rate.sleep(); 

  return 0;
}

