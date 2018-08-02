

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_examples/params.h>

using namespace std;

eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;

int main(int argc, char** argv)
{
  printf("==ex_takeoff==\n");  

  
  if (argc <2)
  {
   ROS_ERROR("usage: ex_takeoff <altitude> " );
   return -1; 
  }
  
  
  ros::init(argc, argv, "ex_takeoff");
  ros::NodeHandle nh;

  ROS_INFO("Number of arguments: %d", argc);

  for (int i = 0; i < argc; i++)
  {
	ROS_INFO ("%s", argv[i]);
  }
  
  // service client
  ros::ServiceClient checkState_client = nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");

  ros::ServiceClient checkPosition_client = nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition");
  
  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");  
  ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
  ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
  
  sleep(10); 


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

  if (arming_client.call(arming_cmd))
	ROS_INFO("Arming command was sent\n");

  //// Takeoff

  double altitude = atof (argv[1]);

  // service 요청 메시지 필드 설정
  takeoff_cmd.request.altitude = altitude;  
  
  if (takeoff_client.call(takeoff_cmd))
	ROS_INFO("Takeoff command was sent\n");

  // Do something (mission)
  sleep(10);

  //// Landing
 
  while ( landing_cmd.response.value != true)
  {
	landing_client.call(landing_cmd);

  }
	ROS_INFO("Landing command was sent\n");
 
  return 0;
}

