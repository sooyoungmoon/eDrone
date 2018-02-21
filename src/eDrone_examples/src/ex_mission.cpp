
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Waypoint.h>
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_msgs/RTL.h>
#include <eDrone_msgs/MissionAddItem.h>
#include <eDrone_msgs/MissionUpload.h>
#include <eDrone_msgs/MissionDownload.h>
#include <eDrone_msgs/MissionClear.h>

using namespace std;
using namespace mavros_msgs;


eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::RTL rtl_cmd;

eDrone_msgs::MissionAddItem missionAddItem_cmd;
eDrone_msgs::MissionUpload missionUpload_cmd;
eDrone_msgs::MissionDownload missionDownload_cmd;
eDrone_msgs::MissionClear missionClear_cmd;

vector<mavros_msgs::Waypoint> waypoints;

void print_waypoints (vector<mavros_msgs::Waypoint> waypoints) // 웨이포인트 정보
{
        for (int i = 0; i < waypoints.size(); i++)
//      for (vector<mavros_msgs::Waypoint>::iterator it = std::begin(waypoints); it! = std::end(waypoints); it++)
        {
                cout << "waypoint[" << i << "]: ";

                cout << endl ;

                cout << waypoints[i] << endl;
        }
}


int main(int argc, char** argv)
{
  printf("==ex_mission==\n");
  
  ros::init(argc, argv, "ex_mission");
  ros::NodeHandle nh;

  
  // service client


  ros::ServiceClient checkState_client = nh.serviceClient<eDrone_msgs::CheckState> ("srv_checkState");

  
  ros::ServiceClient checkPosition_client = nh.serviceClient<eDrone_msgs::CheckPosition> ("srv_checkPosition");

  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");
  ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
  ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
  ros::ServiceClient rtl_client =nh.serviceClient<eDrone_msgs::RTL>("srv_rtl");
  ros::ServiceClient missionAddItem_client =nh.serviceClient<eDrone_msgs::MissionAddItem>("srv_missionAddItem");
  ros::ServiceClient missionUpload_client =nh.serviceClient<eDrone_msgs::MissionUpload>("srv_missionUpload");
  ros::ServiceClient missionDownload_client =nh.serviceClient<eDrone_msgs::MissionDownload>("srv_missionDownload");
  ros::ServiceClient missionClear_client =nh.serviceClient<eDrone_msgs::MissionClear>("srv_missionClear");

  ros::Rate rate(20.0);

  // service 요청 메시지 필드 설정
 
  sleep(10);

  // flight controller 연결 확인

  ROS_INFO("Send checkState command ... \n");
  ROS_INFO("Checking the connection ... \n");

	
  if (checkState_client.call(checkState_cmd))
  {
	ROS_INFO ("CheckState service was requested");
		
	while (checkState_cmd.response.connected == false)
	{
		if (checkState_client.call(checkState_cmd))
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
		if (checkPosition_client.call(checkPosition_cmd))
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
  


 
  printf("Send arming command ... \n");
	
  if (arming_client.call(arming_cmd))
  {
	 printf("Arming command was sent!\n");
  }
	
  
  //// MissionDownload

  printf("Send missionDownload command ... \n");
  
  missionDownload_cmd.request.value = true;

  if (missionDownload_client.call (missionDownload_cmd))
  { 
	printf("missionDownload command was sent\n");
	waypoints = missionDownload_cmd.response.waypoints;

	print_waypoints(waypoints);
  }

   //// MissionClear

   printf("Send missionClear command ... \n");

   {
	missionClear_cmd.request.value = true;
	if (missionClear_client.call(missionClear_cmd))
	{
		printf("missionClear command was sent\n");
	}
   }

  

  

 

  //// MissionAddItem
 
  printf("Send missionAddItem command ... \n");


  // mission item - wp1 - takeoff
   {
     	missionAddItem_cmd.request.frame = 3;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_TAKEOFF;
	
	missionAddItem_cmd.request.is_current = 1;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	//missionAddItem_cmd.request.x_lat = 36.3833999;
	//missionAddItem_cmd.request.y_long = 127.3661762;
	missionAddItem_cmd.request.z_alt = 50;
	
	missionAddItem_client.call(missionAddItem_cmd);
   }   


  
  // mission item - wp2 
   {
     	missionAddItem_cmd.request.frame = 3;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	missionAddItem_cmd.request.is_global = false;
	
	missionAddItem_cmd.request.x_lat = 0;
	missionAddItem_cmd.request.y_long = 50;
	missionAddItem_cmd.request.z_alt = 50;
/*
	missionAddItem_cmd.request.x_lat = 36.3847751;
	missionAddItem_cmd.request.y_long = 127.3661762;
	missionAddItem_cmd.request.z_alt = 50;
*/	
	missionAddItem_client.call(missionAddItem_cmd);
   }   

  
  // mission item - wp3
   {
     	missionAddItem_cmd.request.frame = 3;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	missionAddItem_cmd.request.is_global = false;

	
	missionAddItem_cmd.request.x_lat = 100;
	missionAddItem_cmd.request.y_long = 50;
	missionAddItem_cmd.request.z_alt = 50;
	/*
	missionAddItem_cmd.request.x_lat = 36.3847751;
	missionAddItem_cmd.request.y_long = 127.3689272;
	missionAddItem_cmd.request.z_alt = 50;
	*/
	missionAddItem_client.call(missionAddItem_cmd);
   }

  // mission item - wp4
   {
	
     	missionAddItem_cmd.request.frame = 3;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	missionAddItem_cmd.request.is_global = false;

	
	missionAddItem_cmd.request.x_lat = 100;
	missionAddItem_cmd.request.y_long = 0;
	missionAddItem_cmd.request.z_alt = 50;
	/*
	missionAddItem_cmd.request.x_lat = 36.3833999;
	missionAddItem_cmd.request.y_long = 127.3689272;
	missionAddItem_cmd.request.z_alt = 50;
	*/
	missionAddItem_client.call(missionAddItem_cmd);

   }



  // mission item - wp5
  {

     	missionAddItem_cmd.request.frame = 3;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	missionAddItem_cmd.request.is_global = false;
	
	missionAddItem_cmd.request.x_lat = 0;
	missionAddItem_cmd.request.y_long = 0;
	missionAddItem_cmd.request.z_alt = 50;

	/*
	missionAddItem_cmd.request.x_lat = 36.3833999;
	missionAddItem_cmd.request.y_long = 127.3661762;
	missionAddItem_cmd.request.z_alt = 50;
	*/
	missionAddItem_client.call(missionAddItem_cmd);
  }

  // mission item - wp (Land)
  {

     	missionAddItem_cmd.request.frame = 3;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_LAND;
	
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	
	
	missionAddItem_client.call(missionAddItem_cmd);
  }
  ros::spinOnce();

  sleep(10);


  ROS_INFO("MissionAddItem command was sent\n");


  //// MissionUpload

 
  printf("Send missionUpload command ... \n");
 

  if (!missionUpload_client.call(missionUpload_cmd))
  {
	ROS_INFO("missionUpload command was sent\n");
  }
	

  ros::spinOnce();
  rate.sleep(); 

  return 0;
}

