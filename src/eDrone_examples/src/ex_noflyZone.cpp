

// 2018.03.07
// Sooyoung Moon
// example appication for testing nofly service

#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_msgs/MissionAddItem.h>
#include <eDrone_msgs/MissionUpload.h>
#include <eDrone_msgs/MissionDownload.h>
#include <eDrone_msgs/Geofence.h>
#include <eDrone_msgs/NoflyZoneSet.h>
#include <eDrone_msgs/NoflyZoneReset.h>
#include <eDrone_msgs/NoflyZoneCheck.h>

using namespace std;
using namespace eDrone_msgs;


// 서비스 요청 메시지

eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::MissionAddItem missionAddItem_cmd;
eDrone_msgs::MissionUpload missionUpload_cmd;
eDrone_msgs::MissionDownload missionDownload_cmd;
eDrone_msgs::Geofence geofence_cmd;
eDrone_msgs::NoflyZoneSet noflyZoneSet_cmd;
eDrone_msgs::NoflyZoneReset noflyZoneReset_cmd;
eDrone_msgs::NoflyZoneCheck noflyZoneCheck_cmd;


int main(int argc, char** argv)
{
  printf("==ex_noflyZone==\n");
  
  ros::init(argc, argv, "ex_noflyZone");
  ros::NodeHandle nh;

   
  // service client

  ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  
  ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition"); 
  ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");

  ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");

  ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");

  ros::ServiceClient missionAddItem_client =nh.serviceClient<eDrone_msgs::MissionAddItem>("srv_missionAddItem");

  ros::ServiceClient geofence_client = nh.serviceClient<eDrone_msgs::Geofence>("srv_geofence");

  ros::ServiceClient missionUpload_client =nh.serviceClient<eDrone_msgs::MissionUpload>("srv_missionUpload");

  ros::ServiceClient missionDownload_client =nh.serviceClient<eDrone_msgs::MissionDownload>("srv_missionDownload");
 
  ros::ServiceClient noflyZoneSet_client = nh.serviceClient<eDrone_msgs::NoflyZoneSet>("srv_noflyZoneSet");

  ros::ServiceClient noflyZoneReset_client = nh.serviceClient<eDrone_msgs::NoflyZoneReset>("srv_noflyZoneReset");

  ros::ServiceClient noflyZoneCheck_client = nh.serviceClient<eDrone_msgs::NoflyZoneCheck>("srv_noflyZoneCheck");


 ros::Rate rate(20.0);

 sleep(10);

  //// 연결 상태 학인

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


  //// 현재 위치 확인
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

	ROS_INFO("Send arming command ... \n");
	arming_client.call(arming_cmd);
	ROS_INFO("Arming command was sent\n");

  int loopCnt = 0;


  // noflyZoneSet 서비스 호출

  


  /* 
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
*/

  //// Geofence
/*
  printf("Send geofence command ...\n");

  geofence_cmd.request.value = true;
  geofence_cmd.request.radius = 200;
  geofence_client.call(geofence_cmd);
*/

  //// NoflyZone


  /*
  printf("Send NoflyZone command ...\n");
  
  noflyZone_cmd.request.value = true;
  noflyZone_cmd.request.is_global = true;
  noflyZone_cmd.request.min_x_lat = 36.3842751;
  noflyZone_cmd.request.min_y_long = 127.3684272;
  
  noflyZone_cmd.request.max_x_lat = 36.3852751;
  noflyZone_cmd.request.max_y_long = 127.3694272;
 
  noflyZone_client.call(noflyZone_cmd);

  sleep(5);
 */

/*
  //// MissionAddItem
  loopCnt = 0; 
  printf("Send missionAddItem command ... \n");

  int globalFrame = 3;
  int myFrame = 3;


  // mission item - wp1 - takeoff
   {
     	missionAddItem_cmd.request.frame = 3;

	//missionAddItem_cmd.request.frame = FRAME_LOCAL_NED;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_TAKEOFF;
	missionAddItem_cmd.request.is_global = true;
		
	missionAddItem_cmd.request.is_current = 1;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	//missionAddItem_cmd.request.x_lat = 36.3833999;
	//missionAddItem_cmd.request.y_long = 127.3661762;
	missionAddItem_cmd.request.z_alt = 50;
	
	missionAddItem_client.call(missionAddItem_cmd);

	if (missionAddItem_cmd.response.value == true)
	{
		printf("missionAddItem: success!\n");
	}
	else
	{
		printf("missionAddItem: failure!\n");
	}
   }   


  
  // mission item - wp2 
   {
	//missionAddItem_cmd.request.frame = FRAME_LOCAL_NED;
     	missionAddItem_cmd.request.frame = myFrame;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	missionAddItem_cmd.request.is_global = true;
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	missionAddItem_cmd.request.x_lat = 36.3847751;
	missionAddItem_cmd.request.y_long = 127.3661762;
	missionAddItem_cmd.request.z_alt = 50;
	
	missionAddItem_client.call(missionAddItem_cmd);
	
	if (missionAddItem_cmd.response.value == true)
	{
		printf("missionAddItem: success!\n");
	}
	else
	{
		printf("missionAddItem: failure!\n");
	}
   }   

  
  // mission item - wp3
   {
     	missionAddItem_cmd.request.frame = myFrame;
     	//missionAddItem_cmd.request.frame = FRAME_LOCAL_NED;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	missionAddItem_cmd.request.is_global = true;
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	missionAddItem_cmd.request.x_lat = 36.3847751;
	missionAddItem_cmd.request.y_long = 127.3689272;
	missionAddItem_cmd.request.z_alt = 50;
	
	missionAddItem_client.call(missionAddItem_cmd);
	
	if (missionAddItem_cmd.response.value == true)
	{
		printf("missionAddItem: success!\n");
	}
	else
	{
		printf("missionAddItem: failure!\n");
	}
   }

  // mission item - wp4
   {
	
     	missionAddItem_cmd.request.frame = myFrame;
     	//missionAddItem_cmd.request.frame = FRAME_LOCAL_NED ;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	missionAddItem_cmd.request.is_global = true;
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	missionAddItem_cmd.request.x_lat = 36.3833999;
	missionAddItem_cmd.request.y_long = 127.3689272;
	missionAddItem_cmd.request.z_alt = 50;
	
	missionAddItem_client.call(missionAddItem_cmd);
	
	if (missionAddItem_cmd.response.value == true)
	{
		printf("missionAddItem: success!\n");
	}
	else
	{
		printf("missionAddItem: failure!\n");
	}

   }



  // mission item - wp5
  {

     	//missionAddItem_cmd.request.frame = FRAME_LOCAL_NED;
     	missionAddItem_cmd.request.frame = myFrame;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	missionAddItem_cmd.request.is_global = true;
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	
	missionAddItem_cmd.request.x_lat = 36.3833999;
	missionAddItem_cmd.request.y_long = 127.3661762;
	missionAddItem_cmd.request.z_alt = 50;
	
	missionAddItem_client.call(missionAddItem_cmd);
	
	if (missionAddItem_cmd.response.value == true)
	{
		printf("missionAddItem: success!\n");
	}
	else
	{
		printf("missionAddItem: failure!\n");
	}
  }

  // mission item - wp (Land)
  {

     	missionAddItem_cmd.request.frame = 3;
     	//missionAddItem_cmd.request.frame = FRAME_LOCAL_NED;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_LAND;

	missionAddItem_cmd.request.is_global = true;	
	missionAddItem_cmd.request.is_current = 0;
	missionAddItem_cmd.request.autocontinue = 1;
	
	//missionAddItem_cmd.request.param1 = 0;
	//missionAddItem_cmd.request.param2 = 0;
	//missionAddItem_cmd.request.param3 = 0;
	
	missionAddItem_client.call(missionAddItem_cmd);
	
	if (missionAddItem_cmd.response.value == true)
	{
		printf("missionAddItem: success!\n");
	}
	else
	{
		printf("missionAddItem: failure!\n");
	}
  }
  ros::spinOnce();

  sleep(10);

  // mission item - RTL
 
  {

	missionAddItem_cmd.request.frame = 3;
	missionAddItem_cmd.request.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
	missionAddItem_client.call(missionAddItem_cmd);

  }




 
  while (ros::ok()&& loopCnt <1)
  {
	printf("Send missionAddItem command ... \n");

	if (!missionAddItem_client.call(missionAddItem_cmd))
	{
		ros::spinOnce();
		rate.sleep();
	}
	else break;
	loopCnt++;
  }


  ROS_INFO("missionAddItem command was sent\n");


  //// missionUpload


  loopCnt = 0;

 
  while (ros::ok()&& loopCnt <1)
  {
	printf("Send missionUpload command ... \n");

	if (!missionUpload_client.call(missionUpload_cmd))
	{
		ros::spinOnce();
		rate.sleep();
	}
	else break;
	loopCnt++;
  }

  ROS_INFO("missionUpload command was sent\n");

*/

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
  */

  ros::spinOnce();
  rate.sleep(); 

  return 0;
}


