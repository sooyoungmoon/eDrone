
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
#include "mission_lib/Arming.h"
#include "mission_lib/Takeoff.h"
#include "mission_lib/Landing.h"
#include "mission_lib/WaypointAdd.h"
#include "mission_lib/WaypointPush.h"
#include "mission_lib/WaypointPull.h"
#include "mission_lib/Geofence.h"


//using namespace Mission_API;


mission_lib::Arming arming_cmd;
mission_lib::Takeoff takeoff_cmd;
mission_lib::Landing landing_cmd;

mission_lib::WaypointAdd waypointAdd_cmd;
mission_lib::WaypointPush waypointPush_cmd;
mission_lib::WaypointPull waypointPull_cmd;
mission_lib::Geofence geofence_cmd;


int main(int argc, char** argv)
{
  printf("==ex_wpGeofence==\n");
  
  ros::init(argc, argv, "ex_wpGeofence");
  ros::NodeHandle nh;

  //setup(argc, argv, "ex_simpleTakeoff");
  
  // service client
  ros::ServiceClient arming_client =nh.serviceClient<mission_lib::Arming>("srv_arming");

  
  ros::ServiceClient takeoff_client =nh.serviceClient<mission_lib::Takeoff>("srv_takeoff");

  ros::ServiceClient landing_client =nh.serviceClient<mission_lib::Landing>("srv_landing");


  ros::ServiceClient waypointAdd_client =nh.serviceClient<mission_lib::WaypointAdd>("srv_waypointAdd");

  ros::ServiceClient geofence_client = nh.serviceClient<mission_lib::Geofence>("srv_geofence");


  ros::ServiceClient waypointPush_client =nh.serviceClient<mission_lib::WaypointPush>("srv_waypointPush");

  ros::ServiceClient waypointPull_client =nh.serviceClient<mission_lib::WaypointPull>("srv_waypointPull");
  ros::Rate rate(20.0);

  // service 요청 메시지 필드 설정
  arming_cmd.request.value = true;

  takeoff_cmd.request.value = true;

  landing_cmd.request.value = true; 
  /*
  waypointAdd_cmd.request.value = true;
  waypointAdd_cmd.request.latitude = 10;
  
  waypointAdd_cmd.request.longitude = 10;

  waypointAdd_cmd.request.altitude = 10;
  waypointPush_cmd.request.value = true;
  */
 
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
  /* 
  ROS_ERROR("Arming command was sent\n");

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

  //// Geofence

  printf("Send geofence command ...\n");

  geofence_cmd.request.value = true;
  geofence_cmd.request.radius = 200;
  geofence_client.call(geofence_cmd);

  
  //// WaypointAdd
  loopCnt = 0; 
  printf("Send waypointAdd command ... \n");

  int globalFrame = 3;
  int myFrame = 3;


  // mission item - wp1 - takeoff
   {
     	waypointAdd_cmd.request.frame = 3;

	//waypointAdd_cmd.request.frame = FRAME_LOCAL_NED;
        waypointAdd_cmd.request.command = MAV_CMD_NAV_TAKEOFF;
	waypointAdd_cmd.request.is_global = true;
		
	waypointAdd_cmd.request.is_current = 1;
	waypointAdd_cmd.request.autocontinue = 1;
	waypointAdd_cmd.request.param1 = 0;
	waypointAdd_cmd.request.param2 = 0;
	waypointAdd_cmd.request.param3 = 0;
	
	//waypointAdd_cmd.request.x_lat = 36.3833999;
	//waypointAdd_cmd.request.y_long = 127.3661762;
	waypointAdd_cmd.request.z_alt = 50;
	
	waypointAdd_client.call(waypointAdd_cmd);

	if (waypointAdd_cmd.response.value == true)
	{
		printf("waypointAdd: success!\n");
	}
	else
	{
		printf("waypointAdd: failure!\n");
	}
   }   


  
  // mission item - wp2 
   {
	//waypointAdd_cmd.request.frame = FRAME_LOCAL_NED;
     	waypointAdd_cmd.request.frame = myFrame;
        waypointAdd_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	waypointAdd_cmd.request.is_global = true;
	waypointAdd_cmd.request.is_current = 0;
	waypointAdd_cmd.request.autocontinue = 1;
	waypointAdd_cmd.request.param1 = 0;
	waypointAdd_cmd.request.param2 = 0;
	waypointAdd_cmd.request.param3 = 0;
	
	waypointAdd_cmd.request.x_lat = 36.3847751;
	waypointAdd_cmd.request.y_long = 127.3661762;
	waypointAdd_cmd.request.z_alt = 50;
	
	waypointAdd_client.call(waypointAdd_cmd);
	
	if (waypointAdd_cmd.response.value == true)
	{
		printf("waypointAdd: success!\n");
	}
	else
	{
		printf("waypointAdd: failure!\n");
	}
   }   

  
  // mission item - wp3
   {
     	waypointAdd_cmd.request.frame = myFrame;
     	//waypointAdd_cmd.request.frame = FRAME_LOCAL_NED;
        waypointAdd_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	waypointAdd_cmd.request.is_global = true;
	waypointAdd_cmd.request.is_current = 0;
	waypointAdd_cmd.request.autocontinue = 1;
	waypointAdd_cmd.request.param1 = 0;
	waypointAdd_cmd.request.param2 = 0;
	waypointAdd_cmd.request.param3 = 0;
	
	waypointAdd_cmd.request.x_lat = 36.3847751;
	waypointAdd_cmd.request.y_long = 127.3689272;
	waypointAdd_cmd.request.z_alt = 50;
	
	waypointAdd_client.call(waypointAdd_cmd);
	
	if (waypointAdd_cmd.response.value == true)
	{
		printf("waypointAdd: success!\n");
	}
	else
	{
		printf("waypointAdd: failure!\n");
	}
   }

  // mission item - wp4
   {
	
     	waypointAdd_cmd.request.frame = myFrame;
     	//waypointAdd_cmd.request.frame = FRAME_LOCAL_NED ;
        waypointAdd_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	waypointAdd_cmd.request.is_global = true;
	waypointAdd_cmd.request.is_current = 0;
	waypointAdd_cmd.request.autocontinue = 1;
	waypointAdd_cmd.request.param1 = 0;
	waypointAdd_cmd.request.param2 = 0;
	waypointAdd_cmd.request.param3 = 0;
	
	waypointAdd_cmd.request.x_lat = 36.3833999;
	waypointAdd_cmd.request.y_long = 127.3689272;
	waypointAdd_cmd.request.z_alt = 50;
	
	waypointAdd_client.call(waypointAdd_cmd);
	
	if (waypointAdd_cmd.response.value == true)
	{
		printf("waypointAdd: success!\n");
	}
	else
	{
		printf("waypointAdd: failure!\n");
	}

   }



  // mission item - wp5
  {

     	//waypointAdd_cmd.request.frame = FRAME_LOCAL_NED;
     	waypointAdd_cmd.request.frame = myFrame;
        waypointAdd_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	
	waypointAdd_cmd.request.is_global = true;
	waypointAdd_cmd.request.is_current = 0;
	waypointAdd_cmd.request.autocontinue = 1;
	waypointAdd_cmd.request.param1 = 0;
	waypointAdd_cmd.request.param2 = 0;
	waypointAdd_cmd.request.param3 = 0;
	
	waypointAdd_cmd.request.x_lat = 36.3833999;
	waypointAdd_cmd.request.y_long = 127.3661762;
	waypointAdd_cmd.request.z_alt = 50;
	
	waypointAdd_client.call(waypointAdd_cmd);
	
	if (waypointAdd_cmd.response.value == true)
	{
		printf("waypointAdd: success!\n");
	}
	else
	{
		printf("waypointAdd: failure!\n");
	}
  }

  // mission item - wp (Land)
  {

     	waypointAdd_cmd.request.frame = 3;
     	//waypointAdd_cmd.request.frame = FRAME_LOCAL_NED;
        waypointAdd_cmd.request.command = MAV_CMD_NAV_LAND;

	waypointAdd_cmd.request.is_global = true;	
	waypointAdd_cmd.request.is_current = 0;
	waypointAdd_cmd.request.autocontinue = 1;
	
	//waypointAdd_cmd.request.param1 = 0;
	//waypointAdd_cmd.request.param2 = 0;
	//waypointAdd_cmd.request.param3 = 0;
	
	waypointAdd_client.call(waypointAdd_cmd);
	
	if (waypointAdd_cmd.response.value == true)
	{
		printf("waypointAdd: success!\n");
	}
	else
	{
		printf("waypointAdd: failure!\n");
	}
  }
  ros::spinOnce();

  sleep(10);

  // mission item - RTL
 
  {
/*
	waypointAdd_cmd.request.frame = 3;
	waypointAdd_cmd.request.command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
	waypointAdd_client.call(waypointAdd_cmd);
*/
  }




/* 
  while (ros::ok()&& loopCnt <1)
  {
	printf("Send waypointAdd command ... \n");

	if (!waypointAdd_client.call(waypointAdd_cmd))
	{
		ros::spinOnce();
		rate.sleep();
	}
	else break;
	loopCnt++;
  }
*/

  ROS_ERROR("WaypointAdd command was sent\n");


  //// WaypointPush


  loopCnt = 0;

 
  while (ros::ok()&& loopCnt <1)
  {
	printf("Send waypointPush command ... \n");

	if (!waypointPush_client.call(waypointPush_cmd))
	{
		ros::spinOnce();
		rate.sleep();
	}
	else break;
	loopCnt++;
  }

  ROS_ERROR("WaypointPush command was sent\n");



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
  */

  ros::spinOnce();
  rate.sleep(); 

  return 0;
}

