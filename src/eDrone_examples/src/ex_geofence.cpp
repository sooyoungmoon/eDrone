
/* include */

// 기본 header (ROS & C/C++)
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <vector>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Vector3Stamped.h>

// topic & service 선언 header (mavros, eDrone)
#include <eDrone_msgs/Target.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_msgs/Goto.h>
#include <eDrone_msgs/MissionAddItem.h>
#include <eDrone_msgs/MissionUpload.h>
#include <eDrone_msgs/MissionClear.h>
#include <eDrone_msgs/RTL.h>
#include <eDrone_msgs/GeofenceSet.h>
#include <eDrone_msgs/GeofenceReset.h>
#include <eDrone_msgs/GeofenceCheck.h>
#include <eDrone_examples/params.h>

using namespace std;
using namespace eDrone_msgs;

// 포인터 변수 선언
eDrone_msgs::Target* cur_target_ptr; // 무인기가 현재 향하고 있는 목적지 (경유지)


// 콜백 함수 
void cur_target_cb(const eDrone_msgs::Target::ConstPtr& msg)
{
	*cur_target_ptr = *msg;

	// 현재 목적지 도달 여부 확인
	printf("cur_target_cb(): \n");
	printf("current target: %d \n", cur_target_ptr->target_seq_no);
	
	if (cur_target_ptr->reached == true)
	{
		printf("we reached at the current target\n");
	} 
}




int main(int argc, char** argv)
{
	// 노드 정보 출력 및 노드&노드핸들 초기화

	ROS_INFO("==ex_geofence==\n");

	ros::init(argc, argv, "ex_geofence");
	ros::NodeHandle nh;

	
	// 명령줄 인자 처리
	

	if (argc < 2)
	{
		ROS_ERROR("ex_geofence: usage: ex_geofence <radius> <latitude> <longitude> <altitude>" );
		return -1;
	}


	for (int arg_index = 0; arg_index < argc; arg_index++)
	{
		ROS_INFO("main arg[%d]: %s", arg_index, argv[arg_index] );
	}

	
	ros::Rate rate(20.0);

	// subscriber 선언

	
	// 토픽 메시지 변수 선언  
	eDrone_msgs::Target cur_target; // 무인기가 현재 향하고 있는 목적지 (경유지)
	cur_target_ptr = &cur_target; // cur_target 변수 주소 저장 
	eDrone_msgs::Target next_target; // 다음 목적지 

	// 서비스 메시지 변수 선언 
	eDrone_msgs::CheckState checkState_cmd;
	eDrone_msgs::CheckPosition checkPosition_cmd;
	eDrone_msgs::Arming arming_cmd;
	eDrone_msgs::Takeoff takeoff_cmd;
	eDrone_msgs::Landing landing_cmd;
	eDrone_msgs::Goto goto_cmd;
	eDrone_msgs::RTL rtl_cmd;
	eDrone_msgs::GeofenceSet geofenceSet_cmd;
	eDrone_msgs::GeofenceReset geofenceReset_cmd;	
	eDrone_msgs::GeofenceCheck geofenceCheck_cmd;
	

	// service client 선언
	
	ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");
	ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition >("srv_checkPosition");
	ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");
	ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
	ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
	ros::ServiceClient goto_client = nh.serviceClient<eDrone_msgs::Goto>("srv_goto");
	ros::ServiceClient rtl_client = nh.serviceClient<eDrone_msgs::RTL>("srv_rtl");
	ros::ServiceClient geofenceSet_client = nh.serviceClient<eDrone_msgs::GeofenceSet>("srv_geofenceSet");		    
	ros::ServiceClient geofenceReset_client = nh.serviceClient<eDrone_msgs::GeofenceReset>("srv_geofenceReset");	
	ros::ServiceClient geofenceCheck_client = nh.serviceClient<eDrone_msgs::GeofenceCheck>("srv_geofenceCheck");	
	int cur_target_seq_no = -1; // 현재 target 순번 (0, 1, 2, ...)

	// 초기 부분 경로 설정 (사각형 경로)
	  // % mission 명령에서 사용되는 waypoint와 구분하기 위해 
	  // 위치 이동 명령에서 사용되는 경유지는 target으로 지칭
	
		sleep(10);


	    // 연결 상태 확인


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


	    // 무인기 위치 확인

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


			cout <<"global frame: (" << checkPosition_cmd.response.latitude << ", ";
			cout << checkPosition_cmd.response.longitude << ", ";
			cout << checkPosition_cmd.response.altitude << ") " << endl << endl;

			cout <<"local frame: (" << checkPosition_cmd.response.x << ", ";
			cout << checkPosition_cmd.response.y << ", " << checkPosition_cmd.response.z << ") " << endl;
			
			}
			
ROS_INFO("UAV position was checked!");


	//// Arming

	printf("ex_geofence: Send arming command ... \n");

	if ( arming_client.call(arming_cmd) == true)
	{
		ROS_INFO("ex_geofence: Arming service call success!!\n");
	}
	else
	{

		ROS_ERROR("ex_geofence: Arming service call has failed!! ");
	}
	sleep(10);	

	//// Takeoff
	//double altitude = atof (argv[1]);
	printf("ex_geofence: Send takeoff command ... \n");

	
	takeoff_cmd.request.altitude = 50;
	
	if (takeoff_client.call(takeoff_cmd) == true)
	{
		ROS_INFO("ex_geofence: Takeoff command was sent\n");
	}
	else
	{		
		ROS_ERROR("ex_geofence: Takeoff service call has failed!! ");
	}
	sleep(10);
	
	
	// 명령줄 인자를 이용한 파라미터 값 설정 
	double geofence_radius = atof (argv[1] );
	ROS_INFO ("geofence_radius: %lf",  geofence_radius);

	// Geofence 설정

	geofenceSet_cmd.request.radius = geofence_radius;
	
	if (geofenceSet_client.call (geofenceSet_cmd) == true)
	{
		ROS_INFO ("ex_geofence: GeofenceSet service was called");
		
		if (geofenceSet_cmd.response.value == true)
		{
			ROS_INFO ("ex_geofence: Geofence was set (radius= %lf) \n", geofence_radius);
		}

	}	

	else
	{
		ROS_ERROR("ex_geofence: GeofenceSet service call has failed!! ");
	}
	


	// Geofence 확인
	double latitude = atof (argv[2] );
	double longitude = atof (argv[3] );
	double altitude = atof (argv[4] );
	geofenceCheck_cmd.request.ref_system = "WGS84";

	geofenceCheck_cmd.request.arg1= latitude;
	geofenceCheck_cmd.request.arg2= longitude;
	geofenceCheck_cmd.request.arg3= altitude;

	/*
	geofenceCheck_cmd.request.arg1= 47.3983993;
	geofenceCheck_cmd.request.arg2= 8.5469999;
	geofenceCheck_cmd.request.arg3= 50;
*/
/*	
	geofenceCheck_cmd.request.arg1= 47.3993490;
	geofenceCheck_cmd.request.arg2= 8.5475136;
	geofenceCheck_cmd.request.arg3= 50;
*/

	if (geofenceCheck_client.call (geofenceCheck_cmd) == true)
	{
		ROS_INFO("GeofenceCheck service was called\n");

		if (geofenceCheck_cmd.response.value ==true)
		{
			if (geofenceCheck_cmd.response.violation == true)
			{
				ROS_INFO("Position (%lf, %lf, %lf) is outside of the geofence!!\n", geofenceCheck_cmd.request.arg1,  geofenceCheck_cmd.request.arg2, geofenceCheck_cmd.request.arg3);

			}
			else
			{
				ROS_INFO("Position (%lf, %lf, %lf) is inside the geofence!!\n", geofenceCheck_cmd.request.arg1,  geofenceCheck_cmd.request.arg2, geofenceCheck_cmd.request.arg3);
				
			}
			
		}
	}
	else
	{
		ROS_ERROR("ex_geofence: GeofenceCheck service call has failed!! ");
	}
	


	// Case#1) Geofence 영역 외부 지점에 대해 missionAddItem 서비스 호출
/*
	cout <<"\nCase#1: MissionAddItem 서비스 요청 (Geofence 외부) " << endl;	
	eDrone_msgs::MissionAddItem missionAddItem_cmd;
	eDrone_msgs::MissionUpload missionUpload_cmd;
	eDrone_msgs::MissionClear missionClear_cmd;
	
	ros::ServiceClient missionAddItem_client = nh.serviceClient<eDrone_msgs::MissionAddItem > ("srv_missionAddItem" );

	ros::ServiceClient missionUpload_client = nh.serviceClient<eDrone_msgs::MissionUpload>("srv_missionUpload");
	ros::ServiceClient missionClear_client =nh.serviceClient<eDrone_msgs::MissionClear>("srv_missionClear");
	
	missionAddItem_cmd.request.frame = 3;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	missionAddItem_cmd.request.is_current = 1;
	missionAddItem_cmd.request.autocontinue = 1;

	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	missionAddItem_cmd.request.is_global = true; 
	
	missionAddItem_cmd.request.x_lat = 47.3983993;
	missionAddItem_cmd.request.y_long = 8.5469999;
	missionAddItem_cmd.request.z_alt = 50;	


	cout << "\n 미션 아이템 추가 명령: ( " << missionAddItem_cmd.request.x_lat << ", " << missionAddItem_cmd.request.y_long << ", " << missionAddItem_cmd.request.z_alt << ")" << endl;

	if (missionAddItem_client.call(missionAddItem_cmd))
	{
		if (missionAddItem_cmd.response.value == true )
		{	
			ROS_INFO("ex_geofence: missionAddItem command success!");

		//// MissionUpload
		 
		  printf("ex_geofence: Send missionUpload command ... \n");
		 

		  if (!missionUpload_client.call(missionUpload_cmd))
		  {
			ROS_INFO("missionUpload command was sent\n");
		  }
		}
		else
		{
			ROS_INFO("ex_geofence: missionAddItem command fail!");

		}
}

	// Case#2) Geofence 영역 외부 지점에 대해 goto 서비스 호출

	cout <<"\nCase#2: Goto 서비스 요청 (Geofence 외부) " << endl;	
	
	goto_cmd.request.is_global = true;
	goto_cmd.request.x_lat = 47.3983993;
	goto_cmd.request.y_long = 8.5469999;	
	goto_cmd.request.z_alt = 50;

	if (goto_client.call (goto_cmd) == true)
	{
		if (goto_cmd.response.value == true)
		{
			ROS_INFO ("ex_geofence: goto command success!");
		}
		else
		{
			ROS_INFO ("ex_geofence: goto command fail!");

		}
			
	}


	// GeofenceReset 서비스 호출 
/*
	cout <<"\nGeofenceReset 서비스 요청 " << endl;	
	
	if (geofenceReset_client.call(geofenceReset_cmd) == true)
	{
		
		if (geofenceReset_cmd.response.value == true)
		{
			ROS_INFO ("ex_geofence: GeofenceReset command success!");
		}
		else
		{
			ROS_INFO ("ex_geofence: GeofenceReset command fail!");

		}
	}
*/	

	// Case#3) Geofence 영역 내부 지점에 대해 missionAddItem 서비스 호출
/*
	cout <<"\nCase#3: MissionAddItem 서비스 요청 (Geofence 내부) " << endl;	
	eDrone_msgs::MissionAddItem missionAddItem_cmd;
	eDrone_msgs::MissionUpload missionUpload_cmd;
	eDrone_msgs::MissionClear missionClear_cmd;
	
	ros::ServiceClient missionAddItem_client = nh.serviceClient<eDrone_msgs::MissionAddItem > ("srv_missionAddItem" );

	ros::ServiceClient missionUpload_client = nh.serviceClient<eDrone_msgs::MissionUpload>("srv_missionUpload");
	ros::ServiceClient missionClear_client =nh.serviceClient<eDrone_msgs::MissionClear>("srv_missionClear");
	
	missionAddItem_cmd.request.frame = 3;
        missionAddItem_cmd.request.command = MAV_CMD_NAV_WAYPOINT;
	missionAddItem_cmd.request.is_current = 1;
	missionAddItem_cmd.request.autocontinue = 1;

	missionAddItem_cmd.request.param1 = 0;
	missionAddItem_cmd.request.param2 = 0;
	missionAddItem_cmd.request.param3 = 0;
	missionAddItem_cmd.request.is_global = true; 
	
	missionAddItem_cmd.request.x_lat = 47.3983993;
	missionAddItem_cmd.request.y_long = 8.5469999;
	missionAddItem_cmd.request.z_alt = 50;	


	cout << "\n 미션 아이템 추가 명령: ( " << missionAddItem_cmd.request.x_lat << ", " << missionAddItem_cmd.request.y_long << ", " << missionAddItem_cmd.request.z_alt << ")" << endl;

	if (missionAddItem_client.call(missionAddItem_cmd))
	{
		if (missionAddItem_cmd.response.value == true )
		{	
			ROS_INFO("ex_geofence: missionAddItem command success!");

		//// MissionUpload
		 
		  printf("ex_geofence: Send missionUpload command ... \n");
		 

		  if (!missionUpload_client.call(missionUpload_cmd))
		  {
			ROS_INFO("missionUpload command was sent\n");
		  }
		}
		else
		{
			ROS_INFO("ex_geofence: missionAddItem command fail!");

		}
}

 
	// Case#4) Geofence 영역 내부 지점에 대해 goto 서비스 호출

	cout <<"\nCase#4: Goto 서비스 요청 (Geofence 내부) " << endl;	
	
	goto_cmd.request.is_global = true;
	goto_cmd.request.x_lat = 47.3983993;
	goto_cmd.request.y_long = 8.5469999;	
	goto_cmd.request.z_alt = 50;

	if (goto_client.call (goto_cmd) == true)
	{
		if (goto_cmd.response.value == true)
		{
			ROS_INFO ("ex_geofence: goto command success!");
		}
		else
		{
			ROS_INFO ("ex_geofence: goto command fail!");

		}
			
	}
*/
	rtl_client.call (rtl_cmd); 

	if (rtl_cmd.response.value == true)
	{
		ROS_INFO ("ex_geofence: RTL mode was set!!");
	}

	// 더 이상 남은 목적지가 없으면 HOME 위치로 복귀
	

	return 0;
} 
