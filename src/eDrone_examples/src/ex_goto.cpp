
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <vector>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Vector3Stamped.h>
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_msgs/Goto.h>
#include <eDrone_msgs/RTL.h>
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/params.h>

using namespace std;
std::vector<eDrone_msgs::Target> path; // 무인기 자율 비행 경로


// 메시지 변수 선언
eDrone_msgs::Target cur_target; // 무인기가 현재 향하고 있는 목적지 (경유지)
eDrone_msgs::Target next_target; // 다음 목적지 

// 서비스 선언
eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::Goto goto_cmd;
eDrone_msgs::RTL rtl_cmd;


// 콜백 함수 
void cur_target_cb(const eDrone_msgs::Target::ConstPtr& msg)
{
	cur_target = *msg;

	// 현재 목적지 도달 여부 확인
	ROS_INFO("cur_target_cb(): \n");
	ROS_INFO("current target: %d \n", cur_target.target_seq_no);
	
	if (cur_target.reached == true)
	{
		ROS_INFO("we reached at the current target\n");
	} 
}


int main(int argc, char** argv)
{
	ROS_INFO("==ex_goto_path==\n");

	ros::init(argc, argv, "ex_goto_path");
	ros::NodeHandle nh;

		
	// subscriber 선언
	ros::Subscriber cur_target_sub= nh.subscribe("eDrone_msgs/current_target", 10, cur_target_cb); // 
	
	ros::Rate rate(20.0);

	// service client 선언
	ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  
  	ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition"); 
	ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");
	ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
	ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
	ros::ServiceClient goto_client = nh.serviceClient<eDrone_msgs::Goto>("srv_goto");
	ros::ServiceClient rtl_client = nh.serviceClient<eDrone_msgs::RTL>("srv_rtl");

	int cur_target_seq_no = -1; // 현재 target 순번 (0, 1, 2, ...)

	// 초기 부분 경로 설정 (사각형 경로)
	  // % mission 명령에서 사용되는 waypoint와 구분하기 위해 
	  // 위치 이동 명령에서 사용되는 경유지는 target으로 지칭
	
	// target#1	
	next_target.target_seq_no = 0;
	next_target.is_global = false;
	next_target.x_lat = 0;  
        next_target.y_long = 100;
	next_target.z_alt = 50;
	next_target.reached = false;
	path.push_back(next_target);

	// target#2	
	next_target.target_seq_no = 1;
	next_target.is_global = false;
	next_target.x_lat = 100;    
        next_target.y_long = 100;
	next_target.z_alt = 50;
	next_target.reached = false;
	path.push_back(next_target);

	// target#3 	
	next_target.target_seq_no = 2;
	next_target.is_global = false;
	next_target.x_lat = 100;                                                                                
        next_target.y_long = 0;
	next_target.z_alt = 50;
	next_target.reached = false;
	path.push_back(next_target);

	cur_target_seq_no = 0; // 0번 index의 요소를 현재 target으로 지정
	
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

	ROS_INFO("Send arming command ... \n");
	arming_client.call(arming_cmd);
	ROS_INFO("Arming command was sent\n");



	//// Takeoff

	ROS_INFO("Send takeoff command ... \n");
	takeoff_cmd.request.altitude = ALTITUDE;
	takeoff_client.call(takeoff_cmd);
	ROS_INFO("Takeoff command was sent\n");

	sleep(10);
	
	

	//// Goto


	ROS_INFO("Send goto command ...\n");
	ROS_INFO("let's start a mission! \n");


	goto_cmd.request.is_global = false;
	goto_cmd.request.x_lat = 0;
	goto_cmd.request.y_long = 100;
	goto_cmd.request.z_alt = 50;
	
	goto_client.call(goto_cmd);
	ROS_INFO("Goto command was sent\n");

	// 주기적으로 자율 비행 경로에 새로운 목적지 (경유지) 정보 추가
	// 현재 목적지에 도달한 경우 더 남아 있는 경유지가 있다면 새로운 목적지로 goto service 호출

	while(ros::ok())
	{

		if (cur_target.reached== true)
		{
			if (cur_target.target_seq_no < path.size()-1)
			{

				ROS_INFO("we reached at the current target. Go to the next target\n");

				//int next_target_seq_no = cur_target.target_seq_no +1;
				
				cur_target_seq_no = cur_target.target_seq_no+1;

				next_target = path[cur_target_seq_no];


				goto_cmd.request.target_seq_no = cur_target_seq_no; // target seq no 설정				
				goto_cmd.request.is_global = next_target.is_global;
				goto_cmd.request.x_lat = next_target.x_lat;				
				goto_cmd.request.y_long = next_target.y_long;				
				goto_cmd.request.z_alt = next_target.z_alt;				
				goto_client.call(goto_cmd); // 새로운 경유지로 goto service 호출	
			} 

			else
				break;
		}


		ros::spinOnce();
		rate.sleep();
		// 경유지 추가 (필요 시)
	}


	rtl_client.call(rtl_cmd); // rtl service 호출 (복귀)

	// 더 이상 남은 목적지가 없으면 HOME 위치로 복귀
	
		


	return 0;
} 
