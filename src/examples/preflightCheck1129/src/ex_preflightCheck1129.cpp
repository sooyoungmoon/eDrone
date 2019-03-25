

/*include*/
#ifndef preflightCheck1129
// 기본 header (ROS & C/C++)
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <vector>
#include <stdlib.h>
#include <geometry_msgs/Point.h> 
#include <geographic_msgs/GeoPoint.h> 
#include <string>
#include <unistd.h>

// 토픽 선언 header 
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/Phase.h>

// 파라미터 초기값 선언 header
#include <preflightCheck1129/params.h>

// 서비스 선언 header
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>



/*namespace*/
#endif
using namespace std;
using namespace geographic_msgs;
using namespace geometry_msgs;
using namespace eDrone_msgs;



/*포인터 변수 선언*/
ros::NodeHandle* nh_ptr; // node handle pointer (서버/클라이언트 또는 퍼블리셔/서브스크라이버 선언에 사용)
eDrone_msgs::Target* cur_target_ptr; // cur_target 변수 접근을 위한 포인터 변수 
eDrone_msgs::Phase* cur_phase_ptr; // cur_phase		"



/*this is new section*/



/*콜백 함수 정의*/

// 토픽 콜백 함수

void cur_target_cb(const eDrone_msgs::Target::ConstPtr& msg)
{
	*cur_target_ptr = *msg;

	// 현재 목적지 도달 여부 확인
	ROS_INFO("cur_target_cb(): \n");
	ROS_INFO("current target: %d \n", cur_target_ptr->target_seq_no);
	
	if (cur_target_ptr->reached == true)
	{
		ROS_INFO("we reached at the current target\n");
	} 
}

void cur_phase_cb(const eDrone_msgs::Phase::ConstPtr& msg)
{
	*cur_phase_ptr = *msg;

	// 현재 목적지 도달 여부 확인
	ROS_INFO("cur_phase_cb(): \n");
	ROS_INFO("current phase: %s \n", cur_phase_ptr->phase.c_str()); 
}



/*begin of main*/
int main (int argc, char** argv)
{



/*ROS node 초기화*/

	ROS_INFO("==preflightCheck1129==\n");	
	ros::init(argc, argv, "preflightCheck1129"); 
	ros::NodeHandle nh;
	nh_ptr = &nh; // node handle 주소 저장 	


	for (int arg_index = 0; arg_index < argc; arg_index++)
	{
		ROS_INFO("main arg[%d]: %s", arg_index, argv[arg_index] );
	}



/*Topic 메시지 변수 */
	eDrone_msgs::Target cur_target; // 무인기가 현재 향하고 있는 목적지 (경유지)
	eDrone_msgs::Phase cur_phase; // 무인기의 현재 동작 단계 (ex. UNARMED, ARMED, TAKEOFF, GOTO, ...)
	cur_target_ptr = &cur_target; // cur_target 변수 주소 저장 
	cur_phase_ptr = &cur_phase;



/*Service 메시지 변수*/
	

	eDrone_msgs::CheckState checkState_cmd;
	eDrone_msgs::CheckPosition checkPosition_cmd;



/*this is new section*/



/*Topic Subscriber 객체 */
	
	// rate 설정 
	ros::Rate rate(20.0);

	// 토픽 subscriber 선언 & 초기화 

	ros::Subscriber cur_target_sub = nh.subscribe("eDrone_msgs/current_target", 10, cur_target_cb); // 
	ros::Subscriber cur_phase_sub = nh.subscribe("eDrone_msgs/current_phase", 10, cur_phase_cb);	



/* ROS Service Client 객체 */
	
	ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  
	ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition"); 



/*ROS Service 호출*/


	    // 연결 상태 확인

		sleep(10); // (수정)

		while(true)
		{
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

				if (checkState_cmd.response.connected == true)
				{
					cout << "UAV connected: " << endl;
				}
			
				if (checkState_cmd.response.armed == true )
				{
					cout << "UAV armed: " << endl;
				}

				cout << "flight mode: " << checkState_cmd.response.mode << endl;

				cout << "remaining battery(%): " << checkState_cmd.response.battery_remain << endl;
			}

	  	  // 무인기 위치 확인

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
				sleep(10);
			}


			cout <<"global frame (WGS84): (" << checkPosition_cmd.response.latitude << ", ";
			cout << checkPosition_cmd.response.longitude << ", ";
			cout << checkPosition_cmd.response.altitude << ") " << endl << endl;

			cout <<"local frame (ENU): (" << checkPosition_cmd.response.x << ", ";
			cout << checkPosition_cmd.response.y << ", " << checkPosition_cmd.response.z << ") " << endl;
			
			}
			
			ROS_INFO("UAV position was checked!");
			sleep(1);
		}


		



/*end of main*/
return 0;

}
