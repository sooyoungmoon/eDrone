
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
//#include <mavros_msgs/State.h>
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/CommandTOL.h>
//#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Waypoint.h>
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_msgs/Goto.h>
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/Survey_New.h>
#include <params.h>

using namespace std;

eDrone_msgs::CheckState checkState_cmd;
eDrone_msgs::CheckPosition checkPosition_cmd;
eDrone_msgs::Arming arming_cmd;
eDrone_msgs::Takeoff takeoff_cmd;
eDrone_msgs::Landing landing_cmd;
eDrone_msgs::Goto goto_cmd;
eDrone_msgs::Survey_New survey_new_cmd;


// 경계점 목록

vector<mavros_msgs::Waypoint> boundnary_points;

// 메시지 변수 선언

eDrone_msgs::Target cur_target;

// 콜백 함수 
/*
void cur_target_cb(const eDrone_msgs::Target::ConstPtr& msg)
{
	cur_target = *msg;

	// 현재 목적지 도달 여부 확인
	ROS_INFO("cur_target_cb(): \n");
	ROS_INFO("current target: %d \n", cur_target.target_seq_no);
	
	if (cur_target.reached == true)
	{
		printf("we reached at the current target\n");
	} 
}
*/

int main(int argc, char** argv)
{
	ROS_INFO("==ex_survey_new==\n");

	ros::init(argc, argv, "ex_survey_new");
	ros::NodeHandle nh;

		
	// subscriber 선언
	//ros::Subscriber cur_target_sub= nh.subscribe("eDrone_msgs/current_target", 10, cur_target_cb); // 
	
	ros::Rate rate(20.0);

	// service client 선언
	ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  
 	 ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition"); 
	ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");
	ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
	ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
	ros::ServiceClient survey_new_client =nh.serviceClient<eDrone_msgs::Survey_New>("srv_survey_new");

	sleep(10);

	 //// CheckState

        
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
			if (checkPosition_client.call(checkPosition_cmd));
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

	
	//// Survey


	// 경계점 목록 구성

	 // 1st point

	mavros_msgs::Waypoint point;

	point.x_lat = 0;
	point.y_long = 50;
	boundnary_points.push_back(point);

	 // 2nd point

	point.x_lat = 50;
	point.y_long = 100;
	boundnary_points.push_back(point);

	 // 3rd point

	point.x_lat = 100;
	point.y_long = 50;
	boundnary_points.push_back(point);


	// 
	
	 // 4rd point

	point.x_lat = 25;
	point.y_long = 0;
	boundnary_points.push_back(point);
	

	 // 5rd point

	point.x_lat = 75;
	point.y_long = 0;
	boundnary_points.push_back(point);

	// Survey_New 서비스 호출 

	ROS_INFO("Send survey command ...\n");

	survey_new_cmd.request.boundary_points = boundnary_points;
	survey_new_cmd.request.path_width = 10;
	survey_new_cmd.request.altitude = ALTITUDE;
	survey_new_client.call(survey_new_cmd);

	

	/*
	printf("Send survey command ... \n");
	survey_cmd.request.is_global = false;
	survey_cmd.request.min_x_lat = 10;
	
	survey_cmd.request.min_y_long = 10;
	survey_cmd.request.max_x_lat = 100;
	survey_cmd.request.max_y_long =100;
	survey_cmd.request.row_col_width = 10;

	survey_client.call(survey_cmd);	
	ROS_INFO("Survey command was sent\n");
	*/
	
	

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();		
	}



	return 0;
} 
