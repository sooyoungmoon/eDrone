
/* 2018.08.16 */


/* include */

// 기본 header (ROS & C/C++)
#include <mavlink/v2.0/common/mavlink.h>
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <vector>
#include <stdlib.h>

// 토픽 선언 header 
#include <eDrone_msgs/Target.h>

// 서비스 선언 header
#include <eDrone_msgs/CheckState.h>
#include <eDrone_msgs/CheckPosition.h>
#include <eDrone_msgs/Arming.h>
#include <eDrone_msgs/Takeoff.h>
#include <eDrone_msgs/Landing.h>
#include <eDrone_msgs/Goto.h>
#include <eDrone_msgs/RTL.h>
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/Phase.h>
#include <eDrone_msgs/SurveyArea.h>
// 파라미터 초기값 선언 header
#include <eDrone_examples/params.h>

using namespace std;
using namespace geometry_msgs;
/* 포인터 변수 선언  */

ros::NodeHandle* nh_ptr; // node handle pointer (서버/클라이언트 또는 퍼블리셔/서브스크라이버 선언에 사용)
eDrone_msgs::Target* cur_target_ptr; // cur_target 변수 접근을 위한 포인터 변수 
eDrone_msgs::Phase* cur_phase_ptr; // cur_phase		"

/* 콜백 함수 정의 */

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



// 서비스 콜백 함수 (내용 없음) 


/* main 함수 */

int main(int argc, char** argv)
{
	ROS_INFO("==ex_survey==\n");

	ros::init(argc, argv, "ex_survey"); 
	ros::NodeHandle nh;
	nh_ptr = &nh; // node handle 주소 저장 


	/* 주요 변수 선언 */

	if (argc < 2)
	{
		ROS_ERROR("ex_survey: the number of arguments should be at least 2!!" );
		return -1;
	}


	for (int arg_index = 0; arg_index < argc; arg_index++)
	{
		ROS_INFO("main arg[%d]: %s", arg_index, argv[arg_index] );
	}


	// 토픽 메시지 변수 선언  
	eDrone_msgs::Target cur_target; // 무인기가 현재 향하고 있는 목적지 (경유지)
	eDrone_msgs::Phase cur_phase; // 무인기의 현재 동작 단계 (ex. UNARMED, ARMED, TAKEOFF, GOTO, ...)
	cur_target_ptr = &cur_target; // cur_target 변수 주소 저장 
	eDrone_msgs::Target next_target; // 다음 목적지 
	cur_phase_ptr = &cur_phase;

	// 서비스 메시지 변수 선언 
	eDrone_msgs::CheckState checkState_cmd;
	eDrone_msgs::CheckPosition checkPosition_cmd;
	eDrone_msgs::Arming arming_cmd;
	eDrone_msgs::Takeoff takeoff_cmd;
	eDrone_msgs::Landing landing_cmd;
	eDrone_msgs::Goto goto_cmd;
	eDrone_msgs::RTL rtl_cmd;
	eDrone_msgs::SurveyArea surveyArea_cmd;	

	// 토픽 publisher 초기화 (내용 없음)

	// rate 설정 
	ros::Rate rate(20.0);

	// 토픽 subscriber 선언 & 초기화 

	ros::Subscriber cur_target_sub = nh.subscribe("eDrone_msgs/current_target", 10, cur_target_cb); 
 	ros::Subscriber cur_phase_sub = nh.subscribe("eDrone_msgs/current_phase", 10, cur_phase_cb); // 


	// 서비스 서버 선언 & 초기화 (내용 없음)

	// 서비스 클라이언트 선언 & 초기화

	ros::ServiceClient checkState_client =nh.serviceClient<eDrone_msgs::CheckState>("srv_checkState");  
        ros::ServiceClient checkPosition_client =nh.serviceClient<eDrone_msgs::CheckPosition>("srv_checkPosition"); 
	ros::ServiceClient arming_client =nh.serviceClient<eDrone_msgs::Arming>("srv_arming");
	ros::ServiceClient takeoff_client =nh.serviceClient<eDrone_msgs::Takeoff>("srv_takeoff");
	ros::ServiceClient landing_client =nh.serviceClient<eDrone_msgs::Landing>("srv_landing");
	ros::ServiceClient goto_client = nh.serviceClient<eDrone_msgs::Goto>("srv_goto");
	ros::ServiceClient rtl_client = nh.serviceClient<eDrone_msgs::RTL>("srv_rtl");
	ros::ServiceClient surveyArea_client = nh.serviceClient<eDrone_msgs::SurveyArea>("srv_surveyArea");
	
	// 무인기 자율 비행 경로 
	std::vector<eDrone_msgs::Target> path; 

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
		
		  
		// Arming

		ROS_INFO("Send arming command ... \n"); 
		arming_client.call(arming_cmd);
		
		if (arming_cmd.response.value == true)
		{
			ROS_INFO("Arming command was sent\n");
		}


		// Takeoff
		{
			double altitude = 0;
			altitude = atof (argv[1]); 
			ROS_INFO("Altitude: %lf", altitude);

			ROS_INFO("Send takeoff command ... \n");
			takeoff_cmd.request.altitude = altitude; // 서비스 파라미터 설정
			takeoff_client.call(takeoff_cmd); // 서비스 호출

			if (takeoff_cmd.response.value == true) // 서비스 호출 결과 확인 
			{
				ROS_INFO("Takeoff command was sent\n");
			}
		}
//		sleep(10);


	    // 경로 비행 (임무 수행)			

			/* test용 코드 
			for (int vector_index = 0; vector_index < x_vector.size(); vector_index++)
			{
				ROS_INFO("(X, Y): (%lf, %lf) ", x_vector[vector_index], y_vector[vector_index]);
			}*/

			int cur_target_seq_no = -1; // 현재 target 순번 (0, 1, 2, ...)		
			

			//// Goto

			/*
			while (cur_phase.phase.compare ("READY")!=0)
			{
				ros::spinOnce();
				rate.sleep();
			}

			ROS_INFO("Send goto command ...\n");
			ROS_INFO("let's start a mission! \n");

			cur_target_seq_no = 0;
			goto_cmd.request.is_global = IS_GLOBAL;
			
			if (goto_cmd.request.is_global == true)
			{
				goto_cmd.request.ref_system = "WGS84";
			}
			else
			{
				goto_cmd.request.ref_system = "ENU";
			}

			goto_cmd.request.x_lat = atof (argv[2]);
			goto_cmd.request.y_long = atof (argv[3]);
			goto_cmd.request.z_alt = atof (argv[4]);
			
			goto_client.call(goto_cmd);
			ROS_INFO("Goto command was sent\n");		   
	


	while (cur_target.reached != true || cur_target.target_seq_no < cur_target_seq_no)
	{			
		ros::spinOnce();
		rate.sleep();
	}
	cur_target_seq_no = cur_target.target_seq_no +1;  	
	*/

	while (cur_phase.phase.compare ("READY") !=0)
	{
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("We call SurveyArea API!");
	sleep(5);

	// SurveyArea	
	{
		vector<Point> boundary_points;
		surveyArea_cmd.request.survey_ref_system = SURVEY_REF_SYSTEM;
		surveyArea_cmd.request.survey_altitude =  atof(argv[2]);
		surveyArea_cmd.request.interval = INTERVAL;		


		vector <double> x_vector; // (survey) 경계점들의 x 벡터 
		vector <double> y_vector; // (survey) 경계점들의 y 벡터

		for (int arg_index = 3; arg_index < argc; arg_index+=2)
		{
				double x = atof (argv[arg_index]) ;
				double y = atof (argv[arg_index+1]);				

				x_vector.push_back(x);
				y_vector.push_back(y);
				
				Point p;
				p.x = x;
				p.y = y;

				boundary_points.push_back(p);
		}

		surveyArea_cmd.request.boundary_pts_local = boundary_points;			

		if (surveyArea_client.call(surveyArea_cmd)) // surveyArea 호출
		{
			ROS_INFO("SurveyArea command was sent\n");
		}
	}

	while (cur_phase.phase != "READY")
	{
		ros::spinOnce();
		rate.sleep();
	}

	sleep(10);
/*
	rtl_client.call(rtl_cmd); // rtl service 호출 (복귀)
	
	if (rtl_cmd.response.value == true)
	{
		ROS_INFO("RTL command was sent\n");
        }
*/
	return 0; 
}



