
// (2018.07.16)
// safety_node.cpp
// implementation of safety-related APIs (ex. Geofence, NoflyZone)

/* header files */


// C/C++ 관련 
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include <vector>


// geometry 관련 
#include <geometry_msgs/Point.h>

// ROS 관련 
#include <ros/ros.h>

// mavlink 관련 

#include <mavlink/v2.0/common/mavlink.h>

// mavros 관련 

#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

// eDrone API 관련 

#include <eDrone_msgs/GeofenceSet.h>
#include <eDrone_msgs/GeofenceCheck.h>
#include <eDrone_msgs/GeofenceReset.h>

#include <eDrone_msgs/NoflyZoneSet.h>
#include <eDrone_msgs/NoflyZoneCheck.h>
#include <eDrone_msgs/NoflyZoneReset.h>


using namespace std;
using namespace mavros_msgs;
using namespace geometry_msgs;

/* 주요 변수 선언 */
typedef struct c_str_nofly_Zone
{
	string ref_system; // 비행금지구역 저장 좌표계 (현재는 WGS84로 저장)
	bool isSet; // 비행 금지 구역 설정 여부 (true: 설정, false: 미 설정)
	double pt1_arg1;
	double pt1_arg2;
	double pt1_arg3;
	double pt2_arg1;
	double pt2_arg2;
	double pt2_arg3;

} Nofly_Zone;


typedef struct c_str_vector
{
	double x;
	double y;
	double z;

} Vector_type;


Vector_type crossproduct (Vector_type a, Vector_type b)
{
	Vector_type product;

 	product.x = 0;
	product.y = 0;
	product.z = a.x*b.y - a.y*b.x;
	return product;
}



Nofly_Zone nofly_zone; // 비행금지구역 정보 


// 메시지 변수 선언
mavros_msgs::State current_state; // 무인기 상태 정보

// (무인기 위치 확인 목적)
geometry_msgs::PoseStamped current_pos_local; // 현재 위치 및 자세 정보 (지역 좌표)
sensor_msgs::NavSatFix current_pos_global; // 현재 위치 정보 (전역 좌표)i

// API 호출&처리를 위한 메시지 변수 선언
eDrone_msgs::GeofenceSet geofenceSet_cmd; // 가상울타리 설정 메시지 
eDrone_msgs::GeofenceCheck geofenceCheck_cmd; //가상울타리 확인 메시지 
eDrone_msgs::GeofenceReset geofenceReset_cmd; //가상울타리 해제 메시지

eDrone_msgs::NoflyZoneSet noflyZoneSet_cmd; // 비행금지구역 설정 메시지
eDrone_msgs::NoflyZoneCheck noflyZoneCheck_cmd; // 비행금지구역 확인  메시지
eDrone_msgs::NoflyZoneReset noflyZoneReset_cmd; // 비행금지구역 설정 메시지


// ROS Service 서버 선언

ros::ServiceServer geofenceSet_srv_server;
ros::ServiceServer geofenceCheck_srv_server;
ros::ServiceServer geofenceReset_srv_server;

ros::ServiceServer noflyZoneSet_srv_server;
ros::ServiceServer noflyZoneReset_srv_server;
ros::ServiceServer noflyZoneCheck_srv_server;

// publisher 선언

// subscriber 선언 
ros::Subscriber state_sub;
ros::Subscriber pos_sub_local;
ros::Subscriber pos_sub_global;

// ROS Service 클라이언트 선언


/* callback 함수  */

// 현재 상태 확인 
void state_cb(const mavros_msgs::State::ConstPtr& msg){
	
	current_state = *msg;
/*
	if (current_state.connected)
	{
 	  printf("A UAV is connected\n");
	}
	

	else
	{
  	  printf("A UAV is not connected\n");
	}

	if (current_state.armed)
	{
	  printf("A UAV is armed\n");
	} 
	else
	{
  	  printf("A UAV is not armed\n");
	}
	
	cout << "flight mode:" <<  current_state.mode << endl;

			

	if (current_state.mode.compare("AUTO.RTL") ==0)
	{
//		ROS_INFO("state_cb(): FLIGHT MODE = RTL");
	}*/
}

// 현재 위치 (지역좌표) 확인 
void pos_cb_local(const geometry_msgs::PoseStamped::ConstPtr& msg){

	current_pos_local = *msg;

	double DIST_RANGE = 0.5;	


//					printf("current_position: (%f, %f, %f \n)", current_pos_local.pose.position.x, current_pos_local.pose.position.y, current_pos_local.pose.position.z);	


	// geofence 기능 추가

	double distance_home = 0;

	distance_home = current_pos_local.pose.position.x * current_pos_local.pose.position.x + current_pos_local.pose.position.y * current_pos_local.pose.position.y;

	distance_home = sqrt(distance_home);
/*
	if (distance_home > Geofence_Radius)
	{
		ROS_INFO("Geofence: boundary breach");
		printf("Geofence_Radius: %f\n", Geofence_Radius);

		if (geofence_policy == Warning)
		{		
			printf("Don't cross the line!!\n");
		}
		else if (geofence_policy == RTL)
		{
			modeChange_cmd.request.base_mode = 0;
			modeChange_cmd.request.custom_mode = "AUTO.RTL";
			modeChange_client.call(modeChange_cmd);
		}
		
		else if (geofence_policy == Loiter)
		{

			modeChange_cmd.request.base_mode = 0;
			modeChange_cmd.request.custom_mode = "AUTO.LOITER";
			modeChange_client.call(modeChange_cmd);
		}

		else if (geofence_policy == Landing)
		{

			modeChange_cmd.request.base_mode = 0;
			modeChange_cmd.request.custom_mode = "AUTO.LAND";
			modeChange_client.call(modeChange_cmd);
		}

	}

*/
	// geofence (end)

/*
	if (phase.compare("TAKEOFF")==0)
	{
		cout << "TAKEOFF is ongoing: takeoff_altitude is " << takeoff_altitude << endl;


		if (current_pos_local.pose.position.z >= takeoff_altitude-0.01 )
		{
			cout << "eDrone_control_node: TAKEOFF completed!" << endl;

			phase = "READY"; 
		}
	
		return;
	}


	if (!autonomous_flight)
		return;

	if(  (target_position.reached == false) && (target_position.is_global == false))
	{
		// 목적지 도착 여부 확인
		double distance;

		if ( ((target_position.pos_local.x - DIST_RANGE) < current_pos_local.pose.position.x) &&
			 (current_pos_local.pose.position.x <(target_position.pos_local.x + DIST_RANGE) ) )
		{
			
			if ( ((target_position.pos_local.y - DIST_RANGE) < current_pos_local.pose.position.y) &&
			 (current_pos_local.pose.position.y <(target_position.pos_local.y + DIST_RANGE) ) )
			{
					
				if ( ((target_position.pos_local.z - 0.5) < current_pos_local.pose.position.z) && 
				(current_pos_local.pose.position.z <(target_position.pos_local.z +0.5) ) )
				{
					target_position.reached = true;
					ROS_INFO("pos_cb_local(): The UAV reached to the target position");	

					printf("current_position: (%f, %f, %f \n)", current_pos_local.pose.position.x, current_pos_local.pose.position.y, current_pos_local.pose.position.z);	
					
			//		modeChange_cmd.request.base_mode = 0;
			//		modeChange_cmd.request.custom_mode = MAV_MODE_AUTO_ARMED;
			//		modeChange_cmd.request.custom_mode = "Hold";
			//		modeChange_client.call(modeChange_cmd);
				}

				
			}

		}
	}*/
}

// 현재 위치 (전역 좌표) 확인 

void pos_cb_global(const sensor_msgs::NavSatFix::ConstPtr& msg){

	

	current_pos_global = *msg;


//					printf("current_position: (%f, %f, %f \n)", current_pos_global.latitude, current_pos_global.longitude, current_pos_global.altitude-HOME_ALT);
	
//	if (!autonomous_flight)
//		return;
/*
	if( (target_position.reached == false) && (target_position.is_global == true))
	

	{
		// 목적지 도착 여부 확인
		double distance;

		if ( ((target_position.pos_global.latitude - 0.0001) < current_pos_global.latitude) &&

			 (current_pos_global.latitude <(target_position.pos_global.latitude +0.0001) ) )
		{
			
			if ( ((target_position.pos_global.longitude - 0.0001) < current_pos_global.longitude) &&
			 (current_pos_global.longitude <(target_position.pos_global.longitude +0.0001) ) )
			{
					
				if ( ((target_position.pos_global.altitude - 0.5) < current_pos_global.longitude) && 
				(current_pos_global.longitude <(target_position.pos_global.longitude +0.5) ) )
				{
					target_position.reached = true;
					ROS_INFO("pos_cb_global(): The UAV reached to the target position");	
					
					
					
				//	modeChange_cmd.request.base_mode = 0;
				//	modeChange_cmd.request.custom_mode = "Hold";
				//	modeChange_client.call(modeChange_cmd);
					//printf("current_position: (%f, %f, %f \n)", current_pos_global.latitude, current_pos_global.longitude, current_pos_global.altitude);	
				}

				
			}

		}
	}*/
}


bool srv_noflyZoneSet_cb(eDrone_msgs::NoflyZoneSet::Request &req, eDrone_msgs::NoflyZoneSet::Response &res)
{
  
  // reference system: WGS84 지원

  ROS_INFO ("safety_node: NoflyZoneSet service was called");

  if ( req.ref_system.compare("WGS84") ==0)
  {
 	nofly_zone.pt1_arg1 = req.pt1_arg1;
 	nofly_zone.pt1_arg2 = req.pt1_arg2;
 	nofly_zone.pt1_arg3 = req.pt1_arg3;
 	nofly_zone.pt2_arg1 = req.pt2_arg1;
 	nofly_zone.pt2_arg2 = req.pt2_arg2;
 	nofly_zone.pt2_arg3 = req.pt2_arg3;
  }
  nofly_zone.isSet = true;
  res.value = true;

  return true; 

}


bool srv_noflyZoneReset_cb(eDrone_msgs::NoflyZoneReset::Request &req, eDrone_msgs::NoflyZoneReset::Response &res)
{	
  nofly_zone.isSet = false; 

  res.value = true;
  return true;
}

bool srv_noflyZoneCheck_cb(eDrone_msgs::NoflyZoneCheck::Request &req, eDrone_msgs::NoflyZoneCheck::Response &res)
{

  /* 출발지, 목적지 정보를 입력 받아 비행 금지 구역과의 관계 반환 */

  
  //
  res.value= false;
  ROS_INFO ("safety_node: NoflyZoneCheck service was called");


  cout << "src: (" << req.src_arg2 << " , " << req.src_arg1 << ") " <<endl;
  cout << "dst: (" << req.dst_arg2 << " , " << req.dst_arg1 << ") " <<endl;

  if ( (nofly_zone.isSet != true) || req.ref_system.compare("WGS84") !=0 )
  {
    return true; // 비행 금지 구역이 미 설정되었거나 입력 좌표가 WGS84가 아니면 false 반환 
  }


  // 출발지와 비행 금지 구역 간 관계 확인
  if ( (req.src_arg1 < nofly_zone.pt1_arg1) != ( req.src_arg1 < nofly_zone.pt2_arg1)  )
  {
	ROS_INFO ("TEST1");

	if ( (req.src_arg2 < nofly_zone.pt1_arg2) != ( req.src_arg2 < nofly_zone.pt2_arg2))
	{
		ROS_INFO ("Src is in the NF zone! "); // CASE#1: 출발지가 비행 금지 구역 내
		res.result.assign("SRC_IN_NF_ZONE" );
		res.value = true;
		return true;
 	}	
  }  
     // 목적지와 비행 금지 구역 간 관계 확인 
   if ( (req.dst_arg1 < nofly_zone.pt1_arg1) != ( req.dst_arg1 < nofly_zone.pt2_arg1)  )
  {
	ROS_INFO ("TEST2");

	if ( (req.dst_arg2 < nofly_zone.pt1_arg2) != ( req.dst_arg2 < nofly_zone.pt2_arg2))
	{
		ROS_INFO ("Dst is in the NF zone! "); // CASE#2: 목적지가 비행 금지 구역 내
		res.result.assign("DST_IN_NF_ZONE" );
		res.value = true;
		return true;
	}
  } 

  // 출발지-목적지 간 직선 경로와 비행 금지 구역 간 관계 확인 


   // 1) 벡터 변수 선언 

  	Vector_type directPath;
	directPath.x = req.dst_arg2 - req.src_arg2;
	directPath.y = req.dst_arg1 - req.src_arg1;
	directPath.z = 0;

	
	Vector_type rect_line[4];

	rect_line[0].x= nofly_zone.pt2_arg2 - nofly_zone.pt1_arg2;
	rect_line[0].y= 0;

	rect_line[1].x= 0; 
	rect_line[1].y = nofly_zone.pt2_arg1 - nofly_zone.pt1_arg1;
	rect_line[1].z = 0;

	rect_line[2].x = nofly_zone.pt1_arg2 - nofly_zone.pt2_arg2;
	rect_line[2].y = 0;
	rect_line[2].z = 0;


	rect_line[3].x = 0;	
	rect_line[3].y = nofly_zone.pt1_arg1 - nofly_zone.pt2_arg1;
	rect_line[3].z = 0;
	

	Point rect_point[4];

	rect_point[0].x = nofly_zone.pt1_arg2;
	rect_point[0].y = nofly_zone.pt1_arg1;

	rect_point[1].x = nofly_zone.pt2_arg2;
	rect_point[1].y = nofly_zone.pt1_arg1;

	rect_point[2].x = nofly_zone.pt2_arg2;
	rect_point[2].y = nofly_zone.pt2_arg1;
	
	rect_point[3].x = nofly_zone.pt1_arg2;
	rect_point[3].y = nofly_zone.pt2_arg1;

	ROS_INFO ("TEST3-1");	


   // 2) 직선 경로와 사각형의 각 변 사이에 교차 여부 확인
	bool overlap = false;

	for (int i = 0; i < 4; i++)
	{
		bool crossing = false;

		Vector_type product1, product2;
		Vector_type product3, product4;

		//product = crossproduct (directPath, rectangle[i]);

		// directPath와 두 벡터 간의 외적 값의 곱 구하기

		double c1= 0.0, c2 = 0.0;
		
		Vector_type vec1, vec2;

		vec1.x = rect_point [i ].x - req.src_arg2;
		vec1.y = rect_point [i].y - req.src_arg1;

		vec2.x = rect_point[ (i+1)%4].x - req.src_arg2;
		vec2.y = rect_point[ (i+1)%4].y - req.src_arg1;


		product1 = crossproduct (directPath, vec1);		
		product2 = crossproduct (directPath, vec2);
	
		c1 = product1.z * product2.z;
	
		// rect_line[i]와 두 벡터 간의 외적 값의 곱 구하기

	
		Vector_type vec3, vec4;	
		
		vec3.x = req.src_arg2- rect_point[i].x;
		vec3.y = req.src_arg1 - rect_point[i].y;

		vec4.x = req.dst_arg2 - rect_point[i].x;
		vec4.y = req.dst_arg1 - rect_point[i].y;
	

		product3 = crossproduct (rect_line[i], vec3);
		product4 = crossproduct (rect_line[i], vec4);
		
		c2 = product3.z * product4.z; 

		if ( (c1 <0) && (c2 < 0) ) // directPath와 rect_line[i]는 서로 교차함 
		{
			ROS_INFO ("test3-2");
			cout << "directPath: (" << directPath.x << ", " << directPath.y << " )" << endl;			   	       cout << "rect_line: (" << rect_line[i].x << ", " << rect_line[i].y << " )" << endl;

			cout << "c1: " << c1 << endl;
			cout << "c2: " << c2 << endl;

			crossing = true;
			overlap = true;

			cout << "line crossing: " << i << endl;
		}
	}


	

   // 3) 직선 경로와 사각형의 변들 중 하나가 교차하는 경우 true 반환, 그렇지 않으면 false 반한

	if (overlap ==  true)
	{
		ROS_INFO ("_Src-Dst direct path overlaps the NF zone! " );
		res.result.assign("PATH_OVERLAP_NF_ZONE");
	}

    //선분의 기울기 계산

/*   
   {
      if ( (req.dst_arg2 - req.src_arg2) !=0 ) // src와 dst 두 지점의 경도가 다를 경우 (기울기가 0이 아님)
      {
		//ROS_INFO("TEST3");

		double grad = (req.dst_arg1 - req.src_arg1) / (req.dst_arg2 - req.src_arg2 ); // Src-Dst 직선 경로의 기울기 계산 

		double y_intercept = req.dst_arg1 - grad * req.dst_arg2; // y절편 

		double y1 = grad * nofly_zone.pt1_arg2 + y_intercept; //
		double y2 = grad * nofly_zone.pt2_arg2 + y_intercept;

		cout << "src: ( " << req.src_arg1 << ", " << req.src_arg2 << ") " << endl;
		cout << "dst: ( " << req.dst_arg1 << ", " << req.dst_arg2 << ") " << endl;


		cout <<" grad: " << grad << endl;
		cout <<"y_intercept: " << y_intercept << endl;
		cout << "y1: " << y1 << ", y2: " << y2 << endl;

		// 비행 금지 구역과 src-dst 직선 경로가 서로 겹치는 지 확인 
		if ( ((y1 < nofly_zone.pt1_arg1) != (y1 <nofly_zone.pt2_arg1) ) || 
			
			( (y2 < nofly_zone.pt1_arg1) != ( y2 < nofly_zone.pt2_arg1)  )  )
		{
	//		cout << "y1: " << y1 << ", y2: " << y2 << endl;
		
			ROS_INFO ("Src-Dst direct path overlaps the NF zone! " );
			res.result.assign("PATH_OVERLAP_NF_ZONE");
		}
      }
      else // src와 dst의 경도가 같을 경우 (기울기가 0)
      {
		if ( (req.dst_arg2 < nofly_zone.pt1_arg2) != (req.dst_arg2 < nofly_zone.pt2_arg2 )  )
		{
			//ROS_INFO("TEST2-1");
			
			double min_lat = req.src_arg1;
			double max_lat = req.dst_arg1;

			double nofly_min_lat = nofly_zone.pt1_arg1;
			double nofly_max_lat = nofly_zone.pt2_arg1;
			double nofly_min_lon = nofly_zone.pt1_arg2;			
			double nofly_max_lon = nofly_zone.pt2_arg2;

			//cout << "min_lat: " << min_lat << ", max_lat: " << max_lat << endl;
			//cout << "nofly_min_lat: " << nofly_min_lat << ", nofly_max_lat: " << nofly_max_lat << endl;
		
			


			if ( req.dst_arg1 < req.src_arg1)
			{
				min_lat = req.dst_arg1;
				max_lat = req.src_arg1;
			}


			if (nofly_zone.pt2_arg1 < nofly_zone.pt1_arg1 )
			{
				nofly_min_lat = nofly_zone.pt2_arg1;
				nofly_max_lat = nofly_zone.pt1_arg1;
			}

			if (nofly_zone.pt2_arg2 < nofly_zone.pt1_arg2 )
			{
				nofly_min_lon = nofly_zone.pt2_arg2;
				nofly_max_lon = nofly_zone.pt1_arg2;
			}
		
			if ( ( req.src_arg2 < nofly_min_lon) != (req.src_arg2 < nofly_max_lon) )
			{
				//ROS_INFO ("TEST2-2");
				//	cout << "min_lat: " << min_lat << ", max_lat: " << max_lat << endl;
				//	cout << "nofly_min_lat: " << nofly_min_lat << ", nofly_max_lat: " << nofly_max_lat << endl;
				
				if ( (min_lat < nofly_min_lat ) &&  (max_lat > nofly_max_lat )	)
				{
					//cout << "min_lat: " << min_lat << ", max_lat: " << max_lat << endl;
					//cout << "nofly_min_lat: " << nofly_min_lat << ", nofly_max_lat: " << nofly_max_lat << endl;
		
					ROS_INFO ("_Src-Dst direct path overlaps the NF zone! " );
					res.result.assign("PATH_OVERLAP_NF_ZONE");
				}
			}		

		}

      }



   }


  */	

    // 비행 금지 구역 X 좌표 ( X1, X2) 에 대응되는 Y 좌표 (Y1, Y2) 계산

    //  

  // 

  // 
 /*
  res.value= false;
  ROS_INFO ("safety_node: NoflyZoneCheck service was called");

  // 요청 메시지 포함된 좌표가  

  if ( (nofly_zone.isSet == true) &&  ( req.ref_system.compare("WGS84") ==0) )
  {
		ROS_INFO ("Fist condition");

  	if ( (req.arg1 < nofly_zone.pt1_arg1 ) !=  (req.arg1 < nofly_zone.pt2_arg1 ) )
  	{
		
		ROS_INFO ("Second condition");

		if ((req.arg2 < nofly_zone.pt1_arg2) != (req.arg2 < nofly_zone.pt2_arg2) )
		{
		
			ROS_INFO ("safety_node: NoflyZone violation!");
			res.violation = true;
		}
  	}
  }
  
  res.value = true;
  return true;

  */
}

bool srv_geofenceSet_cb(eDrone_msgs::GeofenceSet::Request &req, eDrone_msgs::GeofenceSet::Response &res)
{
  return true;
}


bool srv_geofenceReset_cb(eDrone_msgs::GeofenceReset::Request &req, eDrone_msgs::GeofenceReset::Response &res)
{
  return true;
}


bool srv_geofenceCheck_cb(eDrone_msgs::GeofenceCheck::Request &req, eDrone_msgs::GeofenceCheck::Response &res)
{
  return true;
}





/* main 함수 */

int main(int argc, char** argv )
{
	ros::init(argc, argv, "safety_node");

 	ros::NodeHandle nh; 
	
	ros::Rate rate (20.0); 

	// publisher 초기화

	// subscriber 초기화
	state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
	pos_sub_local = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose",10,  pos_cb_local); 
	pos_sub_global = nh.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global",10,  pos_cb_global); 

	// 서비스 서버 초기화
	noflyZoneSet_srv_server = nh.advertiseService ( "srv_noflyZoneSet", srv_noflyZoneSet_cb );	
	noflyZoneReset_srv_server = nh.advertiseService ( "srv_noflyZoneReset", srv_noflyZoneReset_cb );	
	noflyZoneCheck_srv_server = nh.advertiseService ( "srv_noflyZoneCheck", srv_noflyZoneCheck_cb );	

	// 서비스 클라이언트 초기화

	while ( ros::ok() ) // API 피 호출 시, 콜백 함수를 통해 해당 기능 처리 후 응답
	{
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
