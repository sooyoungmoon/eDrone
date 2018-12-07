
// (2018.11.22)
// eDrone_safety_node.cpp
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
#include <eDrone_msgs/Target.h>
#include <eDrone_msgs/GeofenceSet.h>
#include <eDrone_msgs/GeofenceCheck.h>
#include <eDrone_msgs/GeofenceReset.h>
#include <eDrone_msgs/NoflyZoneSet.h>
#include <eDrone_msgs/NoflyZoneCheck.h>
#include <eDrone_msgs/NoflyZoneReset.h>
#include <eDrone_msgs/NoflyZone.h> // NoflyZone type
#include <eDrone_msgs/NoflyZones.h> // NoflyZones topic
#include <eDrone_msgs/Geofence.h> // Geofence topic
#include <eDrone_lib/params.h>
#include <eDrone_lib/types.h>
#include <eDrone_lib/GeoUtils.h>

using namespace std;
using namespace mavros_msgs;
using namespace geometry_msgs;
using namespace eDrone_msgs;

extern bool pathOverlap(Point, Point, vector<Point>);

/* */

/* 주요 변수 선언 */

eDrone_msgs::NoflyZones nfZones; // 비행금지구역 (다수)
eDrone_msgs::Geofence geofence; // 가상울타리 


// 메시지 변수 선언
mavros_msgs::State current_state; // 무인기 상태 정보


// (무인기 위치 확인 목적)
geometry_msgs::PoseStamped current_pos_local; // 현재 위치 및 자세 정보 (지역 좌표)
sensor_msgs::NavSatFix current_pos_global; // 현재 위치 정보 (전역 좌표)i

// (홈 위치 획득 목적)
mavros_msgs::HomePosition home_position; // home position

// ('NoflyZones'  TOPIC 출판을 위한 메시지 변수)
//eDrone_msgs::NoflyZone


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
ros::ServiceServer noflyZoneCheck_srv_server;
ros::ServiceServer noflyZoneReset_srv_server;


// publisher 선언

// subscriber 선언 
ros::Subscriber state_sub;
ros::Subscriber pos_sub_local;
ros::Subscriber pos_sub_global;
ros::Subscriber home_sub;

// ROS Service 클라이언트 선언
// home position
 float HOME_LAT ;
 float HOME_LON;
 float HOME_ALT;


// printNoflyzone
void printNFZones()
{
	int cnt = 0;
	cout << "printNFZone(); " << endl;
	for ( vector<NoflyZone>::iterator it = nfZones.noflyZones.begin(); it != nfZones.noflyZones.end(); it++)
	{
		NoflyZone nfZone = *it;
		cout << "NoflyZone#" <<cnt << endl;
		
		for (vector<Target>::iterator it = nfZone.noflyZone_pts.begin(); it!=nfZone.noflyZone_pts.end(); it++)
		{
			Target point = *it;
		  	cout << "point#" << cnt << "(" << point.x_lat <<", " << point.y_long << ", " << point.z_alt << ")" << endl;

		}		
	}	
}


/*
// (2018.11.23) 비행금지구역 우회경로 계산 
vector<Target_Position> getIndirectPath(Target src, Target dest)
{
	std::vector<Target_Position> path;

	ROS_INFO("eDrone_safety_node: getIndirectPath()");

	// ENU 좌표계로 통일

	Point src_pt;
	Point dest_pt;

	if( src.ref_system == "WGS84")
	{
		src_pt = convertGeoToENU(src.x_lat, src.y_long, src.z_alt, HOME_LAT, HOME_LON, HOME_ALT );
	}
	else if (src.ref_system == "ENU" )
	{
		src_pt.x = src.x_lat;
		src_pt.y = src.y_long;
		src_pt.z = src.z_alt;
	}	

	if( dest.ref_system == "WGS84")
	{
		dest_pt = convertGeoToENU(dest.x_lat, dest.y_long, dest.z_alt, HOME_LAT, HOME_LON, HOME_ALT );
	}
	else if (dest.ref_system == "ENU" )
	{
		dest_pt.x = dest.x_lat;
		dest_pt.y = dest.y_long;
		dest_pt.z = dest.z_alt;
	}


	//// Wavefront 맵 범위 지정, cell 배열 생성 

	// Mental Map 범위 계산	
	cout << "set mental map range" << endl;

	double x_min = -1, y_min=-1;
	double x_max = -1, y_max= -1;

	if (src_pt.x < dest_pt.x)
	{
		x_min = src_pt.x;
		x_max = dest_pt.x;
	}
	else
	{
		x_min = dest_pt.x;
		x_max = src_pt.x;
	}

	if (src_pt.y < dest_pt.y)
	{
		y_min = src_pt.y;
		y_max = dest_pt.y;
	}
	else
	{
		y_min = dest_pt.y;
		y_max = src_pt.y;
	}


	// Mental map & grid 생성
	
	Mental_Map mental_map;

	Mental_Map* mental_map_ptr = &mental_map;

	cout << "CELL_WIDTH: " << CELL_WIDTH << endl;
 	cout << "CELL_HEIGHT: " << CELL_HEIGHT << endl ;

	int AREA_WIDTH = ceil ((x_max - x_min) / CELL_WIDTH)  ;
	int AREA_HEIGHT = ceil ((y_max - y_min) / CELL_HEIGHT)  ;

	cout << "x: " << x_min << " ~ " << x_max << endl;
	cout << "y: " << y_min << " ~ " << y_max << endl;
	cout << "AREA_WIDTH: " << AREA_WIDTH << endl ;
	cout << "AREA_HEIGHT: " << AREA_HEIGHT << endl; 

	mental_map.area_width = AREA_WIDTH;
	mental_map.area_height = AREA_HEIGHT;	
	mental_map.grid = new Cell*[AREA_WIDTH+1];



	////  WavefrontMap 생성




	////  비행금지구역 우회 경로 계산




		////  우회경로 반환
	

	// ...

	return path;
}
*/


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

void homePosition_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
        home_position = *msg;
        
//      printf("home position: (%f, %f, %f) \n", home_position.position.x, home_position.position.y, home_position.position.z);

        HOME_LAT = home_position.geo.latitude;
        HOME_LON = home_position.geo.longitude;
        HOME_ALT = home_position.geo.altitude;

        static int print_count =0;

        if (print_count < 1)
        {
                printf("safety_node: home position: (%f, %f, %f) \n", HOME_LAT, HOME_LON, HOME_ALT);
                print_count++;
        }

}



bool srv_noflyZoneSet_cb(eDrone_msgs::NoflyZoneSet::Request &req, eDrone_msgs::NoflyZoneSet::Response &res)
{
  
  // reference system: WGS84 지원

  ROS_INFO ("eDrone_safety_node: NoflyZoneSet service was called");

  if ((req.noflyZoneSet_ref_system.compare("WGS84") ==0)||
   (req.noflyZoneSet_ref_system.compare("ENU") ==0))
  {
	NoflyZone nofly_zone;
	nofly_zone.noflyZone_ref_system = req.noflyZoneSet_ref_system;
	nofly_zone.noflyZone_pts = req.noflyZoneSet_pts;	

	int nCnt = 0;
/*
	for (vector<Target>::iterator it=  nofly_zone.nfZone_pts.begin(); 
it != nofly_zone.nfZone_pts.end(); it++)
{
	 Target point = *it;
         cout << "point#" << nCnt << "(" << point.x_lat <<", " << point.y_long << ", " << point.z_alt << ")" << endl;
         nCnt++;

}
*/

	nfZones.noflyZones.push_back(nofly_zone);

  }
  //  sleep(10);
  //printNFZones();
}


bool srv_noflyZoneReset_cb(eDrone_msgs::NoflyZoneReset::Request &req, eDrone_msgs::NoflyZoneReset::Response &res)
{	
  //nofly_zone.isSet = false; 

  res.value = true;
  return true;
}

bool srv_noflyZoneCheck_cb(eDrone_msgs::NoflyZoneCheck::Request &req, eDrone_msgs::NoflyZoneCheck::Response &res)
{
	res.result = "NONE";
	//#1 주요 변수 선언 
	// ENU로 좌표 맞춤  
//	vector<Point> boundary_pts;	
	Point src;
	Point dest;
	Point target;

	cout << "noflyZoneCheck_cb() " << endl;

        cout << "src: (" <<  req.noflyZoneCheck_src.x_lat << ", " << req.noflyZoneCheck_src.y_long << ", " << req.noflyZoneCheck_src.z_alt << ")" <<endl;
        cout << "dest: (" <<  req.noflyZoneCheck_dest.x_lat << ", " << req.noflyZoneCheck_dest.y_long << ", " << req.noflyZoneCheck_dest.z_alt << ")" <<endl;
	
	cout << "target type: " << req.noflyZoneCheck_dest.ref_system << endl;


	//#2 목적지가 비행금지구역에 속하는 지 여부 확인 

		
	// src의 좌표계를 ENU로 변환 
	if (req.noflyZoneCheck_src.ref_system.compare("WGS84")==0 )
	{
		src = convertGeoToENU( req.noflyZoneCheck_src.x_lat, req.noflyZoneCheck_src.y_long, req.noflyZoneCheck_src.z_alt, HOME_LAT, HOME_LON, HOME_ALT);
	}

	else if  (req.noflyZoneCheck_src.ref_system.compare("ENU")==0 )
	{
		src.x = req.noflyZoneCheck_src.x_lat;
		src.y = req.noflyZoneCheck_src.y_long;
		src.z = req.noflyZoneCheck_src.z_alt;
	}

	// dest의 좌표계를 ENU로 변환 
	if (req.noflyZoneCheck_dest.ref_system.compare("WGS84")==0 )
	{
		dest = convertGeoToENU( req.noflyZoneCheck_dest.x_lat, req.noflyZoneCheck_dest.y_long, req.noflyZoneCheck_dest.z_alt, HOME_LAT, HOME_LON, HOME_ALT);
	}

	else if  (req.noflyZoneCheck_dest.ref_system.compare("ENU")==0 )
	{
	//	cout << "destination coord type: ENU " << endl;
		dest.x = req.noflyZoneCheck_dest.x_lat;
		dest.y = req.noflyZoneCheck_dest.y_long;
		dest.z = req.noflyZoneCheck_dest.z_alt;
	}
	
        // 경계점 목록의 좌표계를 ENU로 변환


	for(vector<NoflyZone>::iterator it = nfZones.noflyZones.begin(); 
		it != nfZones.noflyZones.end();
		it++)
	{
		vector<Point> boundary_pts;	

		NoflyZone nfz = *it; 

		for(vector<Target>::iterator it = nfz.noflyZone_pts.begin(); 
			it != nfz.noflyZone_pts.end();
			it++ )
		{
			Target target = *it;
			Point point;
		
			if (nfz.noflyZone_ref_system == "WGS84")
			{
				point = convertGeoToENU(target.x_lat, target.y_long, target.z_alt, HOME_LAT, HOME_LON, HOME_ALT );
			}
			else if (nfz.noflyZone_ref_system == "ENU")
 
			{
				point.x = target.x_lat;
				point.y= target.y_long;
				point.z = target.z_alt;
			}

			boundary_pts.push_back(point);
		}

		 if (isInside(dest, boundary_pts) == true)
	        {
                	cout << " dest is inside a noflyZone!!" << endl;
                        res.result = "DST_IN_NF";
			return true; 
       		}
		
		if (pathOverlap(src, dest, boundary_pts)==true )
		{
			cout << "src-dest path overlaps the noflyZone!" << endl;
			res.result = "PATH_OVERLAP";
			return true;
		} 
	
	} // 각각의 비행금지구역에 대해 반복 

	/*
	for (vector<Target>::iterator it = req.noflyZoneCheck_noflyZone.noflyZone_pts.begin();
		it != req.noflyZoneCheck_noflyZone.noflyZone_pts.end();
		it++)
	{
		Target point = *it;

		Point boundary_point;

		if (req.noflyZoneCheck_noflyZone.noflyZone_ref_system.compare("WGS84"))
		{
			boundary_point = convertGeoToENU(point.x_lat, point.y_long, point.z_alt, HOME_LAT, HOME_LON, HOME_ALT);	
		}
		else if (req.noflyZoneCheck_noflyZone.noflyZone_ref_system.compare("ENU"))
		{	
			boundary_point.x = point.x_lat;
			boundary_point.y = point.y_long; 
			boundary_point.z = point.z_alt;	
	 
		}

		boundary_pts.push_back(boundary_point);
	}
	*/

	//#3 src -dest 직선 경로와 비행금지구역이 겹치는 지 확인 


	
	
        sleep(10);



	//#4 결과값 반환 

	return true;





///////////////////////////////////////////// 기존 소스 /////////////////////////////////////////

  /* 비행금지구역 정보 반환 */
 // res.pt1 = nofly_zone.pt1;
 // res.pt2 = nofly_zone.pt2;
 
  /*
  res.pt1_arg1 = nofly_zone.pt1_arg1;
  res.pt1_arg2 = nofly_zone.pt1_arg2;
  res.pt1_arg3 = nofly_zone.pt1_arg3;
  
  res.pt2_arg1 = nofly_zone.pt2_arg1;
  res.pt2_arg2 = nofly_zone.pt2_arg2;
  res.pt2_arg3 = nofly_zone.pt2_arg3;
  */
  /* 출발지, 목적지 정보를 입력 받아 비행 금지 구역과의 관계 반환 */
 

  
  //
/*
  res.value= false;
  ROS_INFO ("eDrone_safety_node: NoflyZoneCheck service was called");


  cout << "src: (" << req.NFCheck_src.y_long << " , " << req.NFCheck_src.x_lat << ") " <<endl;
  cout << "dst: (" << req.NFCheck_dst.y_long << " , " << req.NFCheck_dst.x_lat << ") " <<endl;

  if ( (nofly_zone.isSet != true) || req.NFCheck_ref_system.compare("WGS84") !=0 )
  {
    return true; // 비행 금지 구역이 미 설정되었거나 입력 좌표가 WGS84가 아니면 false 반환 
  }


  // 출발지와 비행 금지 구역 간 관계 확인
  if ( (req.NFCheck_src.x_lat < nofly_zone.pt1.x_lat) != ( req.NFCheck_src.x_lat < nofly_zone.pt2.x_lat)  )
  {
//	ROS_INFO ("TEST1");

	if ( (req.NFCheck_src.y_long < nofly_zone.pt1.y_long) != ( req.NFCheck_src.y_long < nofly_zone.pt2.y_long))
	{
		ROS_INFO ("Src is in the NF zone! "); // CASE#1: 출발지가 비행 금지 구역 내
		res.result.assign("SRC_IN_NF_ZONE" );
		res.value = true;
		return true;
 	}	
  }  
     // 목적지와 비행 금지 구역 간 관계 확인 
   if ( (req.NFCheck_dst.x_lat < nofly_zone.pt1.x_lat) != ( req.NFCheck_dst.x_lat < nofly_zone.pt2.x_lat)  )
  {
//	ROS_INFO ("TEST2");

	if ( (req.NFCheck_dst.y_long < nofly_zone.pt1.y_long) != ( req.NFCheck_dst.y_long < nofly_zone.pt1.y_long))
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
	directPath.x = req.NFCheck_dst.y_long - req.NFCheck_src.y_long;
	directPath.y = req.NFCheck_dst.x_lat - req.NFCheck_src.x_lat;
	directPath.z = 0;

	
	Vector_type rect_line[4];

	rect_line[0].x= nofly_zone.pt2.y_long - nofly_zone.pt1.y_long;
	rect_line[0].y= 0;

	rect_line[1].x= 0; 
	rect_line[1].y = nofly_zone.pt2.x_lat - nofly_zone.pt1.x_lat;
	rect_line[1].z = 0;

	rect_line[2].x = nofly_zone.pt1.y_long - nofly_zone.pt2.y_long;
	rect_line[2].y = 0;
	rect_line[2].z = 0;


	rect_line[3].x = 0;	
	rect_line[3].y = nofly_zone.pt1.x_lat - nofly_zone.pt2.x_lat;
	rect_line[3].z = 0;
	
	Point rect_point[4];

	rect_point[0].x = nofly_zone.pt1.y_long;
	rect_point[0].y = nofly_zone.pt1.x_lat;

	rect_point[1].x = nofly_zone.pt2.y_long;
	rect_point[1].y = nofly_zone.pt1.x_lat;

	rect_point[2].x = nofly_zone.pt2.y_long;
	rect_point[2].y = nofly_zone.pt2.x_lat;
	
	rect_point[3].x = nofly_zone.pt1.y_long;
	rect_point[3].y = nofly_zone.pt2.x_lat;

	//ROS_INFO ("TEST3-1");	


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

		vec1.x = rect_point [i].x - req.NFCheck_src.y_long;
		vec1.y = rect_point [i].y - req.NFCheck_src.x_lat;

		vec2.x = rect_point[ (i+1)%4].x - req.NFCheck_src.y_long;
		vec2.y = rect_point[ (i+1)%4].y - req.NFCheck_src.x_lat;


		product1 = crossproduct (directPath, vec1);		
		product2 = crossproduct (directPath, vec2);
	
		c1 = product1.z * product2.z;
	
		// rect_line[i]와 두 벡터 간의 외적 값의 곱 구하기

	
		Vector_type vec3, vec4;	
		
		vec3.x = req.NFCheck_src.y_long- rect_point[i].x;
		vec3.y = req.NFCheck_src.x_lat - rect_point[i].y;

		vec4.x = req.NFCheck_dst.y_long - rect_point[i].x;
		vec4.y = req.NFCheck_dst.x_lat - rect_point[i].y;
	

		product3 = crossproduct (rect_line[i], vec3);
		product4 = crossproduct (rect_line[i], vec4);
		
		c2 = product3.z * product4.z; 

		if ( (c1 <0) && (c2 < 0) ) // directPath와 rect_line[i]는 서로 교차함 
		{
//			ROS_INFO ("test3-2");
//			cout << "directPath: (" << directPath.x << ", " << directPath.y << " )" << endl;			   	       cout << "rect_line: (" << rect_line[i].x << ", " << rect_line[i].y << " )" << endl;

//			cout << "c1: " << c1 << endl;
//			cout << "c2: " << c2 << endl;

			crossing = true;
			overlap = true;

//			cout << "line crossing: " << i << endl;
		}
	}


	

   // 3) 직선 경로와 사각형의 변들 중 하나가 교차하는 경우 true 반환, 그렇지 않으면 false 반한

	if (overlap ==  true)
	{
		ROS_INFO ("Src-Dst direct path overlaps the NF zone! " );
		res.result.assign("PATH_OVERLAP_NF_ZONE");
	}

	else
	{
		ROS_INFO ("Src-Dst direct path does not overlap the NF zone! " );
		res.result.assign("NONE");
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
		double y2 = grad * nofly_zone.pt2.y_long + y_intercept;

		cout << "src: ( " << req.src_arg1 << ", " << req.src_arg2 << ") " << endl;
		cout << "dst: ( " << req.dst_arg1 << ", " << req.dst_arg2 << ") " << endl;


		cout <<" grad: " << grad << endl;
		cout <<"y_intercept: " << y_intercept << endl;
		cout << "y1: " << y1 << ", y2: " << y2 << endl;

		// 비행 금지 구역과 src-dst 직선 경로가 서로 겹치는 지 확인 
		if ( ((y1 < nofly_zone.pt1.x_lat) != (y1 <nofly_zone.pt2.x_lat) ) || 
			
			( (y2 < nofly_zone.pt1.x_lat) != ( y2 < nofly_zone.pt2.x_lat)  )  )
		{
	//		cout << "y1: " << y1 << ", y2: " << y2 << endl;
		
			ROS_INFO ("Src-Dst direct path overlaps the NF zone! " );
			res.result.assign("PATH_OVERLAP_NF_ZONE");
		}
      }
      else // src와 dst의 경도가 같을 경우 (기울기가 0)
      {
		if ( (req.dst_arg2 < nofly_zone.pt1_arg2) != (req.dst_arg2 < nofly_zone.pt2.y_long )  )
		{
			//ROS_INFO("TEST2-1");
			
			double min_lat = req.src_arg1;
			double max_lat = req.dst_arg1;

			double nofly_min_lat = nofly_zone.pt1.x_lat;
			double nofly_max_lat = nofly_zone.pt2.x_lat;
			double nofly_min_lon = nofly_zone.pt1_arg2;			
			double nofly_max_lon = nofly_zone.pt2.y_long;

			//cout << "min_lat: " << min_lat << ", max_lat: " << max_lat << endl;
			//cout << "nofly_min_lat: " << nofly_min_lat << ", nofly_max_lat: " << nofly_max_lat << endl;
		
			


			if ( req.dst_arg1 < req.src_arg1)
			{
				min_lat = req.dst_arg1;
				max_lat = req.src_arg1;
			}


			if (nofly_zone.pt2.x_lat < nofly_zone.pt1.x_lat )
			{
				nofly_min_lat = nofly_zone.pt2.x_lat;
				nofly_max_lat = nofly_zone.pt1.x_lat;
			}

			if (nofly_zone.pt2.y_long < nofly_zone.pt1_arg2 )
			{
				nofly_min_lon = nofly_zone.pt2.y_long;
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

  // */
 
}

bool srv_geofenceSet_cb(eDrone_msgs::GeofenceSet::Request &req, eDrone_msgs::GeofenceSet::Response &res)
{
  cout << " safety_node - geofenceSet_cb(): " << endl;
  geofence.geofence_radius = req.geofenceSet_radius; 

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
	ros::init(argc, argv, "eDrone_safety_node");
 	ros::NodeHandle nh; 
	ros::Rate rate (20.0); 
	geofence.geofence_radius = GEOFENCE_RADIUS;


	// publisher 초기화
	ros::Publisher noflyZones_pub = nh.advertise<eDrone_msgs::NoflyZones> ("eDrone_msgs/noflyZones", 10);
	ros::Publisher geofence_pub = nh.advertise<eDrone_msgs::Geofence> ("eDrone_msgs/geofence", 10);
	

	// subscriber 초기화
	state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
	pos_sub_local = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose",10,  pos_cb_local); 
	pos_sub_global = nh.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global",10,  pos_cb_global); 
	home_sub = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, homePosition_cb);

	// 서비스 서버 초기화
        geofenceSet_srv_server = nh.advertiseService ( "srv_geofenceSet", srv_geofenceSet_cb );
	noflyZoneSet_srv_server = nh.advertiseService ( "srv_noflyZoneSet", srv_noflyZoneSet_cb );	
	noflyZoneReset_srv_server = nh.advertiseService ( "srv_noflyZoneReset", srv_noflyZoneReset_cb );	
        noflyZoneCheck_srv_server = nh.advertiseService ( "srv_noflyZoneCheck", srv_noflyZoneCheck_cb );

	// 서비스 클라이언트 초기화

	// 반복적으로 NoflyZones topic publish 
	while ( ros::ok() ) // API 피 호출 시, 콜백 함수를 통해 해당 기능 처리 후 응답
	{
		// noflyZones topic publish 
		noflyZones_pub.publish(nfZones);	
		geofence_pub.publish(geofence);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
