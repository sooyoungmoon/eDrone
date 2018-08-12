

/* header file */


// C/C++ 관련 
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include <vector>

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

// eDrone 관련 
 // ROS Topics
#include <eDrone_msgs/Target.h> // 현재 목적지 topic 메시지가 선언된 헤더 파일 포함
#include <eDrone_msgs/Phase.h> // 무인기 임무 수행 단계  

 // ROS Services
#include <eDrone_lib/Vehicle.h>
#include <eDrone_lib/GeoInfo.h>
#include <eDrone_lib/GeoUtils.h>
#include <eDrone_msgs/Arming.h> // 시동 서비스 헤더 파일
#include <eDrone_msgs/Takeoff.h> // 이륙 서비스 헤더 파일
#include <eDrone_msgs/Landing.h> // 착륙 서비스 헤더 파일
#include <eDrone_msgs/Goto.h> // 무인기 위치 이동 서비스 헤더 파일 포함
#include <eDrone_msgs/ModeChange.h> // 비행 모드 변경 서비스 헤더 파일
#include <eDrone_msgs/RTL.h> // RTL
#include <eDrone_msgs/GeofenceSet.h> // GeofenceSet 서비스 헤더 파일
#include <eDrone_msgs/GeofenceCheck.h> // GeofenceCheck 서비스 헤더 파일
#include <eDrone_msgs/GeofenceReset.h> // GeofenceReset 서비스 헤더 파일
#include <eDrone_msgs/NoflyZoneSet.h> // 비행 금지 구역 설정
#include <eDrone_msgs/NoflyZoneReset.h> // 비행 금지 구역 해제
#include <eDrone_msgs/NoflyZoneCheck.h> // 비행 금지 구역 확인

using namespace std;
using namespace mavros_msgs;


typedef struct _str_target_position
{
	int target_seq_no; // 목적지 순번 (1, 2, 3, ...)
	bool is_global; // 좌표 유형 (true: global coord, false: local coord)
	string ref_system; // 좌표 유형 - is_global 변수 대체  
	
	Point pos_local; // (x, y, z)
	GeoPoint pos_global; // (lat, long, alt)

	bool reached; // 목적지 도달 여부 (true: 목적지 도착, false: 목적지로 이동 중)
	bool geofence_violation; // geofence 영역 위반 여부 (true: geofence 영역 밖, false: 영역 내) 

	bool noflyZone_violation; // 비행 금지 구역 위반 여부 (true: 비행 금지 구역 내, false: 비행 금지 구역 외)
	
	string noflyZone_status; // 비행 금지 구역과 Src, Dst 간 위치 관계
				// 1) SRC_IN_NF_ZONE
				// 2) DST_IN_NF_ZONE
				// 3) PATH_OVERLAP_NF_ZONE


} Target_Position;



//(2018.02.26) 

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


//// 목적지 변수

Target_Position target_position = { .target_seq_no = -1 }; // 현재 목적지 정보 (published topic)
int num_targets; // 목적지 개수 (goto service마다 1 씩 증가)
bool autonomous_flight = false; // 자율 비행 여부  (goto service가 호출되면 true로 변경됨)

float Geofence_Radius = 1000;
enum Geofence_Policy {Warning, RTL, Loiter, Landing};
enum Geofence_Policy geofence_policy = RTL;




//// 메시지 변수 선언

// (Subscribed topic data)
  // (무인기 상태 정보 수신 목적)
mavros_msgs::State current_state; // 무인기 상태 정보

  // (무인기 위치 확인 목적)
geometry_msgs::PoseStamped current_pos_local; // 현재 위치 및 자세 정보 (지역 좌표)
sensor_msgs::NavSatFix current_pos_global; // 현재 위치 정보 (전역 좌표)

  // (홈 위치 획득 목적)
mavros_msgs::HomePosition home_position; // home position


// (Published topic data)
  // (무인기 위치 이동 목적)
geometry_msgs::PoseStamped target_pos_local; // 목적지 위치 정보 (지역 좌표)
mavros_msgs::GlobalPositionTarget target_pos_global; // 목적지 위치 정보 (전역 좌표)

  // (현재 목적지 정보를 응용 프로그램에 전달)
eDrone_msgs::Target cur_target; // 현재 목적지 정보 (publisher: eDrone_control_node, subscriber: 응용 프로그램)

  // (현재 임무 정보를 응용 프로그램에 전달)
eDrone_msgs::Phase cur_phase; // 현재 임무수행단계 

// Path 정보 
std::vector<Target_Position> path; // 무인기 자율 비행 경로 



//// 서비스 요청 메시지 선언 (mavros)

mavros_msgs::CommandBool arming_cmd; // 시동 명령에 사용될 서비스 선언 
mavros_msgs::CommandTOL takeoff_cmd; // 이륙 명령에 사용될 서비스 선언 
mavros_msgs::CommandTOL landing_cmd; // 착륙 명령에 사용될 서비스 선언 
mavros_msgs::CommandLong commandLong_cmd;// 무인기 제어에 사용될 서비스 선언
mavros_msgs::SetMode modeChange_cmd; // 모드 변경에 사용될 서비스 요청 메시지
mavros_msgs::SetMode rtl_cmd; // 복귀 명령에 사용될 서비스 요청 메시지

eDrone_msgs::GeofenceCheck geofenceCheck_cmd; // 가상 울타리 확인에 사용될 요청 메시지 
eDrone_msgs::NoflyZoneCheck noflyZoneCheck_cmd; // 비행 금지 구역 확인에 사용될 요청 메시지 

// publisher 선언

//ros::Publisher state_pub;
ros::Publisher pos_pub_local;
ros::Publisher pos_pub_global;
ros::Publisher cur_target_pub; // (offboard control에 필요한) 현재 목적지 정보 (도착 여부 포함)
ros::Publisher cur_phase_pub; // 현재 무인기 임무 수행 단계 정보 (Q. 어떤 동작을 하고 있는지?) 

// subscriber 선언

ros::Subscriber state_sub;
ros::Subscriber pos_sub_local;
ros::Subscriber pos_sub_global;
ros::Subscriber home_sub; 

// 서비스 서버 선언
ros::ServiceServer arming_srv_server;
ros::ServiceServer takeoff_srv_server;
ros::ServiceServer landing_srv_server;
ros::ServiceServer modeChange_srv_server;
ros::ServiceServer rtl_srv_server;
ros::ServiceServer goto_srv_server;
ros::ServiceServer geofence_srv_server;
ros::ServiceServer noflyZoneSet_srv_server;
ros::ServiceServer noflyZoneReset_srv_server;
ros::ServiceServer noflyZoneCheck_srv_server;

ros::ServiceServer checkNFZone_srv_server;

//서비스 클라이언트 선언
ros::ServiceClient arming_client; // 서비스 클라이언트 선언
ros::ServiceClient takeoff_client; // 서비스 클라이언트 선언
ros::ServiceClient landing_client; // 서비스 클라이언트 선언
ros::ServiceClient modeChange_client; // 비행 모드 변경 서비스 클라이언트 선언
ros::ServiceClient rtl_client; // 복귀 서비스 클라이언트 
ros::ServiceClient geofenceCheck_client;
ros::ServiceClient noflyZoneCheck_client;

// home position

 float HOME_LAT ;
 float HOME_LON;
 float HOME_ALT;


// (2018.07.03) 서비스 수행 단계
// 최초 phase = UNARMED
// (시동 직후) ARMED
// (이륙 중) TAKEOFF 
// (이륙 직후) READY
// (GOTO 수행 중) GOTO
// ...

string phase = "UNARMED"; 

// (2018.07.03) 이륙 목적

double takeoff_altitude = 0; 



// src-dst 사이의 비행금지구역 우회 경로 계산 함수  
std::vector<Target_Position> getAltPath(Target_Position src, Target_Position dst)
{
	cout << " eDrone_control_node: getAltPath()" << endl;
	cout << "src: (" << src.pos_global.longitude << " , " << src.pos_global.latitude << ") " <<endl;
  	cout << "dst: (" << dst.pos_global.longitude << " , " << dst.pos_global.latitude << ") " <<endl;
	
	std::vector<Target_Position> path;
	noflyZoneCheck_cmd.request.ref_system = "WGS84";

	// 비행금지구역의 경계에 해당하는 꼭지점 (4개) 구하기 

	GeoPoint rect[4];
	noflyZoneCheck_cmd.request.src_arg1 = src.pos_global.latitude;
	noflyZoneCheck_cmd.request.src_arg2 = src.pos_global.longitude;;
	noflyZoneCheck_cmd.request.dst_arg1 = dst.pos_global.latitude;
	noflyZoneCheck_cmd.request.dst_arg2 = dst.pos_global.longitude;

	/* 비행금지구역 정보 획득 */

	if (noflyZoneCheck_client.call(noflyZoneCheck_cmd) == true)
	{
		// 비행금지구역 꼭지점 정보 구하기
		rect[0].latitude = noflyZoneCheck_cmd.response.pt1_arg1-0.001;
		rect[0].longitude = noflyZoneCheck_cmd.response.pt1_arg2-0.001;

		rect[1].latitude = noflyZoneCheck_cmd.response.pt1_arg1-0.001;
		rect[1].longitude = noflyZoneCheck_cmd.response.pt2_arg2+0.001;
		
		rect[2].latitude = noflyZoneCheck_cmd.response.pt2_arg1+0.001;
		rect[2].longitude = noflyZoneCheck_cmd.response.pt2_arg2+0.001;

		rect[3].latitude = noflyZoneCheck_cmd.response.pt2_arg1+0.001;
		rect[3].longitude = noflyZoneCheck_cmd.response.pt1_arg2-0.001;


		if (noflyZoneCheck_cmd.response.result.compare ("PATH_OVERLAP_NF_ZONE" ) !=0)
		{
			ROS_INFO("eDrone_control_node: getAltPath(): no need to compute an alternate path");
		}
	}

	

	/* src-dst 간 우회 경로 계산 */

	// SRC 다음에 이동할 Target (Target#1) 계산

	cout << "Target#1 계산 " << endl; 
	Target_Position target1, target2, target3;

	double distToDst = -1;

	int rectPointIdx = -1;

	for (int i = 0; i < 4; i++)
	{
		
		noflyZoneCheck_cmd.request.src_arg1 = src.pos_global.latitude;
		noflyZoneCheck_cmd.request.src_arg2 = src.pos_global.longitude;

		noflyZoneCheck_cmd.request.dst_arg1 = rect[i].latitude;
		noflyZoneCheck_cmd.request.dst_arg2 = rect[i].longitude;

		if (noflyZoneCheck_client.call (noflyZoneCheck_cmd) == true)
		{       // src - rect[i] 사이의 경로가 비행금지구역과 겹치지 않으면 rect[i]를 Target#1으로 지정 
			if (noflyZoneCheck_cmd.response.result.compare ("PATH_OVERLAP_NF_ZONE" ) !=0)
			{
					target1.is_global = true;
					target1.pos_global.latitude = rect[i].latitude;
					target1.pos_global.longitude = rect[i].longitude;
		
					rectPointIdx = i;

					// ENU로 좌표변환
					Point point = convertGeoToENU(target1.pos_global.latitude, target1.pos_global.longitude, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );

					target1.pos_local.x = point.x;
					target1.pos_local.y = point.y;
					target1.pos_local.z = point.z;
					path.push_back(target1);
					break;
			}
		}
	}

	// Target#1 - Dst 간 직선 경로가 비행금지구역과 겹치지 않으면 
	// 대체 경로는 <Target#1, Dst>

	// Target#1 - Dst 간 직선경로가 비행금지구역과 겹치면 Target#2 계산

	// Target#N - Dst 간 직선경로가 비행금지구역과 겹치지 않으면 

	// 대체 경로는 <Target#1, Target#2, ..., Target#N, Dst>


	// (Target#2) 계산
	cout << " Target#2 계산 " << endl;	
	noflyZoneCheck_cmd.request.src_arg1 = target1.pos_global.latitude;
	noflyZoneCheck_cmd.request.src_arg2 = target1.pos_global.longitude;
	noflyZoneCheck_cmd.request.dst_arg1 = dst.pos_global.latitude;
	noflyZoneCheck_cmd.request.dst_arg2 = dst.pos_global.longitude;

	if (noflyZoneCheck_client.call (noflyZoneCheck_cmd) == true)
	{
		if (noflyZoneCheck_cmd.response.result.compare ("PATH_OVERLAP_NF_ZONE" ) ==0)
		{
			// target#1-Dst 간 직선 경로가 NFZONE 과 겹침. 
			// target#2 계산 필요
			target2.is_global = true;
			target2.pos_global.latitude = rect[(rectPointIdx+1)%4 ].latitude;
 			target2.pos_global.longitude = rect[(rectPointIdx+1)%4 ].longitude;

			// ENU로 좌표변환
			Point point = convertGeoToENU(target2.pos_global.latitude, target2.pos_global.longitude, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );

			target2.pos_local.x = point.x;
			target2.pos_local.y = point.y;
			target2.pos_local.z = point.z;
			
			path.push_back(target2);
		}		
	}
	// (Target#3) 계산
	cout << " Target#3 계산 " << endl;	

	noflyZoneCheck_cmd.request.src_arg1 = target2.pos_global.latitude;
	noflyZoneCheck_cmd.request.src_arg2 = target2.pos_global.longitude;
	noflyZoneCheck_cmd.request.dst_arg1 = dst.pos_global.latitude;
	noflyZoneCheck_cmd.request.dst_arg2 = dst.pos_global.longitude;

	if (noflyZoneCheck_client.call (noflyZoneCheck_cmd) == true)
	{
		if (noflyZoneCheck_cmd.response.result.compare ("PATH_OVERLAP_NF_ZONE" ) ==0)
		{
			// target#1-Dst 간 직선 경로가 NFZONE 과 겹침. 
			// target#2 계산 필요
			target3.is_global = true;
			target3.pos_global.latitude = rect[(rectPointIdx+2)%4 ].latitude;
 			target3.pos_global.longitude = rect[(rectPointIdx+2)%4 ].longitude;

			// ENU로 좌표변환
			Point point = convertGeoToENU(target2.pos_global.latitude, target2.pos_global.longitude, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );

			target3.pos_local.x = point.x;
			target3.pos_local.y = point.y;
			target3.pos_local.z = point.z;
			path.push_back(target3);

		}		
	}


	path.push_back(dst);

/*	
	for (int i = 0; i < 4; i++)
	{
		noflyZoneCheck_cmd.request.src_arg1 = rect[i].latitude;
		noflyZoneCheck_cmd.request.src_arg2 = rect[i].longitude;
	
		noflyZoneCheck_cmd.request.dst_arg1 = dst.pos_global.latitude;
		noflyZoneCheck_cmd.request.dst_arg2 = src.pos_global.longitude;

		if (noflyZoneCheck_client.call (noflyZoneCheck_cmd) == true)
		{       // rect[i]-dst 사이의 경로가 비행금지구역과 겹치지 않으면 rect[i]를 Target#2로 지정 
			if (noflyZoneCheck_cmd.response.result.compare ("PATH_OVERLAP_NF_ZONE" ) !=0)
			{
					target2.is_global = true;
					target2.pos_global.latitude = rect[i].latitude;
					target2.pos_global.longitude = rect[i].longitude;

					// ENU로 좌표변환
					Point point = convertGeoToENU(target2.pos_global.latitude, target2.pos_global.longitude, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );
					target2.pos_local.x = point.x;
					target2.pos_local.y = point.y;
					target2.pos_local.z = point.z;
	
			}
		}
	}
*/


	// 만약 Target#1과 Target#2가 같으면 우회 경로는 <Target#1, DST >
	// 다르면 우회 경로는 <Target#1, Target#2, DST> 가 됨 
/*	
	if ( (target1.pos_global.latitude == target2.pos_global.latitude ) && (target1.pos_global.latitude == target2.pos_global.latitude) )
	{
		path.push_back(target1);
		path.push_back(target2);
		path.push_back(dst);
	} 
	else 
	{
		path.push_back(target1);
		path.push_back(dst);
	}*/
	return path; 
}

void printAltPath (std::vector<Target_Position> altPath)
{
	//  대체 path 정보를 화면에 출력  
	cout << "eDrone_control_node: printAltPath() " << endl;
	int cnt = 0;

	 for (std::vector<Target_Position>::iterator it = altPath.begin() ; it != altPath.end(); ++it)
	{
		cnt++;
		Target_Position target = *it;

		cout << "Target#" << cnt << ": lat: " << target.pos_global.latitude << ", lon: " << target.pos_global.longitude << endl;

		
	}


}



void ref_system_conversion_test()
{
	printf("reference system conversion test: \n");

	printf("GeoPoint => Point\n");
	
	Point point = convertGeoToENU(HOME_LAT, HOME_LON, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );

	//Point point = convertGeoToENU(36.3847751, 127.3689272, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );
	cout << "(" << HOME_LAT  << ", " << HOME_LON << ", " << HOME_ALT << ") =  " << endl;

	cout << "(" << point.x << ", " << point.y << ", " << point.z << ") " << endl;


	printf("GeoPoint => Point\n");


	point = convertGeoToENU(36.3847732, 127.3661727, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );

	cout << "(" << 36.3847732  << ", " << 127.3661727 << ", " << HOME_ALT << ") =  " << endl;

	cout << "(" << point.x << ", " << point.y << ", " << point.z << ") " << endl;


	printf("Point => GeoPoint\n");

	GeoPoint geoPoint = convertENUToGeo( 0.0, 0.0, 0.0, HOME_LAT, HOME_LON, HOME_ALT);
	
	cout << "(0, 0, 0) =  " << endl;
	cout << fixed;
	cout << "(" << geoPoint.latitude << ", " << geoPoint.longitude << ", " << geoPoint.altitude << ") " << endl;

}


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


	if (phase.compare("TAKEOFF")==0)
	{
		//cout << "TAKEOFF is ongoing: takeoff_altitude is " << takeoff_altitude << endl;


		if (current_pos_local.pose.position.z >= takeoff_altitude-0.01 )
		{
			cout << "eDrone_control_node: TAKEOFF completed!" << endl;

			phase = "READY"; 
			cur_phase.phase = phase;
		}
	
		return;
	}


	if (phase.compare("GOTO")==0)
	{	
	
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
					
			
					}

				
				}

			}
		}

	}
}

void pos_cb_global(const sensor_msgs::NavSatFix::ConstPtr& msg){

	

	current_pos_global = *msg;


//					printf("current_position: (%f, %f, %f \n)", current_pos_global.latitude, current_pos_global.longitude, current_pos_global.altitude-HOME_ALT);
	
	if (phase.compare("GOTO")==0)
	{	

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
								
					}

				
				}

			}
		}
	}
}



void homePosition_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
	home_position = *msg;
	
//	printf("home position: (%f, %f, %f) \n", home_position.position.x, home_position.position.y, home_position.position.z);

	HOME_LAT = home_position.geo.latitude;
	HOME_LON = home_position.geo.longitude;		
	HOME_ALT = home_position.geo.altitude;
	
	static int print_count =0;	
	
	if (print_count < 10)
	{
		printf("control_node: home position: (%f, %f, %f) \n", HOME_LAT, HOME_LON, HOME_ALT);	
		print_count++;
	}

}


// callback 함수 (서비스 제공) 정의

bool srv_arming_cb(eDrone_msgs::Arming::Request &req, eDrone_msgs::Arming::Response &res )
{

	ROS_INFO("ARMing request received\n");
	arming_cmd.request.value = true; // 서비스 요청 메시지 필드 설정

	//// Arming

  	while (ros::ok() ) // 서비스 요청 메시지 전달
  	{
  	      printf("eDrone_control_node: send Arming command ...\n");

		if (!arming_client.call(arming_cmd))
        	 {
	 	  ros::spinOnce();
 //   	  	 rate.sleep();
        	}
        	else break;

  	} 
	 
	ROS_INFO("ARMing command was sent\n");

	
	return true;
}




bool srv_takeoff_cb(eDrone_msgs::Takeoff::Request &req, eDrone_msgs::Takeoff::Response &res)
{

	takeoff_altitude = req.altitude; // 이륙 고도 저장 
	//double offset = 25;

	ROS_INFO("***Takeoff request received\n");
	// 서비스 요청 메시지 필드 선언

	takeoff_cmd.request.altitude = 2.5;

	//takeoff_cmd.request.altitude = HOME_ALT + req.altitude;
	//takeoff_cmd.request.altitude = HOME_ALT  + req.altitude - offset;

	ROS_INFO(" HOME_ALT: %lf, req.altitude: %lf", HOME_ALT, req.altitude);
  	takeoff_cmd.request.latitude = HOME_LAT; // 자동으로 home position 값을 얻어 와서 설정되도록 변경 필요
  	takeoff_cmd.request.longitude = HOME_LON;
 	takeoff_cmd.request.yaw = 0;
  	takeoff_cmd.request.min_pitch = 0;

	// 서비스 요청 메시지 전달 
  
	while (ros::ok() )
  	{
 		printf("send Takeoff command ...\n");
 	
		if (!takeoff_client.call(takeoff_cmd))
		{
		  ros::spinOnce();
                //  rate.sleep();
		}
		else break;
  	}
	phase = "TAKEOFF"; 
	cur_phase.phase = phase;

 	ROS_INFO("Takeoff command was sent\n");
	res.value = true;
}

bool srv_landing_cb(eDrone_msgs::Landing::Request &req, eDrone_msgs::Landing::Response &res)
{
	
	ROS_INFO("Landing request received\n");
	//// 서비스 요청 메시지 필드 설정 

	landing_cmd.request.altitude = 10;

  	landing_cmd.request.latitude = HOME_LAT;

  	landing_cmd.request.longitude = HOME_LON;

  	landing_cmd.request.min_pitch = 0;

  	landing_cmd.request.yaw = 0;

	//// 서비스 요청 메시지 전달 

	//// Landing


	// 현재 이륙 중이면 false 반환 
	if (phase.compare("TAKEOFF") ==0)
	{
		res.value = false;
		return true;
	}

        // landing 서비스 호출 

 	 while (ros::ok() )
  	{
		printf("send Landing command ...\n");

		if (!landing_client.call(landing_cmd))
		{
	 	 ros::spinOnce();
	 	 // rate.sleep();
		}
		else break;
  	
 	} 
 	ROS_INFO("Landing command was sent\n");
	res.value = true;
	return true;
}

bool srv_modeChange_cb(eDrone_msgs::ModeChange::Request &req, eDrone_msgs::ModeChange::Response &res)
{
	
	std::cout << "srv_modeChange_cb(): change the mode to " << req.mode << endl; 

	modeChange_cmd.request.base_mode = 0;
        
	modeChange_cmd.request.custom_mode.assign(req.mode);

       if (modeChange_client.call(modeChange_cmd)==true)
	{
		std::cout << " modeChange cmd was sent!\n " << endl;
	}

	
	res.value = true;

	return true;
}

bool srv_rtl_cb(eDrone_msgs::RTL::Request &req, eDrone_msgs::RTL::Response &res)
{
	std::cout << "srv_rtl_cb():return to home"  << endl; 

	modeChange_cmd.request.base_mode = 0;
        
	modeChange_cmd.request.custom_mode.assign("AUTO.RTL");

       if (modeChange_client.call(modeChange_cmd)==true)
	{
		std::cout << " modeChange cmd was sent!\n " << endl;
	}
	phase = "RTL";

	cur_phase.phase= phase;

	res.value = true;
	return true;
}


bool srv_goto_cb(eDrone_msgs::Goto::Request &req, eDrone_msgs::Goto::Response &res)
{	

	ROS_INFO("eDrone_control_node: Goto request received\n");
	printf("eDrone_control_node: target_seq_no: %d\n", target_position.target_seq_no);


	cout<< "req.x_lat: " << req.x_lat << ", req.y_long: " << req.y_long << endl;

	/* goto 서비스 처리 절차 */

	// 좌표계 종류 확인 (ex. WGS84, ENU) & 목적지 좌표 (지역, 전역) 저장 
	if (req.ref_system.compare("WGS84")==0) // 전역 좌표인 경우
	{
		target_position.pos_global.latitude = req.x_lat;
		target_position.pos_global.longitude = req.y_long;
		target_position.pos_global.altitude = req.z_alt;


		// ENU로 좌표변환
		Point point = convertGeoToENU(req.x_lat, req.y_long, req.z_alt, HOME_LAT, HOME_LON, HOME_ALT );

		target_position.pos_local.x = point.x;
		target_position.pos_local.y = point.y;
		target_position.pos_local.z = point.z;

	}

	else if (req.ref_system.compare("ENU")==0)
	{
		target_position.pos_local.x = req.x_lat;	
		target_position.pos_local.y = req.y_long;
		target_position.pos_local.z = req.z_alt;

		// WGS84로 좌표 변환
		GeoPoint geoPoint = convertENUToGeo(req.x_lat, req.y_long, req.z_alt, HOME_LAT, HOME_LON, HOME_ALT );
		cout <<"converENUToGeo() was called" << endl;

		target_position.pos_global.latitude = geoPoint.latitude;	
		target_position.pos_global.longitude = geoPoint.longitude;		
		target_position.pos_global.altitude = geoPoint.altitude;		

	}

	// Geofence 검사

		geofenceCheck_cmd.request.ref_system = "WGS84";	
		geofenceCheck_cmd.request.arg1= target_position.pos_global.latitude;
        	geofenceCheck_cmd.request.arg2= target_position.pos_global.longitude;
		geofenceCheck_cmd.request.arg3= target_position.pos_global.altitude;

	//	geofenceCheck_cmd.request.arg1=  req.x_lat;
        //	geofenceCheck_cmd.request.arg2= req.y_long;
	//	geofenceCheck_cmd.request.arg3= req.z_alt;
		
		ROS_INFO("eDrone_control_node: trying to call GeofenceCheck service");

		if (geofenceCheck_client.call (geofenceCheck_cmd) == true)
		{

			if (geofenceCheck_cmd.response.value == true )
			{
				if (geofenceCheck_cmd.response.violation == true)
				{			
					target_position.geofence_violation = true;				
					ROS_INFO("eDrone_control_node: Goto service rejected: geofence violation!\n");
					res.value = false;
					return true;
				}
			else
			{
				target_position.geofence_violation = false; 				
			}		
		}		
	}


	// NoflyZone 검사 

 	// path 목록에 목적지 정보 추가   
		// (비행 금지 구역과 src-dst 간 직선경로가 겹치는 경우, path 재설정 필요)

		
		ROS_INFO("eDrone_control_node: trying to call noflyZoneCheck service");

		noflyZoneCheck_cmd.request.ref_system = "WGS84";

		noflyZoneCheck_cmd.request.src_arg1 = current_pos_global.latitude;
 		noflyZoneCheck_cmd.request.src_arg2 = current_pos_global.longitude;
		noflyZoneCheck_cmd.request.src_arg3 = current_pos_global.altitude;
		noflyZoneCheck_cmd.request.dst_arg1 = target_position.pos_global.latitude;
		noflyZoneCheck_cmd.request.dst_arg2 = target_position.pos_global.longitude;
		noflyZoneCheck_cmd.request.dst_arg3 = target_position.pos_global.altitude;

	//	noflyZoneCheck_cmd.request.dst_arg1 = req.x_lat;
	//	noflyZoneCheck_cmd.request.dst_arg2 = req.y_long;
	//	noflyZoneCheck_cmd.request.dst_arg3 = req.z_alt;

		cout<< "_req.x_lat: " << req.x_lat << ", req.y_long: " << req.y_long << endl;
	

		if (noflyZoneCheck_client.call(noflyZoneCheck_cmd) == true)
		{
			// 현재 위치가 비행 금지 구역 이내이면 Home으로 복귀 

			if (noflyZoneCheck_cmd.response.result.compare ("SRC_IN_NF_ZONE" ) ==0) 
			{
				ROS_INFO("CASE#1: noflyZoneCheck result: SRC_IN_NF_ZONE");

				target_position.noflyZone_status = "SRC_IN_NF_ZONE";

				rtl_client.call(rtl_cmd); // 이륚 위치로 복귀 
				phase = "RTL";		
				res.value = false;
				return true;
			}


			// 목적지가 비행 금지 구역 이내이면 요청 거부 
			else if (noflyZoneCheck_cmd.response.result.compare ("DST_IN_NF_ZONE" ) ==0)
			{
				ROS_INFO("CASE#2: noflyZoneCheck result: DST_IN_NF_ZONE");
				res.value= false;
				res.value = false;
				return true;
			}
			
			// src-dst 간 직선 경로가 비행금지구역과 겹치는 경우
			else if (noflyZoneCheck_cmd.response.result.compare ("PATH_OVERLAP_NF_ZONE" ) ==0)
			{
				ROS_INFO("CASE#3: noflyZoneCheck result: PATH_OVERLAP_NF_ZONE");

				Target_Position src;
				src.pos_global.latitude = current_pos_global.latitude;
				src.pos_global.longitude = current_pos_global.longitude;			

				Target_Position dst;
				dst.pos_global.latitude = target_position.pos_global.latitude;
				dst.pos_global.longitude = target_position.pos_global.longitude;			

				std::vector<Target_Position> altPath = getAltPath(src, dst);

				printAltPath(altPath);		
				path.insert( path.end(), altPath.begin(), altPath.end() );
				// 우회 경로를 path에 저장 

				
			}

			// src, dst가 비행금지구역과 독립적으로 위치한 경우 	
			else
			{
				ROS_INFO("CASE#4: noflyZoneCheck result: NONE");
				// path에 목적지 정보 저장
				target_position.reached = false;
				path.push_back (target_position);
				//target_position.target_seq_no++; // target_position 에 실제로 정보를 저장할 때 sequ_no 증가 
			}

		}	
	
	return true;
}	

	// ========================================================== 이전 소스 =======================================================
	// # 요청 받은 위치 정보 저장

	// # noflyZone, geofence 위반 여부를 가리키는 변수 값 reset (false)

	// 현재 target (목적지)의 sequnce number 지정 (이전 값에 +1)

	// autonomous_flight 변수: goto 서비스가 한 번이라도 호출된 경우 true, 그렇지 않은 경우 false 값을 가짐 -> 목적지 좌표를 publish 할 것인가를 결정하는데 사용됨 
/*
	double distance_home = 0;

	target_position.reached = false;
	cur_target.reached = false;

	target_position.is_global = req.is_global;
	target_position.target_seq_no++;
	cur_target.target_seq_no = target_position.target_seq_no;
        //target_position.target_seq_no = req.target_seq_no; 
        autonomous_flight = true;

	target_position.geofence_violation = false;
	target_position.noflyZone_violation = false; 

	ROS_INFO("eDrone_control_node: Goto request received\n");
	printf("eDrone_control_node: target_seq_no: %d\n", target_position.target_seq_no);
*/

	
	// 전역 좌표인 경우, 지역 좌표롤 변환하여 사용 	
	
	// geofence 위반 여부 검사 -> 가상 울타리 밖인 경우, 서비스 거부

	// noflyZone 위반 여부 검사
	  // 목적지가 noflyZone 내부이면 서비스 거부

	  // 목적지가 noflyZone 외부이더라도 현재 위치 - 목적지 사이의 최단 경로 상에 비행금지구역이 위치하면 새로 경로를 구성해야 함
/*
	

	
	if (req.is_global) // 전역 좌표인 경우
	{

		/* Geofence check */				
/*
		geofenceCheck_cmd.request.ref_system = "WGS84";
		geofenceCheck_cmd.request.arg1=  req.x_lat;
        	geofenceCheck_cmd.request.arg2= req.y_long;
		geofenceCheck_cmd.request.arg3= req.z_alt;
		
		ROS_INFO("eDrone_utility_node: trying to call GeofenceCheck service");

	if (geofenceCheck_client.call (geofenceCheck_cmd) == true)
	{

		if (geofenceCheck_cmd.response.value == true )
		{
			if (geofenceCheck_cmd.response.violation == true)
			{			
				target_position.geofence_violation = true;				
				ROS_INFO("eDrone_control_node: Goto service rejected: geofence violation!\n");
				res.value = false;
				return true;
			}
			else
			{
				target_position.geofence_violation = false; 				
			}		
		}		
	}

		/* Geofence check 완료 */	
		

		/* 비행 금지 구역 check */		
/*		noflyZoneCheck_cmd.request.ref_system = "WGS84";
		noflyZoneCheck_cmd.request.arg1 = req.x_lat;
		noflyZoneCheck_cmd.request.arg2 = req.y_long;
		
		ROS_INFO("eDrone_control_node: trying to call NoflyZoneCheck service... ");
	
		if (noflyZoneCheck_client.call (noflyZoneCheck_cmd)==true)
		{
			if (noflyZoneCheck_cmd.response.violation == true)
			{

				target_position.noflyZone_violation = true; // 비행 금지 구역 위반 
				ROS_INFO ("eDrone_control_node: Goto service rejected: noflyZone violation!");
				res.value = false;
				return true;
			}
			else
			{
				target_position.noflyZone_violation = false; 
				//res.value = true;
			}
	
		}

		/* 비행 금지 구역 check 완료 */

/*
			
		if ( target_position.noflyZone_violation = false && target_position.geofence_violation == false)
		{
			res.value = true;
		}


		/*
		target_position.pos_global.latitude = req.x_lat;
		target_position.pos_global.longitude = req.y_long;
		target_position.pos_global.altitude = req.z_alt;

		target_pos_global.latitude = req.x_lat;
		target_pos_global.longitude = req.y_long;
		target_pos_global.altitude = req.z_alt;
		*/
/*		
		Point point = convertGeoToENU(req.x_lat, req.y_long, req.z_alt, HOME_LAT, HOME_LON, HOME_ALT );

		//Point point = convertGeoToENU(req.x_lat, req.y_long, req.z_alt, HOME_LAT, HOME_LON, HOME_ALT);

		cout << "convertGeoToENU() was called " << endl;		

		target_position.is_global = false; 	
		req.is_global = false;
		target_position.pos_local.x = point.x;
		target_position.pos_local.y = point.y;
		target_position.pos_local.z = point.z;

		//target_position.pos_local.z = HOME_ALT;

		target_pos_local.pose.position.x = point.x;
		target_pos_local.pose.position.y = point.y;

		//target_pos_local.pose.position.z = HOME_ALT;
		target_pos_local.pose.position.z = point.z;

		printf("target position (ENU): (%f, %f, %f)\n", point.x, point.y, point.z);
		sleep(10); 
		target_position.geofence_violation = false;
	}
	else // 지역 좌표인 경우
	{
		// 좌표 변환 (ENU -> WGS84)
		GeoPoint geoPoint = convertENUToGeo(req.x_lat, req.y_long, req.z_alt, HOME_LAT, HOME_LON, HOME_ALT );
		cout <<"converENUToGeo() was called" << endl;
		/* Geofence check */				
/*
		geofenceCheck_cmd.request.ref_system = "WGS84";
		geofenceCheck_cmd.request.arg1= geoPoint.latitude;
        	geofenceCheck_cmd.request.arg2= geoPoint.longitude;		
		
		ROS_INFO("eDrone_utility_node: trying to call GeofenceCheck service");

	if (geofenceCheck_client.call (geofenceCheck_cmd) == true)
	{

		if (geofenceCheck_cmd.response.value == true )
		{
			if (geofenceCheck_cmd.response.violation == true)
			{			
				target_position.geofence_violation = true;				
				ROS_INFO("eDrone_control_node: Goto service rejected: geofence violation!\n");
				res.value = false;
				return true;
			}
			else
			{
				target_position.geofence_violation = false; 				
			}		
		}		
	}

		/* Geofence check 완료 */	
		
		/* 비행 금지 구역 check */ 
/*
		noflyZoneCheck_cmd.request.ref_system = "WGS84";

		noflyZoneCheck_cmd.request.arg1 = geoPoint.latitude;
                noflyZoneCheck_cmd.request.arg2 = geoPoint.longitude;

                ROS_INFO("eDrone_control_node: trying to call NoflyZoneCheck service... ");

                if (noflyZoneCheck_client.call (noflyZoneCheck_cmd)==true)
                {
                        if (noflyZoneCheck_cmd.response.violation == true)
                        {
                                target_position.noflyZone_violation = true; // 비행 금지 구역 위반 
                                ROS_INFO ("eDrone_control_node: goto service rejected: noflyZone violation!");
                                res.value = false;
				return true;
                        }
                        else
                        {
                                target_position.noflyZone_violation = false;
                                //res.value = true;
                        }

                }
	
		
		if ( target_position.noflyZone_violation = false && target_position.geofence_violation == false)
		{
			res.value = true;
		}


		
		target_pos_local.pose.position.x = req.x_lat;

		target_pos_local.pose.position.y = req.y_long;
		target_pos_local.pose.position.z = req.z_alt;

		printf("target position (ENU): (%f, %f, %f)\n", target_pos_local.pose.position.x, target_pos_local.pose.position.y, target_pos_local.pose.position.z);
		

		distance_home = pow(req.x_lat, 2.0) + pow(req.y_long, 2.0);
		distance_home = sqrt(distance_home);
/*
		if (distance_home > Geofence_Radius)
		{
			printf("goto service was rejected: the target is out of the geofence boundary\n");
			target_position.geofence_violation = true;
			res.value = false;
		}
		else
*/	/*	{
			printf("goto the target: (%f, %f, %f)\n", req.x_lat, req.y_long, req.z_alt);
		//	target_position.reached = false;
			sleep(5);
		//	cur_target.reached = false;
			target_position.geofence_violation = false;
			
			target_position.pos_local.x = req.x_lat;
			target_position.pos_local.y = req.y_long;
			target_position.pos_local.z = req.z_alt;
		
			target_pos_local.pose.position.x = req.x_lat;
			target_pos_local.pose.position.y = req.y_long;
			target_pos_local.pose.position.z = req.z_alt;
			res.value = true;
		}
	}
	return true;
}*/
/*
bool srv_geofence_cb(eDrone_msgs::Geofence::Request &req, eDrone_msgs::Geofence::Response &res)
{
	Geofence_Radius = req.radius;

	switch (req.action)
	{
	case Warning:
		
		geofence_policy = Warning;	
		break;

	case RTL:
		geofence_policy = RTL;
		break;
	
	case Loiter:
		geofence_policy = Loiter;
		break;

	case Landing:

		geofence_policy = Landing;
		break;

	default:
		geofence_policy = RTL;
		break;
	}	

	res.value = true;
	return true;
}*/
/*
bool srv_noflyZoneSet_cb(eDrone_msgs::NoflyZoneSet::Request &req, eDrone_msgs::NoflyZoneSet::Response &res)
{
  
  // reference system: WGS84 지원

  ROS_INFO ("eDrone_control_node: NoflyZoneSet service was called");

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
  res.value= false;
  ROS_INFO ("eDrone_control_node: NoflyZoneCheck service was called");

  // 요청 메시지 포함된 좌표가  

  if ( (nofly_zone.isSet == true) &&  ( req.ref_system.compare("WGS84") ==0) )
  {
		ROS_INFO ("Fist condition");

  	if ( (req.arg1 < nofly_zone.pt1_arg1 ) !=  (req.arg1 < nofly_zone.pt2_arg1 ) )
  	{
		
		ROS_INFO ("Second condition");

		if ((req.arg2 < nofly_zone.pt1_arg2) != (req.arg2 < nofly_zone.pt2_arg2) )
		{
		
			ROS_INFO ("eDrone_control_node: NoflyZone violation!");
			res.violation = true;
		}
  	}
  }
  
  res.value = true;
  return true;
}*/
/*
bool srv_noflyZone_cb(eDrone_msgs::NoflyZone::Request &req, eDrone_msgs::NoflyZone::Response &res)
{
	ROS_INFO ("nofly_zone service was called:\n");
	
	nofly_zone.isSet = true;

	nofly_zone.min_x_lat = req.min_x_lat;
		
	nofly_zone.min_y_long = req.min_y_long;

	nofly_zone.max_x_lat = req.max_x_lat;

	nofly_zone.max_y_long = req.max_y_long;


	cout << "x_lat: " << nofly_zone.min_x_lat << " ~ " << nofly_zone.max_x_lat << endl;

	cout << "y_long: " << nofly_zone.min_y_long << " ~ " << nofly_zone.max_y_long << endl;


	return true;

}

bool srv_checkNFZone_cb (eDrone_msgs::CheckNFZone::Request &req, eDrone_msgs::CheckNFZone::Response &res)
{
	ROS_INFO ("checkNFZone service was called:\n");

	if (nofly_zone.isSet == true)
	{
		res.isSet = true;

		res.min_x_lat = nofly_zone.min_x_lat;
		res.min_y_long = nofly_zone.min_y_long;
		res.max_x_lat = nofly_zone.max_x_lat;
		res.max_y_long = nofly_zone.max_y_long;
	}

	else
	{
		res.isSet = false;
	}
	
	
}
*/

int main(int argc, char** argv)

{

	ros::init(argc, argv, "eDrone_control_node");

 	ros::NodeHandle nh; 
	
	ros::Rate rate (20.0); 

	// publisher 초기화
	pos_pub_local = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
        pos_pub_global = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global", 10);
	cur_target_pub = nh.advertise<eDrone_msgs::Target>("eDrone_msgs/current_target", 10);
	cur_phase_pub = nh.advertise<eDrone_msgs::Phase>("eDrone_msgs/current_phase", 10);

	// subscriber 초기화
	state_sub = nh.subscribe<mavros_msgs::State> ("mavros/state", 10, state_cb);
	pos_sub_local = nh.subscribe<geometry_msgs::PoseStamped> ("mavros/local_position/pose",10,  pos_cb_local); 
	pos_sub_global = nh.subscribe<sensor_msgs::NavSatFix> ("mavros/global_position/global",10,  pos_cb_global); 
	home_sub = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, homePosition_cb);

	//// 서비스 서버 선언

	arming_srv_server = nh.advertiseService("srv_arming", srv_arming_cb);
	takeoff_srv_server = nh.advertiseService("srv_takeoff", srv_takeoff_cb);
	landing_srv_server = nh.advertiseService("srv_landing", srv_landing_cb);	
	modeChange_srv_server = nh.advertiseService("srv_modeChange", srv_modeChange_cb);
	rtl_srv_server = nh.advertiseService("srv_rtl", srv_rtl_cb);
	goto_srv_server = nh.advertiseService("srv_goto", srv_goto_cb); 
	//geofence_srv_server = nh.advertiseService("srv_geofence", srv_geofence_cb);		
//	noflyzone_srv_server = nh.advertiseService("srv_noflyZone", srv_noflyZone_cb);	
	//checkNFZone_srv_server = nh.advertiseService("srv_checkNFZone", srv_checkNFZone_cb);
	
	// noflyZoneSet_srv_server = nh.advertiseService("srv_noflyZoneSet", srv_noflyZoneSet_cb);

	// noflyZoneCheck_srv_server = nh.advertiseService("srv_noflyZoneCheck", srv_noflyZoneCheck_cb);

	// noflyZoneReset_srv_server = nh.advertiseService("srv_noflyZoneReset", srv_noflyZoneReset_cb);

	//// 서비스 클라이언트 선언

        arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming"); // service client 선언
	takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/takeoff"); // 서비스 클라이언트 선언
	landing_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/land"); // 서비스 클라이언트 선언
	noflyZoneCheck_client = nh.serviceClient<eDrone_msgs::NoflyZoneCheck> ("srv_noflyZoneCheck" );
	geofenceCheck_client = nh.serviceClient<eDrone_msgs::GeofenceCheck> ("srv_geofenceCheck" );
	rtl_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
	modeChange_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
	target_position.reached = false; // 목적지 도착 여부를 false로 초기화

//        ROS_INFO("control_node: home position: (%f, %f, %f) \n", HOME_LAT, HOME_LON, HOME_ALT); 	

	while (ros::ok() )
	{

		// 현재 phase 토픽 출판

		cur_phase.phase = phase;
		cur_phase_pub.publish (cur_phase);

		if (phase.compare ("UNARMED") ==0)
		{
			// 시동 명령 대기 
		}
		else if (phase.compare ("ARMED") ==0)
		{
			// 이륙 명령 대기
		}
		else if (phase.compare ("TAKEOFF") == 0) // 현재 이륙 중인 경우 
		{
			//cout << "eDrone_control_node: publish Topic to takeoff - altitude " << takeoff_altitude <<  endl;	

			// change mode to offboard
			modeChange_cmd.request.base_mode = 0;
			modeChange_cmd.request.custom_mode = "OFFBOARD";
			if (modeChange_client.call(modeChange_cmd))
			{
				if (modeChange_cmd.response.mode_sent)
				{;
					//cout << "offboard enabled" << endl;
				}

			}	
				
			// set topic field values
			target_pos_local.pose.position.x = 0;
			target_pos_local.pose.position.y = 0;	
			target_pos_local.pose.position.z = takeoff_altitude;	
			
			// publish topic
			
			pos_pub_local.publish (target_pos_local);
			ros::spinOnce();
			rate.sleep();
		}
		else if (phase.compare ("READY") ==0)
		{
			// 이륙 후, GOTO 등의 명령을 대기하고 있는 상태 

			  // path에 목적지 정보가 있으면 첫 번째 데이터를 읽어 와서 target position 갱신 

				if (!path.empty() )
				{


					target_position = path[0];
					target_position.target_seq_no++;
					
					path.erase(path.begin());

					phase = "GOTO";

					/*
					// 갱신된 target 정보 publish (응용 프로그램에 타겟 정보 전달)
					cur_target.x_lat = target_position.pos_local.x;
					cur_target.y_long = target_position.pos_local.y;
					cur_target.z_alt = target_position.pos_local.z;
	
					// 위치 이동을 위한 publish 
					target_pos_local.pose.position.x = target_position.pos_local.x;
					target_pos_local.pose.position.y = target_position.pos_local.y;
					target_pos_local.pose.position.z = target_position.pos_local.z;
					*/
				}
				else
				{
					//cout << "eDrone_control_node: main() - phase is READY & path is empty" << endl;

					// 갱신된 target 정보 publish (응용 프로그램에 타겟 정보 전달)
					cur_target.target_seq_no = target_position.target_seq_no;
					cur_target.ref_system = target_position.ref_system;
					cur_target.x_lat = target_position.pos_local.x;
					cur_target.y_long = target_position.pos_local.y;
					cur_target.z_alt = target_position.pos_local.z;
					cur_target.reached = target_position.reached;
					cur_target_pub.publish(cur_target);	

					// 현재 목적지 위치 publish (실제 위치 이동)	
					target_pos_local.pose.position.x = target_position.pos_local.x;
					target_pos_local.pose.position.y = target_position.pos_local.y;
					target_pos_local.pose.position.z = target_position.pos_local.z;
			
					pos_pub_local.publish (target_pos_local);
				}

		}
		else if (phase.compare ("GOTO") ==0)
		{
			// target position으로 현재 이동 중이면 위치 정보 publish 
			if (target_position.reached != true)
			{
			
				// 현재 목적지 정보 publish (응용 프로그램에 target 정보 제공)
				cur_target.target_seq_no = target_position.target_seq_no;
				cur_target.ref_system = target_position.ref_system;
				cur_target.is_global = target_position.is_global;
				cur_target.reached = target_position.reached;

				if (target_position.is_global == false)
				{
					cur_target.x_lat = target_position.pos_local.x;
					cur_target.y_long = target_position.pos_local.y;
					cur_target.z_alt = target_position.pos_local.z;
				}

				else
				{
					cur_target.x_lat  = target_position.pos_global.latitude;
					cur_target.y_long  = target_position.pos_global.longitude;
					cur_target.z_alt  = target_position.pos_global.altitude;
				}

				cur_target_pub.publish(cur_target);			

				// 현재 목적지 위치 publish (실제 위치 이동)	
				target_pos_local.pose.position.x = target_position.pos_local.x;
				target_pos_local.pose.position.y = target_position.pos_local.y;
				target_pos_local.pose.position.z = target_position.pos_local.z;
			
				pos_pub_local.publish (target_pos_local);
			}			

			else// target position에 도착했으면 path 검사
			{
			  // path에 목적지 정보가 있으면 첫 번째 데이터를 읽어 와서 target position 갱신 

				if (!path.empty() )
				{
					target_position = path[0];
					target_position.target_seq_no++;

					path.erase(path.begin());

					// 갱신된 target 정보 publish (응용 프로그램에 타겟 정보 전달)
					cur_target.x_lat = target_position.pos_local.x;
					cur_target.y_long = target_position.pos_local.y;
					cur_target.z_alt = target_position.pos_local.z;

					//cout <<" eDrone_control_node: main() - arrived at the target position. path is not empty() " << endl;
					//cout <<" target_seq_no: " << target_position.target_seq_no << endl;
					//sleep(5);
					// 위치 이동을 위한 publish 
					target_pos_local.pose.position.x = target_position.pos_local.x;
					target_pos_local.pose.position.y = target_position.pos_local.y;
					target_pos_local.pose.position.z = target_position.pos_local.z;
				}

			  // path가 비어 있으면 READY phase로 전이
				else
				{	
				       //cout <<" eDrone_control_node: main() - arrived at the target position. path is empty() " << endl;
					//cout <<" target_seq_no: " << target_position.target_seq_no << endl;
					phase = "READY";
					//sleep(5);					
				}
			}
		}

		ros::spinOnce();
	  	rate.sleep();
	
	}


/*
	while ( ros::ok() )
	{

        	// ref_system_conversion_test();

//		if (current_state.mode.compare("AUTO.RTL") == 0) // RTL 모드일 경우 바로 종료
//		{
//			break;
//		}

		// 현재 목적지 정보 publish
		
		// 1) Target 토픽 publish (응용 프로그램 측에 현재 진행 중인 target 위치 및 도착 여부 알림)

		// 2) 위치 이동을 위한 setpoint_position/local 또는 setpoint_raw/global 토픽 출판
		cur_phase.phase = phase;
		cur_phase_pub.publish (cur_phase);
				
	//	if (phase.compare ("IDLE") !=0)
	//	{
	//		printf("altitude: %lf, phase: %s\n", current_pos_local.pose.position.z, cur_phase.phase.c_str());
	//	}
		
		if (phase.compare ("TAKEOFF") == 0) // 현재 이륙 중인 경우 
		{
			//cout << "eDrone_control_node: publish Topic to takeoff - altitude " << takeoff_altitude <<  endl;	

			// change mode to offboard
			modeChange_cmd.request.base_mode = 0;
			modeChange_cmd.request.custom_mode = "OFFBOARD";
			if (modeChange_client.call(modeChange_cmd))
			{
				if (modeChange_cmd.response.mode_sent)
				{;
					//cout << "offboard enabled" << endl;
				}

			}
			
	
			// set topic field values
			target_pos_local.pose.position.x = 0;
			target_pos_local.pose.position.y = 0;	
			target_pos_local.pose.position.z = takeoff_altitude;	
			
			// publish topic
			
			pos_pub_local.publish (target_pos_local);
			ros::spinOnce();
			rate.sleep();

		}


		else if (autonomous_flight == true)
		//if (autonomous_flight == true && target_position.geofence_violation == false)
		{

			cur_phase.phase = phase;
			cur_phase_pub.publish (cur_phase);
			


			if (target_position.geofence_violation || target_position.noflyZone_violation) // geofence 또는 비행 금지 구역 위반 시 목표 지점으로 이동하지 않고 대기 
			{
				ros::spinOnce();
	      		  	rate.sleep();
				continue;
			}

			cur_target.target_seq_no = target_position.target_seq_no;
		
			cur_target.is_global = target_position.is_global;

			if (target_position.is_global == false)
			{
				cur_target.x_lat = target_position.pos_local.x;
				cur_target.y_long = target_position.pos_local.y;
				cur_target.z_alt = target_position.pos_local.z;
			}

			else
			{
				cur_target.x_lat  = target_position.pos_global.latitude;
				cur_target.y_long  = target_position.pos_global.longitude;
				cur_target.z_alt  = target_position.pos_global.altitude;
			}
		
			cur_target.reached = target_position.reached;


			cur_target_pub.publish(cur_target);

			// 현재 목적지로 이동

			if (target_position.reached == false)
			{
				if (target_position.is_global == false)
				{
				//	pos_pub_local.
					ROS_INFO("Moving to the target point (local coord.)");
				
			//	printf("target position: (%f, %f, %f)\n", target_position.pos_local.x,
		//		 target_position.pose_local.y, target_position.pos_local.z);
				
					printf("target position: (%f, %f, %f)\n", target_pos_local.pose.position.x,
				 target_pos_local.pose.position.y, target_pos_local.pose.position.z);
						//printf("phase: %s", phase.c_str());
			
					pos_pub_local.publish(target_pos_local); // 목적지 위치 정보 (지역 좌표) 출판 - type: geometry_msgs::poseStamped
				}

				else
				{
				//	pos_pub_global

					ROS_INFO("Moving to the target point (global coord)");
					//printf("target position: (%f, %f, %f)\n", target_position.pos_global.latitude, 
					target_position.pos_global.longitude, 	target_position.pos_global.altitude);
 					//mavros_msgs::GlobalPositionTarget target_pos_global; // 목적지 위치 정보 (전역 좌표)
				
					printf("target position: (%f, %f, %f)\n", target_pos_global.latitude, 
					target_pos_global.longitude, 	target_pos_global.altitude);

					pos_pub_global.publish(target_pos_global);// 목적지 위치 정보 (전역 좌표 출판) - type: mavros_msgs/GlobalPositionTarget
				}

				if (current_state.mode.compare("AUTO.RTL") !=0)
				{
					//ROS_INFO("state_cb(): FLIGHT MODE = RTL");
				
					modeChange_cmd.request.base_mode = 0;
					modeChange_cmd.request.custom_mode = "OFFBOARD";
					modeChange_client.call(modeChange_cmd);
				}
			}
		  	
			//ros::spinOnce();
      		  	//rate.sleep();

			//printf("home position: (%f, %f, %f) \n", home_position.position.x, home_position.position.y, home_position.position.z);	
	
		}// if (autonomous_flight)

			ros::spinOnce();
      		  	rate.sleep();

	}
*/
	return 0;
}

