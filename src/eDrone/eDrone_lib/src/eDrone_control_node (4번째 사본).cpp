

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

 // eDrone (ROS Topics, ROS Services, Data types, parameters)
#include <eDrone_lib/Vehicle.h>
#include <eDrone_lib/GeoInfo.h>
#include <eDrone_lib/GeoUtils.h>
#include <eDrone_lib/types.h>
#include <eDrone_lib/params.h>
#include <eDrone_msgs/Arming.h> // 시동 API 헤더 파일
#include <eDrone_msgs/Takeoff.h> // 이륙 API 헤더 파일
#include <eDrone_msgs/Landing.h> // 착륙 API 헤더 파일
#include <eDrone_msgs/Goto.h> // 무인기 위치 이동 API 헤더 파일 포함
#include <eDrone_msgs/GotoPath.h> // 경로 비행 API 헤더 파일 포함 
#include <eDrone_msgs/ModeChange.h> // 비행 모드 변경 API 헤더 파일
#include <eDrone_msgs/RTL.h> // RTL
#include <eDrone_msgs/GeofenceSet.h> // GeofenceSet API 헤더 파일
#include <eDrone_msgs/GeofenceCheck.h> // GeofenceCheck API 헤더 파일
#include <eDrone_msgs/GeofenceReset.h> // GeofenceReset API 헤더 파일
#include <eDrone_msgs/NoflyZoneSet.h> // 비행 금지 구역 설정
#include <eDrone_msgs/NoflyZoneReset.h> // 비행 금지 구역 해제
#include <eDrone_msgs/NoflyZoneCheck.h> // 비행 금지 구역 확인
#include <eDrone_msgs/NoflyZone.h> // (2018.11.19) 비행금지구역 data type  
#include <eDrone_msgs/NoflyZones.h> //	(2018.11.19) 다수 비행 금지구역 topoic msg type
#include <eDrone_msgs/SurveyArea.h> // 영역 탐색 
#include <eDrone_msgs/SurveyPath.h> // 경로 탐색 (경로 이동, 사진 촬영, +a) 
#include <eDrone_msgs/Orbit.h> // hotPoint API를 위한 선회 비행 

using namespace std;
using namespace mavros_msgs;
using namespace eDrone_msgs;

// test 함수 
void printMentalMap (Mental_Map* mental_map_ptr, const int AREA_WIDTH, const int AREA_HEIGHT);
void printAltPath (std::vector<Target_Position> altPath);
// 경로 계산 함수
std::vector<Target_Position> getAltPath(Target_Position src, Target_Position dst);
std::vector<Target_Position> getCoveragePath(vector<geometry_msgs::Point> points, double altitude, double interval);
void initCell (Cell* cell_ptr, int index_x, int index_y, const int CELL_WITDH, const int CELL_HEIGHT, const double base_x, const double base_y);
void updateMap(Mental_Map* mental_map_ptr, int curCell_x, int curCell_y);
bool isOccupiedCell (Cell* cell_ptr);
bool  (Cell* cell_ptr);

//// 목적지 변수
//Target_Position target_position = { .target_seq_no = 0 }; // (2018.10.02) 현재 목적지 정보 (published topic)
Target_Position target_position = { .target_seq_no = -1 }; // 현재 목적지 정보 (published topic)
int num_targets; // 목적지 개수 (goto service마다 1 씩 증가)
bool autonomous_flight = false; // 자율 비행 여부  (goto service가 호출되면 true로 변경됨)

float Geofence_Radius = 1000;
//enum Geofence_Policy {Warning, RTL, Loiter, Landing};
//enum Geofence_Policy geofence_policy = RTL;

//// 메시지 변수 선언

// (Subscribed topic data)
  // (무인기 상태 정보 수신 목적)
mavros_msgs::State current_state; // 무인기 상태 정보

  // (무인기 위치 확인 목적)
geometry_msgs::PoseStamped current_pos_local; // 현재 위치 및 자세 정보 (지역 좌표)
sensor_msgs::NavSatFix current_pos_global; // 현재 위치 정보 (전역 좌표)

  // (홈 위치 획득 목적)
mavros_msgs::HomePosition home_position; // home position

  // (비행금지구역 획득 목적)

NoflyZones nfZones;// 

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

// Orbit API (hotPoint) 관련
 std::vector<Target_Position> orbit_path; // 특정 위치를 기준으로 하는 선회 비행 경로 
 int orbit_req_cnt = 0; // (요청된) 선회 비행 횟수
 int orbit_cnt = 0; // 현재 선회 비행 횟수 

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
ros::Subscriber noflyZones_sub; 

// 서비스 서버 선언
ros::ServiceServer arming_srv_server;
ros::ServiceServer takeoff_srv_server;
ros::ServiceServer landing_srv_server;
ros::ServiceServer modeChange_srv_server;
ros::ServiceServer rtl_srv_server;
ros::ServiceServer goto_srv_server;
ros::ServiceServer gotoPath_srv_server;
ros::ServiceServer surveyArea_srv_server;
ros::ServiceServer orbit_srv_server;


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
// (다른 지점으로 이동 중) GOTO
// (선회 비행) ORBIT

string phase = "UNARMED"; 

// (2018.07.03) 이륙 목적

double takeoff_altitude = 0; 

// 경로 계산
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



////////////////////
bool  isOccupiedCell (Cell* cell_ptr) { return false;}

void initCell (Cell* cell_ptr, int index_x, int index_y, const int CELL_WIDTH, const int CELL_HEIGHT, const double base_x, const double base_y){
  cell_ptr->index_x = index_x;
  cell_ptr->index_y = index_y;
  cell_ptr->x = index_x * CELL_WIDTH + base_x;
  cell_ptr->y = index_y * CELL_HEIGHT + base_y;
  cell_ptr->visited = false;
  cell_ptr->occupied = false;
  cell_ptr->observed = false;
  cell_ptr->rel_altitude_real= 0; 
  cell_ptr->rel_altitude_estimated = 0;
  cell_ptr->includedInPath = false; // (2018.11.07) 

  cout << "index_x: " << cell_ptr->index_x <<endl;
  cout << "index_y: " << cell_ptr->index_y << endl;
  cout << " x: " << cell_ptr->x <<endl;
  cout << " y: " << cell_ptr->y << endl;

}

bool isNoflyZoneCell (Cell* cell_ptr) {return false;} // (2018.11.06) - 추후 구현 예정 


void printMentalMap (Mental_Map* mental_map_ptr, const int AREA_WIDTH, const int AREA_HEIGHT)
{
	cout<< "mental map" << endl;

	for (int j=AREA_HEIGHT-1; j > -1; j--)
	{
		for (int i = 0;  i < AREA_WIDTH; i++)
		{
			if (mental_map_ptr->grid[i][j].observed == true) {
				if (mental_map_ptr->grid[i][j].occupied == true)
				{
					cout << 'x' << ' ';
				}
				else 
				{
					cout << 'o' << ' ';
				}
			}
			else {
				cout << '-' << ' ';
			} 

		} // inner loop
		
		cout << std::endl;

	}

}

void printWavefrontMap (int** m, int area_width, int area_height)
//void printWavefrontMap (WavefrontMap* m )
{
  //
	cout << " printWavefrontMap(): " << endl;

	for (int index_y=area_height; index_y >= 0; index_y--)
	{
		for (int index_x = 0; index_x < area_width+1; index_x++)
		{
			cout << " " << m[index_x][index_y] << " ";
		}
		cout << endl;
	}
	sleep(10);
  
}

void printPoint( Point point)
{
    cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << endl;

}



void printAltPath (std::vector<Target_Position> altPath) // (2018.11.24) printPath로 이름 변경 요 
{
	//  대체 path 정보를 화면에 출력  
	cout << "eDrone_control_node: printAltPath() " << endl;
	int cnt = 0;

	 for (std::vector<Target_Position>::iterator it = altPath.begin() ; it != altPath.end(); ++it)
	{
		cnt++;
		Target_Position target = *it;


		if (target.ref_system == "WGS84")
		{
			cout << "Target#" << cnt << ": lat: " << target.pos_global.latitude << ", lon: " << target.pos_global.longitude << endl;
		}
		else if (target.ref_system == "ENU")
		{
			cout << "Target#" << cnt << ": x: " << target.pos_local.x << ", y: " << target.pos_local.y << endl;

		}
		
	}


}


// 주어진 영역 탐색에 필요한 coverage path 계산
std::vector<Target_Position> getCoveragePath(vector<eDrone_msgs::Target> points, double altitude, double interval)
{
	// ENU 좌표계 가정 - 함수 호출 위치에서 좌표 변환 필요 (WGS84->ENU)

	std::vector<Target_Position> path;

	ROS_INFO ("eDrone_control_node: getCoveragePath() was called");

	// Mental Map 범위 계산	
	cout << "set mental map range" << endl;

	double x_min = -1, y_min=-1;
	double x_max = -1, y_max= -1;
	
	double distance_min = -1;
	double distance_max = -1;

	Target entry_point;
	Target exit_point;	

	for (vector<Target>::iterator it = points.begin(); it != points.end(); it++)
	{
		Target point = *it;
		
		if ( x_min < 0 || point.x_lat < x_min)
		{
			x_min = point.x_lat;
		}
		if ( y_min < 0 || point.y_long < y_min)
		{
			y_min = point.y_long;
		}

		if ( x_max < 0 || point.x_lat > x_max)
		{
			x_max = point.x_lat;
		}
		if ( y_max < 0 || point.y_long > y_max)
		{
			y_max = point.y_long;
		}

		double distance = sqrt ( pow ( (double) current_pos_local.pose.position.x - (double)  point.x_lat, (double) 2) + pow ( (double) current_pos_local.pose.position.y - (double) point.y_long , (double) 2) );

		if ( distance_min < 0 || distance < distance_min)
		{
			distance_min = distance;
			entry_point = point;
		}
		if ( distance_max < 0 || distance > distance_max)
		{
			distance_max = distance;
			exit_point = point;
		}

	} // 경계점들을 포함하는 사각형 영역 계산, entry/exit point 계산 

	cout << "entry point: (" << entry_point.x_lat << ", " << entry_point.y_long << ")" << endl;
	cout << "exit point: (" << exit_point.x_lat << ", " << exit_point.y_long <<  ")" << endl;

	// Mental Map & Grid 생성 

	Mental_Map mental_map;

	Mental_Map* mental_map_ptr = &mental_map;

	int CELL_WIDTH = interval;
	int CELL_HEIGHT = interval;
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

	// CELL 배열 동적 할당 

	for (int c = 0; c < AREA_WIDTH+1; c++)
	{
		mental_map.grid[c] = new Cell[AREA_HEIGHT+1];
	}

	cout << "Dynamic allocation" << endl;

	// Cell 정보 초기화 
	for (int i = 0; i < AREA_WIDTH+1; i++)
	{
		for(int j = 0; j < AREA_HEIGHT+1;j++)
		{
		//	cout << " i, j = " << i << ", " << j << endl;			
			Cell* cell_ptr = &(mental_map.grid[i][j]);
			initCell(cell_ptr, i, j, CELL_WIDTH, CELL_HEIGHT, x_min, y_min);
		}
	}

	// src CELL, dst CELL 계산

	Cell* src_cell_ptr= NULL; 
	Cell* dst_cell_ptr = NULL;

	int src_cell_index_x = (entry_point.x_lat - x_min) / CELL_WIDTH;
	int src_cell_index_y = (entry_point.y_long - y_min) / CELL_HEIGHT;
	src_cell_ptr = &(mental_map.grid[src_cell_index_x][src_cell_index_y]);
	
	cout << " (src cell) index_x:" << src_cell_ptr-> index_x << ", index_y: " << src_cell_ptr-> index_y << "x: " << src_cell_ptr->x << "y: " << src_cell_ptr->y << endl;
	
	int dst_cell_index_x = (exit_point.x_lat - x_min) / CELL_WIDTH;
	int dst_cell_index_y = (exit_point.y_long - y_min) / CELL_HEIGHT;
	dst_cell_ptr = &(mental_map.grid[dst_cell_index_x][dst_cell_index_y]);
	cout << " (dst cell) index_x:" << dst_cell_ptr-> index_x << ", index_y: " << dst_cell_ptr-> index_y << "x: " << dst_cell_ptr->x << "y: " << dst_cell_ptr->y << endl;

	/* Wavefront Map 생성 */

	// label - 1: 장애물, 2: 비행금지구역, 3: 방문된 cell, 4: Goal cell	 

	//int waveFrontMap[AREA_WIDTH+1][AREA_HEIGHT+1]={0};


	int ** waveFrontMap = new int*[AREA_WIDTH+1];

	for (int c = 0; c < AREA_WIDTH+1; c++)
	{
		waveFrontMap[c] = new int[AREA_HEIGHT+1];
	}

	for (int x_index = 0; x_index < AREA_WIDTH+1; x_index++)
	{
		for(int y_index = 0; y_index < AREA_HEIGHT+1; y_index++)
		{
			waveFrontMap[x_index][y_index] = 0;
		}
	} 
	

	vector <Cell*> queue;  // BFS에 필요한 queue 선언
	
	// g에서부터 시작
	
	queue.push_back (dst_cell_ptr);
	 int index_x = dst_cell_ptr->index_x;
  	int index_y = dst_cell_ptr->index_y;
  	waveFrontMap[index_x][index_y] = 4;
	  
	bool pathExist = false; // whether a path from src to dst exists

	while (queue.empty() != true)
	{
		Cell* nextCell = queue.front(); // queue에서 다음 cell 제거, wavefront map 확장 
		if (nextCell == src_cell_ptr) { // wavefront map이 현재 cell까지 생성된 경우 map 생성 중단
			break;
		}
		queue.erase (queue.begin());
		int index_x = nextCell->index_x; 
		int index_y = nextCell->index_y;
		
   		// 다음 cell의 이웃 cell(8개) 들을 queue에 추가
   		for (int local_index_x = -1; local_index_x < 2; local_index_x++)
   		{
			// x 인덱스 범위가 mental_map 범위를 벗어 나면 continue
			if((index_x + local_index_x)<0 || (index_x+local_index_x ) > (AREA_WIDTH) ) {
				continue;
       			}

			for (int local_index_y= -1; local_index_y < 2; local_index_y++)
			{
				// y 인덱스의 범위가 mental_map 범위를 벗어 나면 continue
	 			if ( ((index_y+local_index_y) < 0 ) || ((index_y+local_index_y) > (AREA_HEIGHT) )  )
				{
					continue;
				}

				 int index_x_neighbor = index_x + local_index_x;
				 int index_y_neighbor = index_y + local_index_y;
	 			 Cell* neighbor = &(mental_map_ptr-> grid[index_x_neighbor][index_y_neighbor]); // 현재 탐색 중인 Cell의 인접 cell 
				
				 if ( waveFrontMap[index_x_neighbor][index_y_neighbor] > 0 ) // 이미 label이 설정된 cell은 중복하여 설정하지 않음
				 {
	 				continue;
	  			 }
				
				// Case#1 이미 방문된 Cell인 경우
				if ( neighbor->visited == true )  {

					waveFrontMap[index_x_neighbor][index_y_neighbor] = 3;
					queue.push_back(neighbor);

					if (neighbor == src_cell_ptr) {// queue에 추가된 노드가 starting node이면
						pathExist = true; // s->g까지 경로 존재 여부를 true로 설정  
					}

				}
				// Case#2 장애물이 있는 Cell인 경우, 높이를 기준으로 선택적으로 queue에 추가 
				  // (2018.11.06) 현재는 모든 경우에 우회하도록 구현 

				else if ( isOccupiedCell(neighbor) == true)
				{					
					waveFrontMap[index_x_neighbor][index_y_neighbor] = 1; 						
				}
	
				// Case#3 비행금지구역에 속한 Cell인 경우
                                else if ( (neighbor) == true)
				{
					waveFrontMap[index_x_neighbor][index_y_neighbor] = 2;					
				}
			
				// Case#4 그 밖의 경우 
				else
				{
					waveFrontMap[index_x_neighbor][index_y_neighbor] = waveFrontMap[index_x][index_y] +1; //(2018.10.02)
					queue.push_back ( neighbor );
					if (neighbor == src_cell_ptr) {// queue에 추가된 노드가 starting node이면  
						pathExist = true; // s->g까지 경로 존재 여부를 true로 설정  
					}
				}

			}
			
		}
	} //  wavefront map 생성

	 printWavefrontMap(waveFrontMap, AREA_WIDTH, AREA_HEIGHT);

	/* Coverage Path 계산 */ 

	// path 존재 여부 확인

	if (pathExist != true) // src-dst path가 존재하지 않으면 빈 path 반환
	{
		return path;
	}
	// src cell에서 시작

	cout << "path exist - " << endl;

	bool pathComputed = false;
	src_cell_ptr->includedInPath = true;
	Cell* cur_cell_ptr = src_cell_ptr; // 경로 계산에 필요한 임시 cell 포인터
	Cell* neighbor_cell_ptr = NULL; // 현재 검사 중인 cell의 이웃 cell을 가리키는 포인터 
	
	int cnt = 0;
	// (이웃 cell 선택) 반복

	while (cur_cell_ptr != dst_cell_ptr)
	{
		if (cnt > 1000) {break;} else {cnt++;}

		if (pathComputed == true) {break;}
	
		int max_label = -1;
		Cell* neighbor_max_label = NULL;

		int index_x = cur_cell_ptr->index_x;
		int index_y = cur_cell_ptr->index_y;

		for (int local_index_x = -1; local_index_x < 2; local_index_x++ )
		{
			int index_x_neighbor = index_x + local_index_x;

			if ( index_x_neighbor < 0 || index_x_neighbor > AREA_WIDTH) {continue;}

			for (int local_index_y = -1; local_index_y < 2; local_index_y++)
			{
				int index_y_neighbor = index_y + local_index_y;

				if ( index_y_neighbor < 0 || index_y_neighbor > AREA_HEIGHT) {continue;}
			
				if (local_index_x ==0 && local_index_y ==0) { continue; } // 자기 자신은 탐색 대상에서 제외 			

				neighbor_cell_ptr = &(mental_map_ptr->grid[index_x_neighbor][index_y_neighbor]); 

				if (neighbor_cell_ptr->includedInPath == true) { continue;} // 이미 경로에 추가된 cell은 제외 
				if (waveFrontMap[index_x_neighbor][index_y_neighbor] == 1) { continue; } // 장애물이 있는 cell은 제외
				else if (waveFrontMap[index_x_neighbor][index_y_neighbor] == 2) { continue; } // 비행금지구역에 속한 cell은 제외
				else if (waveFrontMap[index_x_neighbor][index_y_neighbor] == 3) { continue; }// 기 방문된 cell은 제외 

				if ( &mental_map_ptr->grid[index_x_neighbor][index_y_neighbor] == dst_cell_ptr) { // 이웃 cell이 목적지 cell인 경우
					max_label = waveFrontMap[index_x_neighbor][index_y_neighbor]; 
					neighbor_max_label =  &mental_map_ptr->grid[index_x_neighbor][index_y_neighbor];
					pathComputed = true;
					break;	
				}			

				if ( max_label < 0 || ( waveFrontMap[index_x_neighbor][index_y_neighbor] > max_label) )// 이웃 cell 들 중 label 값이 가장 큰 cell 탐색				
				{
						max_label = waveFrontMap[index_x_neighbor][index_y_neighbor];
						neighbor_max_label = &mental_map_ptr->grid[index_x_neighbor][index_y_neighbor];
				}				
								

			}

			if (pathComputed == true) { break; }

		} // 현재 노드의 이웃 노드 검사 - 다음 노드 선택
		cout << "neighbor_max_label: (" <<  neighbor_max_label->x << "," << neighbor_max_label->x << ")" << endl;
		if (neighbor_max_label != NULL) {
			cur_cell_ptr = neighbor_max_label;
		
			// 고도값 설정
			if (cur_cell_ptr->occupied == true)
			{
				cur_cell_ptr-> z = cur_cell_ptr->obstacle_height + altitude;
			}
			else
			{
				cur_cell_ptr-> z = altitude;
			}
	
			Target_Position target_position;
			target_position.ref_system = "ENU";
		
			target_position.pos_local.x = cur_cell_ptr->x;
			target_position.pos_local.y = cur_cell_ptr->y;
			target_position.pos_local.z = cur_cell_ptr->z;

			path.push_back(target_position); // path에 waypoint 추가
			cur_cell_ptr->includedInPath = true;
		}

	} 	

	// coverage path 반환 

 	return path;
}
//std::vector<Target_Position> getCoveragePath(vector<geometry_msgs::Point> points, double altitude, double interval)


/*
void initCell (Cell* cell_ptr, int index_x, int index_y, const int CELL_WIDTH, const int CELL_HEIGHT, const double base_x, const double base_y){
  cell_ptr->index_x = index_x;
  cell_ptr->index_y = index_y;
  cell_ptr->x = index_x * CELL_WIDTH + base_x;
  cell_ptr->y = index_y * CELL_HEIGHT + base_y;
  cell_ptr->visited = false;
  cell_ptr->occupied = false;
  cell_ptr->observed = false;
  cell_ptr->rel_altitude_real= 0; 
  cell_ptr->rel_altitude_estimated = 0;
}*/


void updateMap(Mental_Map* mental_map_ptr, int curCell_x, int curCell_y)
{
ROS_INFO("updateMap was called");

 cout << " Current Cell index: " << curCell_x << ", " << curCell_y << endl; 
 // 현재 위치 확인 

  const int AREA_WIDTH = mental_map_ptr->area_width;
  const int AREA_HEIGHT = mental_map_ptr->area_height;

 /* */


 // 현재 cell의 지표면 높이 측정& 기록

 // double rel_altitude_real = checkPosition_cmd.response.altitude - checkPosition_cmd.response.z; // 



// mental_map_ptr->grid[curCell_x][curCell_y].rel_altitude_real = rel_altitude_real; 
  // ROS_INFO ("updateMap(): relative altitude is %lf meters",  rel_altitude_real); 

 int SENSING_RANGE= 0 ; 

if ( AREA_WIDTH > AREA_HEIGHT) SENSING_RANGE = AREA_WIDTH;
else SENSING_RANGE = AREA_HEIGHT;

 // 각 방향에 대해 가장 가까운 장애물 위치를 확인하고 mental_map 확장
 for (int local_index_x = (-1) * SENSING_RANGE; local_index_x <SENSING_RANGE; local_index_x++ ) // 현재 cell을 기준으로 8개 방향의 장애물 확인 & mental_map  확장 

// for (int local_index_x = (-1) * SENSING_RANGE; local_index_x <SENSING_RANGE; local_index_x++ ) // 현재 cell을 기준으로 8개 방향의 장애물 확인 & mental_map  확장 
{ 

	int index_x = curCell_x + local_index_x;

	if (index_x <0  || index_x > AREA_WIDTH-1) // x,y 인덱스가 mental map 범위를 벗어나면 continue 
	{
		continue;
	}

 	for (int local_index_y = (-1) * SENSING_RANGE; local_index_y < SENSING_RANGE; local_index_y++  )
	{
		int index_y = curCell_y + local_index_y; 
		
		if (index_y < 0 || index_y > AREA_HEIGHT-1)
		{
			continue;
		} 

		// 현재는 모든 cell이 free cell이라고 가정, 
		// 필요 시 센싱 범위 내 CELL에 장애물이 있는지 확인하는 함수 구현할것 

		Cell* observedCell = &(mental_map_ptr->grid[index_x][index_y]);
	
		if (observedCell->observed == false) // 새롭게 발견된 cell인 경우
		{
			// 센싱 범위 내 CELL들이 기존에 free cell 또는 occupied cell 목록에 
			// 추가되지 않은 경우, 새롭게 추가

			bool isOccupied = isOccupiedCell (observedCell); // 하늘을 나는 무인기의 특성 상, sensing 범위 내의 모든 cell들에 대해 장애물이 있는지 여부를 판단할 수 있다고 가정함 

			// 1) free cell인 경우

			if (isOccupied == false)
			{
				observedCell->occupied = false;
				mental_map_ptr->freeCells.push_back(observedCell);
			}
		
			// 2) occupied cell인 경우 
			// occupiedCells.push_back(else
			else{
				observedCell->occupied = true;
				mental_map_ptr->occupiedCells.push_back(observedCell);
			}
		
			observedCell->observed = true; // 		
		}	
	

		if (observedCell->occupied == true)
		{
			cout << "observedCell: " << endl;
			cout << " (index_x, index_y): (" << index_x << ", " << index_y << ")" << endl;
			cout << "  (index_x, index_y): (" << observedCell->index_x << ", " << observedCell->index_y << ")" << endl;
			cout << " visited: " << observedCell->visited << endl;
			cout << " occupied: " << observedCell->occupied << endl;
			cout << " observed: " << observedCell->observed << endl;
		}
				
	}

 } // 장애물 확인, mental_map 확장 
 


}

// src-dst 사이의 비행금지구역 우회 경로 계산 함수  
std::vector<Target_Position> getAltPath(Target_Position src, Target_Position dst)
{
	cout << " eDrone_control_node: getAltPath()" << endl;
	cout << "src: (" << src.pos_global.longitude << " , " << src.pos_global.latitude << ") " <<endl;
  	cout << "dst: (" << dst.pos_global.longitude << " , " << dst.pos_global.latitude << ") " <<endl;
	
	std::vector<Target_Position> path;
//	noflyZoneCheck_cmd.request.NFCheck_ref_system = "WGS84";

	// 비행금지구역의 경계에 해당하는 꼭지점 (4개) 구하기 
/*
	GeoPoint rect[4];

	noflyZoneCheck_cmd.request.NFCheck_src.x_lat = src.pos_global.latitude;
	noflyZoneCheck_cmd.request.NFCheck_src.y_long = src.pos_global.longitude;
	noflyZoneCheck_cmd.request.NFCheck_dst.x_lat = src.pos_global.latitude;
	noflyZoneCheck_cmd.request.NFCheck_dst.y_long = src.pos_global.longitude;

	// 비행금지구역 정보 획득 

	if (noflyZoneCheck_client.call(noflyZoneCheck_cmd) == true)
	{
		// 비행금지구역 꼭지점 정보 구하기
		rect[0].latitude = noflyZoneCheck_cmd.response.pt1.x_lat-0.001;
		rect[0].longitude = noflyZoneCheck_cmd.response.pt1.y_long-0.001;
		
		rect[1].latitude = noflyZoneCheck_cmd.response.pt1.x_lat-0.001;
		rect[1].longitude = noflyZoneCheck_cmd.response.pt2.y_long+0.001;
		
		rect[2].latitude = noflyZoneCheck_cmd.response.pt2.x_lat+0.001;
		rect[2].longitude = noflyZoneCheck_cmd.response.pt2.y_long+0.001;

		rect[3].latitude = noflyZoneCheck_cmd.response.pt2.x_lat+0.001;
		rect[3].longitude = noflyZoneCheck_cmd.response.pt1.y_long-0.001;


	
		if (noflyZoneCheck_cmd.response.result.compare ("PATH_OVERLAP_NF_ZONE" ) !=0)
		{
			ROS_INFO("eDrone_control_node: getAltPath(): no need to compute an alternate path");
		}
	}

	

	// src-dst 간 우회 경로 계산

	// SRC 다음에 이동할 Target (Target#1) 계산

	cout << "Target#1 계산 " << endl; 
	Target_Position target1, target2, target3;

	double distToDst = -1;

	int rectPointIdx = -1;

	for (int i = 0; i < 4; i++)
	{
		
		noflyZoneCheck_cmd.request.NFCheck_src.x_lat = src.pos_global.latitude;
		noflyZoneCheck_cmd.request.NFCheck_src.y_long = src.pos_global.longitude;

		noflyZoneCheck_cmd.request.NFCheck_dst.x_lat = rect[i].latitude;
		noflyZoneCheck_cmd.request.NFCheck_dst.y_long = rect[i].longitude;

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
	noflyZoneCheck_cmd.request.NFCheck_src.x_lat = target1.pos_global.latitude;
	noflyZoneCheck_cmd.request.NFCheck_src.y_long = target1.pos_global.longitude;
	noflyZoneCheck_cmd.request.NFCheck_dst.x_lat = dst.pos_global.latitude;
	noflyZoneCheck_cmd.request.NFCheck_dst.y_long = dst.pos_global.longitude;

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

	noflyZoneCheck_cmd.request.NFCheck_src.x_lat = target2.pos_global.latitude;
	noflyZoneCheck_cmd.request.NFCheck_src.y_long = target2.pos_global.longitude;
	noflyZoneCheck_cmd.request.NFCheck_dst.x_lat = dst.pos_global.latitude;
	noflyZoneCheck_cmd.request.NFCheck_dst.y_long = dst.pos_global.longitude;

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


	// 만약 Target#1과 Target#2가 같으면 우회 경로는 <Target#1, DST >
	// 다르면 우회 경로는 <Target#1, Target#2, DST> 가 됨 
*/
	return path; 
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


		if (current_pos_local.pose.position.z >= takeoff_altitude-0.1 )
		{
			cout << "eDrone_control_node: TAKEOFF completed!" << endl;

			phase = "READY"; 
			cur_phase.phase = phase;
		}
	
		return;
	}

	if ( (phase.compare("GOTO")==0) || (phase.compare("ORBIT")==0) )

//	if (phase.compare("GOTO")==0)
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

void noflyZones_cb(const eDrone_msgs::NoflyZones::ConstPtr& msg)
{
        // 현재 목적지 도달 여부 확인
       // ROS_INFO("eDrone_control_node: noflyZones_cb(): \n");

        nfZones = *msg;
/*
        int cnt = 0;
        for ( vector<NoflyZone>::iterator it = nfZones.noflyZones.begin(); it != nfZones.noflyZones.end(); it++)
        {
                NoflyZone nfZone = *it;
                cout << "NoflyZone#" <<cnt << endl;

                for (vector<Target>::iterator it = nfZone.noflyZone_pts.begin(); it!=nfZone.noflyZone_pts.end(); it++)
                {
                        Target point = *it;
                        cout << "point#" << cnt << "(" << point.x_lat <<", " << point.y_long << ", " << point.z_alt << ")" << endl;
                }

		if (cnt++ >5) {break;}
        }*/
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

	takeoff_altitude = req.takeoff_altitude; // 이륙 고도 저장 
	//double offset = 25;

	ROS_INFO("***Takeoff request received\n");
	// 서비스 요청 메시지 필드 선언

	takeoff_cmd.request.altitude = 2.5;
	

	ROS_INFO(" HOME_ALT: %lf, req.takeoff_altitude: %lf", HOME_ALT, req.takeoff_altitude);
  	takeoff_cmd.request.latitude = HOME_LAT; // 자동으로 home position 값을 얻어 와서 설정되도록 변경 필요
  	takeoff_cmd.request.longitude = HOME_LON;
 //	takeoff_cmd.request.yaw = 0;
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
	
	std::cout << "srv_modeChange_cb(): change the mode to " << req.modeChange_mode << endl; 

	modeChange_cmd.request.base_mode = 0;
        
	modeChange_cmd.request.custom_mode.assign(req.modeChange_mode);

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
	cout<< "req.goto_point.x_lat: " << req.goto_point.x_lat << ", req.goto_point.y_long: " << req.goto_point.y_long << endl;
	//sleep(5); // (10.02) test 

	/* goto 서비스 처리 절차 */

	// 좌표계 종류 확인 (ex. WGS84, ENU) & 목적지 좌표 (지역, 전역) 저장 
	if (req.goto_ref_system.compare("WGS84")==0) // 전역 좌표인 경우
	{
		target_position.pos_global.latitude = req.goto_point.x_lat;
		target_position.pos_global.longitude = req.goto_point.y_long;
		target_position.pos_global.altitude = req.goto_point.z_alt;


		// ENU로 좌표변환
		Point point = convertGeoToENU(req.goto_point.x_lat, req.goto_point.y_long, req.goto_point.z_alt, HOME_LAT, HOME_LON, HOME_ALT );

		target_position.pos_local.x = point.x;
		target_position.pos_local.y = point.y;
		target_position.pos_local.z = point.z;

	}

	else if (req.goto_ref_system.compare("ENU")==0)
	{
		target_position.pos_local.x = req.goto_point.x_lat;	
		target_position.pos_local.y = req.goto_point.y_long;
		target_position.pos_local.z = req.goto_point.z_alt;

		// WGS84로 좌표 변환
		GeoPoint geoPoint = convertENUToGeo(req.goto_point.x_lat, req.goto_point.y_long, req.goto_point.z_alt, HOME_LAT, HOME_LON, HOME_ALT );
		cout <<"converENUToGeo() was called" << endl;

		target_position.pos_global.latitude = geoPoint.latitude;	
		target_position.pos_global.longitude = geoPoint.longitude;		
		target_position.pos_global.altitude = geoPoint.altitude;		

	}

	// Geofence 검사

		geofenceCheck_cmd.request.geofence_ref_system = "WGS84";	
		geofenceCheck_cmd.request.geofence_arg1= target_position.pos_global.latitude;
        	geofenceCheck_cmd.request.geofence_arg2= target_position.pos_global.longitude;
		geofenceCheck_cmd.request.geofence_arg3= target_position.pos_global.altitude;

	//	geofenceCheck_cmd.request.geofence_arg1=  req.x_lat;
        //	geofenceCheck_cmd.request.geofence_arg2= req.y_long;
	//	geofenceCheck_cmd.request.geofence_arg3= req.z_alt;
		
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

		
		ROS_INFO("eDrone_control_node: noflyZoneCheck");
	
		Target src;
		src.ref_system = "ENU";
		src.x_lat = current_pos_local.pose.position.x;
		src.y_long =  current_pos_local.pose.position.y;
		src.z_alt =  current_pos_local.pose.position.z;

		Target dest;
		dest.ref_system = "ENU";
		dest.x_lat = target_position.pos_local.x;
		dest.y_long =  target_position.pos_local.x;
		dest.z_alt =  target_position.pos_local.z;


		for (vector<NoflyZone>::iterator it = nfZones.noflyZones.begin(); 
			it != nfZones.noflyZones.end();
			it++ )
		{
			NoflyZone nfz = *it;
		//	vector<Target> boundary_pts;

		  	for (vector<Target>::iterator it = nfz.noflyZone_pts.begin();
	                it !=  nfz.noflyZone_pts.end(); it++)
			{
				Target boundary_point = *it;
				cout << "boundary point: (" << boundary_point.x_lat << " , " << boundary_point.y_long << ", " << boundary_point.z_alt << ") " <<endl;
		//		boundary_pts.push_back(boundary_point);
			}			

		//	nfz.noflyZone_pts = boundary_pts;
		//	noflyZoneCheck_cmd.request.noflyZoneCheck_noflyZone = nfz;
		//	noflyZoneCheck_cmd.request.noflyZoneCheck_noflyZone.noflyZone_pts = nfz.noflyZone_pts;
		/*	
		Target src;
		src.ref_system = "ENU";
		src.x_lat = current_pos_local.pose.position.x;
		src.y_long =  current_pos_local.pose.position.y;
		src.z_alt =  current_pos_local.pose.position.z;
		*/
			noflyZoneCheck_cmd.request.noflyZoneCheck_src =src;
			noflyZoneCheck_cmd.request.noflyZoneCheck_dest.ref_system = req.goto_ref_system;
			noflyZoneCheck_cmd.request.noflyZoneCheck_dest.x_lat = req.goto_point.x_lat;
			noflyZoneCheck_cmd.request.noflyZoneCheck_dest.y_long = req.goto_point.y_long;
			noflyZoneCheck_cmd.request.noflyZoneCheck_dest.z_alt = req.goto_point.z_alt;
		
			noflyZoneCheck_cmd.request.noflyZoneCheck_dest = req.goto_point;  
			if ( noflyZoneCheck_client.call(noflyZoneCheck_cmd))
			{
				cout << " noflyZoneCheck API was called " << endl;
			}

			ROS_INFO("noflyZoneCheck result: %s ", noflyZoneCheck_cmd.response.result.c_str() ); 
		} 
	
		if (noflyZoneCheck_cmd.response.result == "DST_IN_NF")
		{
			res.value = false; // 목적지가 비행금지구역 내에 있으면 goto 명령 거부 
			return true; //
		}

		else if (noflyZoneCheck_cmd.response.result ==  "PATH_OVERLAP")
		{
			vector<Target_Position> indirectPath = getIndirectPath(src, dest);
		}

		
	//	noflyZoneCheck_cmd.request.NFCheck_ref_system = "WGS84";
/*		
		noflyZoneCheck_cmd.request.NFCheck_src.x_lat = current_pos_global.latitude;
 		noflyZoneCheck_cmd.request.NFCheck_src.y_long = current_pos_global.longitude;
		noflyZoneCheck_cmd.request.NFCheck_src.z_alt = current_pos_global.altitude;
		noflyZoneCheck_cmd.request.NFCheck_dst.x_lat = target_position.pos_global.latitude;
		noflyZoneCheck_cmd.request.NFCheck_dst.y_long = target_position.pos_global.longitude;
		noflyZoneCheck_cmd.request.NFCheck_dst.z_alt = target_position.pos_global.altitude;

	//	noflyZoneCheck_cmd.request.dst_arg1 = req.x_lat;
	//	noflyZoneCheck_cmd.request.dst_arg2 = req.y_long;
	//	noflyZoneCheck_cmd.request.dst_arg3 = req.z_alt;

		cout<< "_req.goto_point.x_lat: " << req.goto_point.x_lat << ", req.goto_point.y_long: " << req.goto_point.y_long << endl;	

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

		else
		{
			ROS_INFO(" noflyZoneCheck API call failed !");
			sleep(5);
		}
*/	
	return true;
}	

bool srv_gotoPath_cb(eDrone_msgs::GotoPath::Request &req, eDrone_msgs::GotoPath::Response &res)
{	

	ROS_INFO("eDrone_control_node: GotoPath request received\n");
	//phase = "GOTOPATH";
	
	// 좌표계 종류 확인 (ex. WGS84, ENU) & 목적지 좌표 (지역, 전역) 저장 
	if (req.gotoPath_ref_system.compare("WGS84")==0) // 전역 좌표인 경우
	{
		for (vector<Target>::iterator it = req.gotoPath_pts.begin(); it != req.gotoPath_pts.end(); it++)
		//for (vector<GeoPoint>::iterator it = req.path_pts_global.begin(); it != req.path_pts_global.end(); it++)
		{
			Target target = *it;
			//GeoPoint geoPoint = *it;
			
			target_position.pos_global.latitude = target.x_lat;
			target_position.pos_global.longitude = target.y_long;
			target_position.pos_global.altitude = target.z_alt;

			// ENU로 좌표변환
			Point point = convertGeoToENU(target.x_lat, target.y_long, target.z_alt, HOME_LAT, HOME_LON, HOME_ALT );
			target_position.ref_system = "ENU";
			target_position.pos_local.x = point.x;
			target_position.pos_local.y = point.y;
			target_position.pos_local.z = point.z;

			// Geofence 검사

			// NoflyZone 검사 	

			
			target_position.reached = false;

			// path에 목적지 또는 부분 경로 추가 		
			path.push_back (target_position); 
		}
	}
	else if (req.gotoPath_ref_system.compare("ENU") == 0) // 지역 좌표
	//else if (req.path_ref_system.compare("ENU") == 0) // 지역 좌표
	{
		for (vector<Target>::iterator it = req.gotoPath_pts.begin(); it != req.gotoPath_pts.end(); it++)
		{
			Target target = *it;
			//Point point = *it;
			target_position.ref_system = "ENU";
			target_position.pos_local.x = target.x_lat;
			target_position.pos_local.y = target.y_long;
			target_position.pos_local.z = target.z_alt;

			// WGS84로 좌표변환
			GeoPoint geoPoint = convertENUToGeo(target.x_lat, target.y_long, target.z_alt, HOME_LAT, HOME_LON, HOME_ALT );

			target_position.pos_global.latitude = geoPoint.latitude;
			target_position.pos_global.longitude = geoPoint.longitude;
			target_position.pos_global.altitude = geoPoint.latitude;

			// Geofence 검사

			// NoflyZone 검사 	

			target_position.reached = false;
			// path에 목적지 또는 부분 경로 추가 		
			path.push_back (target_position); 

		}
		


	}
	// Geofence 검사

	// NoflyZone 검사 

	// path 구성

	return true;
}
	

bool srv_surveyArea_cb(eDrone_msgs::SurveyArea::Request &req, eDrone_msgs::SurveyArea::Response &res)
{
	bool result = false;	

	ROS_INFO ("eDrone_control_node: surveyArea service was called");

	// 경로 계산 

	vector<Target_Position> coveragePath;

	if (req.surveyArea_ref_system == "ENU")
	{	
		coveragePath = getCoveragePath(req.surveyArea_pts, req.surveyArea_altitude, req.surveyArea_interval);	
	}
	else if (req.surveyArea_ref_system == "WGS84")
	{

		// ENU 좌표로 변환

		vector<Target> surveyArea_pts;

		for (vector<Target>::iterator it = req.surveyArea_pts.begin(); it != req.surveyArea_pts.end(); it++)
		{
			Target target = *it;

			// ENU로 좌표변환
			Point point = convertGeoToENU(target.x_lat, target.y_long, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );
			target.x_lat = point.x;
			target.y_long = point.y;	
			target.ref_system = "ENU";		
			surveyArea_pts.push_back(target);

		}

		coveragePath = getCoveragePath(surveyArea_pts, req.surveyArea_altitude, req.surveyArea_interval);	

	}
	
	printAltPath(coveragePath);	

	for (vector<Target_Position>::iterator it = coveragePath.begin(); it != coveragePath.end(); it++ )
	{
		Target_Position target_position = *it;	
		target_position.reached = false;	
		path.push_back(target_position);
	} // path에 coveragePath 추가 

		

	


	//path.insert( path.end(), coveragePath.begin(), coveragePath.end() );


	// vector<Points> type -> vector<Target_Position> type conversion is needed
	
//	path.insert( path.end(), coveragePath.begin(), coveragePath.end() );

	result = true;
	return result;
}

bool srv_orbit_cb(eDrone_msgs::Orbit::Request &req, eDrone_msgs::Orbit::Response &res)
{
	ROS_INFO ("eDrone_control_node: Orbit service was called");

	bool result = false;

	if ( phase.compare ( "READY") != 0  ) // READY 상태에서만 선회 비행 가능 
	{
		
		return result;
	}
	
	phase = "ORBIT"; // phase 변경 
	cur_phase.phase = phase;
	// 선회 비행 시작 지점으로 이동하기 위해 target_position 설정 
	if (req.orbit_ref_system == "ENU")
	{
		orbit_req_cnt = req.orbit_req_cnt; // 요청된 선회비행횟수 저장

	
		// #1. (기준점과 r로 결정되는 원)과 (현 위치와 기준점을 잇는 직선) 사이의 교점 (2개)을 구하고 그 중 현 위치에 더 가까운 지점을 path에 추가 (선회 비행 시작점)

		// #1-1 교점을 구하기 위한 이차방정식 계산
		
		Point cur_position; // 현재 위치
		cur_position.x = current_pos_local.pose.position.x;
		cur_position.y = current_pos_local.pose.position.y;
		
		// 직선의 방정식:  y = Ax + B
		// 직선의 기울기: inclination = (y2 - y1) / (x2 - x1)
		// (y 절편): intercept_y = cur_position.y -  inclination * cur_position.x

		double inclination = (req.orbit_center.y_long - cur_position.y ) / (req.orbit_center.x_lat - cur_position.x);
		double A = inclination;
		double intercept_y = cur_position.y -  inclination * cur_position.x;
		double B = intercept_y;

		cout << " y = Ax + B " << endl;
		cout << " A = " << A << endl;
		cout << " B = " << B << endl;
		
		// 원의 방정식 (표준형): (x-center_x)^2 + (y-center_y)^2 = req.orbit_radius^2
		// center_x = point.x, center_y = point.y, r = req.orbit_radius
		double center_x = req.orbit_center.x_lat;		
		//double center_x = req.x_lat;
		//double C = center_x;
		double center_y = req.orbit_center.y_long;
		//double center_y = req.y_long;
		//double D = center_y;
		double r = req.orbit_radius;

		// 원의 방정식 (일반형): x^2 + y^2 + Cx + Dy + E = 0

		double C =  (-2) * center_x;
		double D =  (-2) * center_y;
		double E = pow ( center_x, 2) + pow ( center_y, 2) - pow ( r, 2);
				
		cout << " x^2 + y^2 + Cx + Dy + E = 0 " << endl;
		cout << " C = " << C << endl;
		cout << " D = " << D << endl;
		cout << " E = " << E << endl;

		// 직선의 방정식을 원의 방정식에 대입 -> x에 대한 이차방정식으로 표현

		// Fx^2 + Gx + H = 0
		// F = A^2 + 1
		// G = 2AB + C + AD
		// H = B^2 + BD + E

		double F = pow (A, 2) + 1;
		double G = 2* A*B + C + A*D;
		double H = pow (B, 2) + B*D + E;

		cout << " Fx^2 + Gx + H = 0" << endl;
		cout << " F = " << F << endl;
		cout << " G = " << G << endl;
		cout << " H = " << H << endl;

		// #1-2 원과 직선과의 교점 (2개) 계산

		// x = (-G +- sqrt(G^2-4FH))/2F
		
		Point cross_pt1, cross_pt2;

		if ( (pow (G,2) - 4 * F * H) < 0)
		{
			cout <<" 교점을 구할 수 없음" << endl;
			return false;
		}

		cross_pt1.x = ( (-1) * G + sqrt ( pow(G,2) - 4*F*H ) ) / (2*F);
		cross_pt2.x = ( (-1) * G - sqrt ( pow(G,2) - 4*F*H ) ) / (2*F);
		cross_pt1.y = 	inclination * 	cross_pt1.x + intercept_y;
		cross_pt2.y = 	inclination * 	cross_pt2.x + intercept_y;
	
		cout << "crossing pt1: << (" << cross_pt1.x << ", " << cross_pt1.y << ")" << endl;
		cout << "crossing pt2: << (" << cross_pt2.x << ", " << cross_pt2.y << ")" << endl;

		// #2 첫 번째 목적지 계산, path 에 추가
		

		double dist1 =0;
		double dist2 =0; 

		dist1=  pow ( (cross_pt1.x - cur_position.x), 2) + pow ( (cross_pt1.y - cur_position.y), 2);
		dist2=  pow ( (cross_pt2.x - cur_position.x), 2) + pow ( (cross_pt2.y - cur_position.y), 2);

		cout << "dist1: " <<dist1 << "dist2: " << dist2 << endl;
	
		Target_Position starting_pt;		

		//Target_Position target;

		target_position.ref_system = "ENU";
		target_position.reached = false;

		if (dist1 < dist2)
		{
			target_position.pos_local.x = cross_pt1.x;
			target_position.pos_local.y = cross_pt1.y;
			target_position.pos_local.z = takeoff_altitude;
		

			/*
			starting_pt.pos_local.x = cross_pt1.x;
			starting_pt.pos_local.y = cross_pt1.y;
			starting_pt.pos_local.z = takeoff_altitude;
			*/
		}

		else
		{
			target_position.pos_local.x = cross_pt2.x;
			target_position.pos_local.y = cross_pt2.y;
			target_position.pos_local.z = takeoff_altitude;

			/*
			starting_pt.pos_local.x = cross_pt2.x;
			starting_pt.pos_local.y = cross_pt2.y;
			starting_pt.pos_local.z = takeoff_altitude;
			*/

			cout << " target position: (" << target_position.pos_local.x << ", target_position.pos_local.y: " << target_position.pos_local.y << endl;	
		}
			orbit_path.push_back (target_position);


		// #2 나머지 목적지들 (원형 비행 경로) 계산, path에 추가 
		// #2-1 선회 비행 경로 상에서 시작점이 원점과 이루는 각도 계산
	        
		double rel_x = target_position.pos_local.x - req.orbit_center.x_lat;

		double rel_y = target_position.pos_local.y - req.orbit_center.y_long;
		double radian =  atan2 (rel_y, rel_x);

		double degree = radian * (180 / PI);

		cout << "원점과 시작점이 이루는 각도: (radian) " << radian << ", (degree) " << degree << endl;
			

		// #2-2 나머지 지점들의 좌표 계산, path에 추가 (요청된 선회 비행 횟수만큼 반복) 	

		for (int orbit_cnt = 0; orbit_cnt <orbit_req_cnt; orbit_cnt++)
		{

			for (double theta = 0; theta < 2*PI; theta += 0.05)
			{
				Target_Position point;
				point.pos_local.x = req.orbit_radius * cos(radian + theta) + center_x;
				point.pos_local.y = req.orbit_radius * sin(radian + theta) + center_y;
				point.pos_local.z = takeoff_altitude;
				point.reached = false;
				orbit_path.push_back (point);
			
				//cout << "(x, y) = (" << point.pos_local.x << ", " << point.pos_local.y << ")" << endl;
			}	
		}
	//	sleep(10);	
	}

	else if (req.orbit_ref_system == "WGS84" )
	{
		// 구현 예정 
	}	
	

	// phase 설정 
	
//	cur_phase.phase = phase;
	result = true;

	return result;
}

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
	noflyZones_sub = nh.subscribe<eDrone_msgs::NoflyZones> ("eDrone_msgs/noflyZones", 10, noflyZones_cb );

	//// 서비스 서버 선언
	arming_srv_server = nh.advertiseService("srv_arming", srv_arming_cb);
	takeoff_srv_server = nh.advertiseService("srv_takeoff", srv_takeoff_cb);
	landing_srv_server = nh.advertiseService("srv_landing", srv_landing_cb);	
	modeChange_srv_server = nh.advertiseService("srv_modeChange", srv_modeChange_cb);
	rtl_srv_server = nh.advertiseService("srv_rtl", srv_rtl_cb);
	goto_srv_server = nh.advertiseService("srv_goto", srv_goto_cb);
	gotoPath_srv_server = nh.advertiseService("srv_gotoPath", srv_gotoPath_cb);
	surveyArea_srv_server = nh.advertiseService("srv_surveyArea", srv_surveyArea_cb);
	orbit_srv_server = nh.advertiseService("srv_orbit", srv_orbit_cb);
	
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

//		cout << "eDrone_control_node: cur_phase = " << cur_phase.phase << endl;

		// 현재 phase 토픽 출판

		cur_phase.phase = phase;
		cur_phase_pub.publish (cur_phase);

		ros::spinOnce();
		rate.sleep();	

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
			// 이륙 후, 또는 직전 위치 이동 명령 수행 후, 다음 명령을 대기하고 있는 상태
			// 위치 이동 관련 명령이 호출되면, 경로 재설정 후 READY 상태로 전이 -> 경로 비행  

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

					target_pos_local.pose.position.x = current_pos_local.pose.position.x;
					target_pos_local.pose.position.y = current_pos_local.pose.position.y;
					target_pos_local.pose.position.z = current_pos_local.pose.position.z;

					/*
					target_pos_local.pose.position.x = target_position.pos_local.x;
					target_pos_local.pose.position.y = target_position.pos_local.y;
					target_pos_local.pose.position.z = target_position.pos_local.z;
					*/
					pos_pub_local.publish (target_pos_local);
					ros::spinOnce();
					rate.sleep();
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

				if (target_position.ref_system== "ENU")
				{
					cur_target.x_lat = target_position.pos_local.x;
					cur_target.y_long = target_position.pos_local.y;
					cur_target.z_alt = target_position.pos_local.z;
				}
				else if (target_position.ref_system== "WGS84")
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


				// yaw 제어 테스트
				target_pos_local.pose.orientation.x = PI; 
			
				pos_pub_local.publish (target_pos_local);
			}			

			else// target position에 도착했으면 path 검사
			{
			  // path에 목적지 정보가 있으면 첫 번째 데이터를 읽어 와서 target position 갱신 
			

				if (!path.empty() )
				{
					target_position = path[0];

					
					target_position.pos_local.x = path[0].pos_local.x;
					target_position.pos_local.y = path[0].pos_local.y;
					target_position.pos_local.z = path[0].pos_local.z;
					target_position.pos_global.latitude = path[0].pos_global.latitude;
					target_position.pos_global.longitude = path[0].pos_global.longitude;
					target_position.pos_global.altitude = path[0].pos_global.altitude;
					
					target_position.target_seq_no = cur_target.target_seq_no + 1;
					target_position.reached = false;

					//cout <<" target_seq_no: " << target_position.target_seq_no << endl;

					//cout <<" target position: (" << target_position.pos_local.x  << ", " << target_position.pos_local.y << ")" << endl;

					//path.erase(path.begin());

					// 갱신된 target 정보 publish (응용 프로그램에 타겟 정보 전달)
					
					cur_target.target_seq_no = target_position.target_seq_no;
					cur_target.ref_system = target_position.ref_system;
					cur_target.is_global = target_position.is_global;
					cur_target.reached = target_position.reached;			
				
					if (target_position.ref_system== "ENU")
					{
						cur_target.x_lat = target_position.pos_local.x;
						cur_target.y_long = target_position.pos_local.y;
						cur_target.z_alt = target_position.pos_local.z;
					}
					else if (target_position.ref_system== "WGS84")
					{
						cur_target.x_lat = target_position.pos_global.latitude;
						cur_target.y_long = target_position.pos_global.longitude;
						cur_target.z_alt = target_position.pos_global.altitude;
					}
					cout <<" eDrone_control_node: main() - arrived at the target position. path is not empty() " << endl;
					cout <<" target_seq_no: " << cur_target.target_seq_no << endl;
					cout <<" target position: (" << cur_target.x_lat << ", " << cur_target.y_long << ")" << endl;

					//sleep(5);
					// 위치 이동을 위한 publish 
					target_pos_local.pose.position.x = target_position.pos_local.x;
					target_pos_local.pose.position.y = target_position.pos_local.y;
					target_pos_local.pose.position.z = target_position.pos_local.z;
					path.erase(path.begin());
				}

			  // path가 비어 있으면 READY phase로 전이
				else
				{

					cout << "eDrone_control_node: Goto phase: path is empty" <<endl; 
	
				       //cout <<" eDrone_control_node: main() - arrived at the target position. path is empty() " << endl;
					//cout <<" target_seq_no: " << target_position.target_seq_no << endl;
					phase = "READY";
					//sleep(5);					
				}
			}
		}

		else if (phase.compare ("ORBIT") ==0) // (2018.10.05)
		{
			//ROS_INFO ("PHASE = ORBIT");

			double theta = 0 ;
			//while ( ros::ok()) // 기준점 중심으로 반복 선회 비행 
			{
				if (target_position.reached != true) // 현재 목적지에 도착 전이면 해당 좌표 출판 (pub) 
				{

				//	cout << "target_position: (" << target_position.pos_local.x << ", " << target_position.pos_local.y << ", " << target_position.pos_local.z << ")" << endl;  
					
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
					ros::spinOnce();

					
					rate.sleep();
				}
				else // 현재 목적지에 도착했으면 다음 목적지 정보 설정  
				{
					if (!orbit_path.empty()) // 선회 비행 경로 변수에 목적지가 남아 있으면 해당 위치로 이동
					{
						target_position = orbit_path[0];
						
						orbit_path.erase(orbit_path.begin());

						// 갱신된 target 정보 publish (응용 프로그램에 타겟 정보 전달)
						cur_target.x_lat = target_position.pos_local.x;
						cur_target.y_long = target_position.pos_local.y;
						cur_target.z_alt = target_position.pos_local.z;

						target_pos_local.pose.position.x = target_position.pos_local.x;
						target_pos_local.pose.position.y = target_position.pos_local.y;
						target_pos_local.pose.position.z = target_position.pos_local.z;
						pos_pub_local.publish (target_pos_local);
						phase = "ORBIT";
						cur_phase.phase = phase;
						cur_phase_pub.publish (cur_phase);
						ros::spinOnce();
						rate.sleep();

					}	
					else // 선회 비행을 마쳤으면 READY 단계로 전이 
					{
						cout << "orbit path is empty. Go to READY phase" << endl;
						phase = "READY";			
						cur_phase.phase = phase;
					}
				}
			} 
		}
		

		ros::spinOnce();
	  	rate.sleep();
	
	}
	return 0;
}

