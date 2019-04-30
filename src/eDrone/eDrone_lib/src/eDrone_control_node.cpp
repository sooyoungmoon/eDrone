

/* header file */

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
//#include <mavlink/v2.0/common/mavlink.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandHome.h> // (2019.04.10)
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

// eDrone (ROS Topics, ROS Services, Data types, parameters)
#include <eDrone_msgs/Target.h> // 현재 목적지 topic 메시지가 선언된 헤더 파일 포함
#include <eDrone_msgs/Phase.h> // 무인기 임무 수행 단계  
#include <eDrone_msgs/Geofence.h> // 가상울타리 정보 
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
//#include <eDrone_msgs/SurveyPath.h> // 경로 탐색 (경로 이동, 사진 촬영, +a)
#include <eDrone_msgs/Orbit.h> // hotPoint API를 위한 선회 비행 
#include <eDrone_lib/Vehicle.h>
#include <eDrone_lib/GeoInfo.h>
#include <eDrone_lib/GeoUtils.h>
#include <eDrone_lib/types.h>
#include <eDrone_lib/params.h>

using namespace std;
using namespace mavros_msgs;
using namespace eDrone_msgs;

// test functionss
void printMentalMap (Mental_Map* mental_map_ptr,
                     const int AREA_WIDTH,
                     const int AREA_HEIGHT);

void printPath (std::vector<Target_Position> altPath);

void printWavefrontMap(int** mental_map,
                       int AREA_WIDTH,
                       int AREA_HEIGHT);

// 경로 계산 함수

vector<Target_Position> getOrbitPath();// 선회비행경로계산

std::vector<Target_Position> getCoveragePath(vector<geometry_msgs::Point> points,
                                             double altitude,
                                             double interval);

void initCell (Cell* cell_ptr,
               int index_x,
               int index_y,
               const int CELL_WITDH,
               const int CELL_HEIGHT,
               const double base_x,
               const double base_y);

void updateMap(Mental_Map* mental_map_ptr,
               int curCell_x,
               int curCell_y);

bool isOccupiedCell (Cell* cell_ptr);

void checkNoflyZoneCells(Mental_Map* mental_map); // (Mental_Map 상에서) 비행금지 구역에 속한 cell 표시 

void printPoint(Point point); // Point 출력 함수 

Target_Position target_position;// (04.30) // 현재 목적지 정보 (published topic)

int num_targets; // 목적지 개수 (goto service마다 1 씩 증가)

// Subscribed ROS topics
mavros_msgs::State current_state; // 무인기 상태 정보
geometry_msgs::PoseStamped current_pos_local; // 현재 위치 및 자세 정보 (지역 좌표)
sensor_msgs::NavSatFix current_pos_global; // 현재 위치 정보 (전역 좌표)
NoflyZones nfZones;
Geofence geofence;

// Published ROS topics
geometry_msgs::PoseStamped base_pos_local; // 실질적인　원점 (ENU) -（goto 명령을　내릴　때，　목적지　좌표에　더함）
geometry_msgs::PoseStamped target_pos_local; // 목적지 위치 정보 (지역 좌표)
mavros_msgs::GlobalPositionTarget target_pos_global; // 목적지 위치 정보 (전역 좌표)
eDrone_msgs::Target cur_target; // 현재 목적지 정보 (publisher: eDrone_control_node, subscriber: 응용 프로그램)
eDrone_msgs::Phase cur_phase; // 현재 무인기　상태 （ex. UNARMED, ARMED, READY, GOTO, ...)
std::vector<Target_Position> path; // 자율 비행 경로

// Orbit API (hotPoint) 관련 변수
std::vector<Target_Position> orbit_path; // 특정 위치를 기준으로 하는 선회 비행 경로
int orbit_req_cnt = 0; // (요청된) 선회 비행 횟수
int orbit_cnt = 0; // 현재 선회 비행 횟수
Target orbit_center; // (04/30)
double orbit_radius; // (04/30)

//　ROS Service 요청 메시지 선언 (mavros)
mavros_msgs::CommandBool arming_cmd; // 시동 명령에 사용될 서비스 선언 
mavros_msgs::CommandTOL takeoff_cmd; // 이륙 명령에 사용될 서비스 선언 
mavros_msgs::CommandTOL landing_cmd; // 착륙 명령에 사용될 서비스 선언 
mavros_msgs::CommandLong commandLong_cmd;// 무인기 제어에 사용될 서비스 선언
mavros_msgs::SetMode modeChange_cmd; // 모드 변경에 사용될 서비스 요청 메시지
mavros_msgs::SetMode rtl_cmd; // 복귀 명령에 사용될 서비스 요청 메시지
mavros_msgs::CommandHome setHome_cmd; // (2019.04.11) setHome　요청　메시지　
eDrone_msgs::GeofenceCheck geofenceCheck_cmd; // 가상 울타리 확인에 사용될 요청 메시지 
eDrone_msgs::NoflyZoneCheck noflyZoneCheck_cmd; // 비행 금지 구역 확인에 사용될 요청 메시지 

ros::Publisher pos_pub_local;
ros::Publisher pos_pub_global;
ros::Publisher cur_target_pub; // (offboard control에 필요한) 현재 목적지 정보 (도착 여부 포함)
ros::Publisher cur_phase_pub; // 현재 무인기 임무 수행 단계 정보

ros::Subscriber state_sub;
ros::Subscriber pos_sub_local;
ros::Subscriber pos_sub_global;
ros::Subscriber home_sub; 
ros::Subscriber noflyZones_sub; 
ros::Subscriber geofence_sub; 

ros::ServiceServer arming_srv_server;
ros::ServiceServer takeoff_srv_server;
ros::ServiceServer landing_srv_server;
ros::ServiceServer modeChange_srv_server;
ros::ServiceServer rtl_srv_server;
ros::ServiceServer goto_srv_server;
ros::ServiceServer gotoPath_srv_server;
ros::ServiceServer surveyArea_srv_server;
ros::ServiceServer orbit_srv_server;

ros::ServiceClient arming_client; // 서비스 클라이언트 선언
ros::ServiceClient takeoff_client; // 서비스 클라이언트 선언
ros::ServiceClient landing_client; // 서비스 클라이언트 선언
ros::ServiceClient modeChange_client; // 비행 모드 변경 서비스 클라이언트 선언
ros::ServiceClient rtl_client; // 복귀 서비스 클라이언트 
ros::ServiceClient geofenceCheck_client;
ros::ServiceClient noflyZoneCheck_client;
ros::ServiceClient setHome_client; // (2019.04.10) setHome client

double HOME_LAT;
double HOME_LON;
double HOME_ALT;

double takeoff_altitude = 0; // 이륙　고도

// (0424) goto
Target src;
Target dest;

// (0423) survey
string survey_ref_system = "";
vector<eDrone_msgs::Target> survey_points;
double survey_altitude = 0;
double survey_interval = 0;

vector<Target_Position> getOrbitPath()// 선회비행경로계산
{
    std::vector<Target_Position> path;
    printf("getOrbitPath() was called\n");


    printf("center: (%lf,%lf,%lf)\n", orbit_center.x_lat, orbit_center.y_long, orbit_center.z_alt);
    printf("radius: %lf\n", orbit_radius);
    printf("cnt: %d\n", orbit_req_cnt);

    //sleep(10);
    // #1. 선회 비행 시작점 계산
    Point cur_position; // 현재 위치
    cur_position.x = current_pos_local.pose.position.x;
    cur_position.y = current_pos_local.pose.position.y;

    double inclination = (orbit_center.y_long - cur_position.y ) / (orbit_center.x_lat - cur_position.x);
    double A = inclination;
    double intercept_y = cur_position.y -  inclination * cur_position.x;
    double B = intercept_y;

    cout << " y = Ax + B " << endl;
    cout << " A = " << A << endl;
    cout << " B = " << B << endl;

    double center_x = orbit_center.x_lat;
    double center_y = orbit_center.y_long;
    double r = orbit_radius;
    double C =  (-2) * center_x;
    double D =  (-2) * center_y;
    double E = pow ( center_x, 2) + pow ( center_y, 2) - pow ( r, 2);

    // 직선의 방정식을 원의 방정식에 대입 -> x에 대한 이차방정식으로 표현

    double F = pow (A, 2) + 1;
    double G = 2* A*B + C + A*D;
    double H = pow (B, 2) + B*D + E;

    Point cross_pt1, cross_pt2;

    if ( (pow (G,2) - 4 * F * H) < 0)
    {
        cout <<" 교점을 구할 수 없음" << endl;
        return path;
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

    Target_Position target;

    target.ref_system = "ENU";
    target.reached = false;

    if (dist1 < dist2)
    {
        target.pos_local.x = cross_pt1.x;
        target.pos_local.y = cross_pt1.y;
        target.pos_local.z = takeoff_altitude;
    }

    else
    {
        target.pos_local.x = cross_pt2.x;
        target.pos_local.y = cross_pt2.y;
        target.pos_local.z = takeoff_altitude;
    }
    path.push_back (target);

    // #3 나머지 목적지들 (원형 비행 경로) 계산, path에 추가

    double rel_x = target.pos_local.x - orbit_center.x_lat;
    double rel_y = target.pos_local.y - orbit_center.y_long;
    double radian =  atan2 (rel_y, rel_x);
    double degree = radian * (180 / PI);

    for (int orbit_cnt = 0; orbit_cnt <orbit_req_cnt; orbit_cnt++)
    {

        cout << "orbit_cnt: " <<  orbit_cnt << endl;

        for (double theta = 0; theta < 2*PI; theta += 0.05)
        {
            Target_Position point;
            point.pos_local.x = orbit_radius * cos(radian + theta) + center_x;
            point.pos_local.y = orbit_radius * sin(radian + theta) + center_y;
            point.pos_local.z = takeoff_altitude;
            point.ref_system = "ENU";
            point.reached = false;
            path.push_back (point);
        }
    }

    return path;
}

// 경로 계산
vector<Target_Position> getIndirectPath(Target src, Target dest)//　비행금지구역 우회경로 계산
{
    std::vector<Target_Position> path;

    printf("eDrone_control_node: getIndirectPath()");
    Point src_pt;
    Point dest_pt;

    if( src.ref_system == "WGS84") // ENU 좌표계로 통일
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

    // Wavefront 맵 범위 지정, cell 배열 생성
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

    Mental_Map mental_map;
    Mental_Map* mental_map_ptr = &mental_map;

    int AREA_WIDTH = ceil ((x_max - x_min) / CELL_WIDTH)  ;
    int AREA_HEIGHT = ceil ((y_max - y_min) / CELL_HEIGHT)  ;

    mental_map.area_width = AREA_WIDTH;
    mental_map.area_height = AREA_HEIGHT;
    mental_map.grid = new Cell*[AREA_WIDTH+1];

    // CELL 배열 동적할당

    for (int c = 0; c < AREA_WIDTH+1; c++)
    {
        mental_map.grid[c] = new Cell[AREA_HEIGHT+1];
    }

    // Cell 정보 초기화
    for (int i = 0; i < AREA_WIDTH+1; i++)
    {
        for(int j = 0; j < AREA_HEIGHT+1;j++)
        {
            Cell* cell_ptr = &(mental_map.grid[i][j]);
            initCell(cell_ptr, i, j, CELL_WIDTH, CELL_HEIGHT, x_min, y_min);
        }
    }
    // src CELL, dst CELL 생성
    Cell* src_cell_ptr= NULL;
    Cell* dst_cell_ptr = NULL;

    int src_cell_index_x = (src.x_lat - x_min) / CELL_WIDTH;
    int src_cell_index_y = (src.y_long - y_min) / CELL_HEIGHT;
    src_cell_ptr = &(mental_map.grid[src_cell_index_x][src_cell_index_y]);

    int dst_cell_index_x = (dest.x_lat - x_min) / CELL_WIDTH;
    int dst_cell_index_y = (dest.y_long - y_min) / CELL_HEIGHT;
    dst_cell_ptr = &(mental_map.grid[dst_cell_index_x][dst_cell_index_y]);

    //  WavefrontMap 생성
    // label - 1: 장애물, 2: 비행금지구역, 3: 방문된 cell, 4: Goal cell
    checkNoflyZoneCells(&mental_map);

    int ** waveFrontMap = new int*[AREA_WIDTH+1];

    for (int c = 0; c < AREA_WIDTH+1; c++)
    {
        waveFrontMap[c] = new int[AREA_HEIGHT+1];
    }

    for (int x_index = 0; x_index < AREA_WIDTH+1; x_index++)
    {
        for(int y_index = 0; y_index < AREA_HEIGHT+1; y_index++)
        {
            if ( mental_map.grid[x_index][y_index].noflyZone == true)
            {
                waveFrontMap[x_index][y_index] = 2; // noflyZone에 속한 cell을  wavefront 상에 표시
            }
            else
            {
                waveFrontMap[x_index][y_index] = 0;
            }
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
            if((index_x + local_index_x)<0 ||
                    (index_x+local_index_x ) > (AREA_WIDTH) )
            {
                continue;
            }

            for (int local_index_y= -1; local_index_y < 2; local_index_y++)
            {
                // y 인덱스의 범위가 mental_map 범위를 벗어 나면 continue
                if ( ((index_y+local_index_y) < 0 ) ||
                     ((index_y+local_index_y) > (AREA_HEIGHT) )  )
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
                else if ( (neighbor->noflyZone) == true)
                {
                    waveFrontMap[index_x_neighbor][index_y_neighbor] = 2;
                }

                // Case#4 그 밖의 경우
                else
                {
                    waveFrontMap[index_x_neighbor][index_y_neighbor] =
                            waveFrontMap[index_x][index_y] +1; //(2018.10.02)
                    queue.push_back ( neighbor );
                    if (neighbor == src_cell_ptr) {// queue에 추가된 노드가 starting node이면
                        pathExist = true; // s->g까지 경로 존재 여부를 true로 설정
                    }
                }
            }

        }
    }

    ////  비행금지구역 우회 경로 계산

    if (pathExist != true) // src-dst path가 존재하지 않으면 빈 path 반환
    {
        return path;
    }

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

        int min_label = -1;
        Cell* neighbor_min_label = NULL;

        int index_x = cur_cell_ptr->index_x;
        int index_y = cur_cell_ptr->index_y;

        for (int local_index_x = -1; local_index_x < 2; local_index_x++ )
        {
            int index_x_neighbor = index_x + local_index_x;

            if ( index_x_neighbor < 0 ||
                 index_x_neighbor > AREA_WIDTH) {continue;}

            for (int local_index_y = -1; local_index_y < 2; local_index_y++)
            {
                int index_y_neighbor = index_y + local_index_y;

                if ( index_y_neighbor < 0 || index_y_neighbor > AREA_HEIGHT) {continue;}

                if (local_index_x ==0 &&
                        local_index_y ==0) { continue; } // 자기 자신은 탐색 대상에서 제외

                neighbor_cell_ptr = &(mental_map_ptr->grid[index_x_neighbor][index_y_neighbor]);

                if (neighbor_cell_ptr->includedInPath == true)
                {
                    continue;
                } // 이미 경로에 추가된 cell은 제외
                if (waveFrontMap[index_x_neighbor][index_y_neighbor] == 1)
                {
                    continue;
                } // 장애물이 있는 cell은 제외

                else if (waveFrontMap[index_x_neighbor][index_y_neighbor] == 2)
                {
                    continue;
                } // 비행금지구역에 속한 cell은 제외
                else if (waveFrontMap[index_x_neighbor][index_y_neighbor] == 3)
                {
                    continue;
                }// 기 방문된 cell은 제외

                if ( &mental_map_ptr->grid[index_x_neighbor][index_y_neighbor]
                     == dst_cell_ptr)
                {
                    // 이웃 cell이 목적지 cell인 경우, break
                    min_label = waveFrontMap[index_x_neighbor][index_y_neighbor];
                    neighbor_min_label =  &mental_map_ptr->grid[index_x_neighbor][index_y_neighbor];
                    pathComputed = true;
                    break;
                }

                if ( min_label < 0 ||
                     ( waveFrontMap[index_x_neighbor][index_y_neighbor] < min_label) )// 이웃 cell 들 중 label 값이 가장 큰 cell 탐색
                {
                    min_label = waveFrontMap[index_x_neighbor][index_y_neighbor];
                    neighbor_min_label = &mental_map_ptr->grid[index_x_neighbor][index_y_neighbor];
                }
            }

            if (pathComputed == true) { break; }

        } // 현재 노드의 이웃 노드 검사 - 다음 노드 선택

        cout << "neighbor_min_label: (" <<  neighbor_min_label->x << "," << neighbor_min_label->y << ")" << endl;

        if (neighbor_min_label != NULL) {
            cur_cell_ptr = neighbor_min_label;

            // 고도값 설정
            if (cur_cell_ptr->occupied == true)
            {
                cur_cell_ptr-> z = cur_cell_ptr->obstacle_height + dest_pt.z;
            }
            else
            {
                cur_cell_ptr-> z = dest_pt.z;
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

    // add dest cell to the path
    Target_Position target_position;
    target_position.ref_system = "ENU";
    target_position.pos_local.x =  dest_pt.x;
    target_position.pos_local.y =  dest_pt.y;
    target_position.pos_local.z =  dest_pt.z;
    path.push_back(target_position); // path에 dest 추가

    return path;
}

bool  isOccupiedCell (Cell* cell_ptr) { return false;}

void initCell (Cell* cell_ptr,
               int index_x,
               int index_y,
               const int CELL_WIDTH,
               const int CELL_HEIGHT,
               const double base_x,
               const double base_y)
{
    cell_ptr->index_x = index_x;
    cell_ptr->index_y = index_y;
    cell_ptr->x = index_x * CELL_WIDTH + base_x;
    cell_ptr->y = index_y * CELL_HEIGHT + base_y;
    cell_ptr->z = 0;
    cell_ptr->visited = false;
    cell_ptr->noflyZone = false;
    cell_ptr->occupied = false;
    cell_ptr->observed = false;
    cell_ptr->rel_altitude_real= 0;
    cell_ptr->rel_altitude_estimated = 0;
    cell_ptr->includedInPath = false; // (2018.11.07)
}

void checkNoflyZoneCells (Mental_Map* mental_map)
{
    int area_width = mental_map->area_width;
    int area_height = mental_map-> area_height;

    cout << "checkNoflyZoneCells(); " << endl;
    int cnt = 0;

    // Noflyzone을 하나씩 검사
    for ( vector<NoflyZone>::iterator it = nfZones.noflyZones.begin();
          it != nfZones.noflyZones.end();
          it++)
    {
        NoflyZone nfZone = *it;
        cout << "NoflyZone#" <<cnt++ << endl;

        vector<Point> points;

        // 각 NoflyZone의 x 범위, y 범위 계산
        // 위 범위에 속한 cell들에 대해서만 isInside() 함수를 호출하여 비행금지구역 포함 여부 계산

        double x_min = -1, x_max =1;
        double y_min = -1, y_max = -1;
        int pcnt = 0;

        for (vector<Target>::iterator it = nfZone.noflyZone_pts.begin(); it!=nfZone.noflyZone_pts.end(); it++)
        {
            Target boundary_point = *it;
            Point point;

            if (nfZone.noflyZone_ref_system == "WGS84")
            {
                point = convertGeoToENU(boundary_point.x_lat,
                                        boundary_point.y_long, boundary_point.z_alt, HOME_LAT, HOME_LON, HOME_ALT);

            }
            else if ( nfZone.noflyZone_ref_system  == "ENU")
            {
                point.x = boundary_point.x_lat;
                point.y = boundary_point.y_long;
                point.z = boundary_point.z_alt;
            }

            if (x_min < 0 || point.x < x_min)
            {
                x_min = point.x;
            }
            if (x_max < 0 || point.x > x_max)
            {
                x_max = point.x;
            }
            if (y_min < 0 || point.y < y_min)
            {
                y_min = point.y;
            }
            if (y_max < 0 || point.y > y_max)
            {
                y_max = point.y;
            }

            points.push_back(point);

        }
        // 비행금지구역에 속한 Cell 확인
        for (int i = 0; i < area_width+1; i++)
        {
            for(int j = 0; j < area_height+1;j++)
            {
                Cell* cell_ptr = &(mental_map->grid[i][j]);
                Point cell_point;
                cell_point.x = cell_ptr->x;
                cell_point.y = cell_ptr->y;
                cell_point.z = cell_ptr->z;

                if ( (cell_ptr->x < x_min) != (cell_ptr->x < x_max) )
                {
                    if ( (cell_ptr->y < y_min) != (cell_ptr->y < y_max) )
                    {
                        if (isInside(cell_point, points)==true)
                        {
                            mental_map->grid[i][j].noflyZone = true;
                        }
                    }
                }
            }
        }
    }
}



bool isNoflyZoneCell (Cell* cell_ptr) { // 주어진 cell이 Noflyzone에 속하는 지 판단 


    Point cell_point;

    cell_point.x = cell_ptr->x;
    cell_point.y = cell_ptr->y;
    cell_point.z = cell_ptr->z;

    for (vector<NoflyZone>::iterator it = nfZones.noflyZones.begin(); it!= nfZones.noflyZones.end(); it++)
    {
        NoflyZone nfZone = *it;

        vector<Target> noflyZone_pts = nfZone.noflyZone_pts;

        vector<Point> boundary_pts;

        for(vector<Target>::iterator it = noflyZone_pts.begin(); it != noflyZone_pts.end(); it++)
        {
            Target point = *it;
            Point boundary_point;

            if (point.ref_system == "WGS84")
            {
                boundary_point = convertGeoToENU(point.x_lat, point.y_long, point.z_alt, HOME_LAT, HOME_LON, HOME_ALT);
            }
            else if (point.ref_system == "ENU")
            {
                boundary_point.x = point.x_lat;
                boundary_point.y = point.y_long;
                boundary_point.z = point.z_alt;
            }

            boundary_pts.push_back(boundary_point);
        }

    }
    return false;
}

void printMentalMap (Mental_Map* mental_map_ptr,
                     const int AREA_WIDTH,
                     const int AREA_HEIGHT)
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
{
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


void printPath (std::vector<Target_Position> altPath) //
{
    cout << "eDrone_control_node: printPath() " << endl;
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

    printf ("eDrone_control_node: getCoveragePath() was called");

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

    //cout << "entry point: (" << entry_point.x_lat << ", " << entry_point.y_long << ")" << endl;
    //cout << "exit point: (" << exit_point.x_lat << ", " << exit_point.y_long <<  ")" << endl;

    // Mental Map & Grid 생성

    Mental_Map mental_map;

    Mental_Map* mental_map_ptr = &mental_map;

    int CELL_WIDTH = interval;
    int CELL_HEIGHT = interval;
    // cout << "CELL_WIDTH: " << CELL_WIDTH << endl;
    // cout << "CELL_HEIGHT: " << CELL_HEIGHT << endl ;

    int AREA_WIDTH = ceil ((x_max - x_min) / CELL_WIDTH)  ;
    int AREA_HEIGHT = ceil ((y_max - y_min) / CELL_HEIGHT)  ;

    /*
    cout << "x: " << x_min << " ~ " << x_max << endl;
    cout << "y: " << y_min << " ~ " << y_max << endl;
    cout << "AREA_WIDTH: " << AREA_WIDTH << endl ;
    cout << "AREA_HEIGHT: " << AREA_HEIGHT << endl;
    */
    mental_map.area_width = AREA_WIDTH;
    mental_map.area_height = AREA_HEIGHT;
    mental_map.grid = new Cell*[AREA_WIDTH+1];

    // CELL 배열 동적 할당

    for (int c = 0; c < AREA_WIDTH+1; c++)
    {
        mental_map.grid[c] = new Cell[AREA_HEIGHT+1];
    }

    // cout << "Dynamic allocation" << endl;

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

    // cout << " (src cell) index_x:" << src_cell_ptr-> index_x << ", index_y: " << src_cell_ptr-> index_y << "x: " << src_cell_ptr->x << "y: " << src_cell_ptr->y << endl;

    int dst_cell_index_x = (exit_point.x_lat - x_min) / CELL_WIDTH;
    int dst_cell_index_y = (exit_point.y_long - y_min) / CELL_HEIGHT;
    dst_cell_ptr = &(mental_map.grid[dst_cell_index_x][dst_cell_index_y]);
    // cout << " (dst cell) index_x:" << dst_cell_ptr-> index_x << ", index_y: " << dst_cell_ptr-> index_y << "x: " << dst_cell_ptr->x << "y: " << dst_cell_ptr->y << endl;

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
                else if ( isNoflyZoneCell(neighbor) == true)
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
        // cout << "neighbor_max_label: (" <<  neighbor_max_label->x << "," << neighbor_max_label->x << ")" << endl;
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
    return path;
}


void updateMap(Mental_Map* mental_map_ptr, int curCell_x, int curCell_y)
{
    printf("updateMap was called");

    cout << " Current Cell index: " << curCell_x << ", " << curCell_y << endl;
    // 현재 위치 확인

    const int AREA_WIDTH = mental_map_ptr->area_width;
    const int AREA_HEIGHT = mental_map_ptr->area_height;
    int SENSING_RANGE= 0 ;

    if ( AREA_WIDTH > AREA_HEIGHT) SENSING_RANGE = AREA_WIDTH;
    else SENSING_RANGE = AREA_HEIGHT;

    // 각 방향에 대해 가장 가까운 장애물 위치를 확인하고 mental_map 확장
    for (int local_index_x = (-1) * SENSING_RANGE;
         local_index_x <SENSING_RANGE;
         local_index_x++ ) // 현재 cell을 기준으로 8개 방향의 장애물 확인 & mental_map  확장
    {

        int index_x = curCell_x + local_index_x;

        if (index_x <0  || index_x > AREA_WIDTH-1) // x,y 인덱스가 mental map 범위를 벗어나면 continue
        {
            continue;
        }

        for (int local_index_y = (-1) * SENSING_RANGE;
             local_index_y < SENSING_RANGE;
             local_index_y++  )
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

                bool isOccupied = isOccupiedCell (observedCell);

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


void ref_system_conversion_test()
{
    printf("reference system conversion test: \n");

    printf("GeoPoint => Point\n");
    Point point = convertGeoToENU(HOME_LAT, HOME_LON, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );
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

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}


void pos_cb_local(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

    current_pos_local = *msg;

    double DIST_RANGE = 0.5;

    static int idx = 0;

    if (idx++ % 10 ==0)
    {
        printf("\t\t\t\t\t current_position (ENU): (%f, %f, %f ) (%s) \n",
               current_pos_local.pose.position.x,
               current_pos_local.pose.position.y,
               current_pos_local.pose.position.z,
               cur_phase.phase.c_str());

    }

    if (cur_phase.phase.compare("TAKEOFF")==0)
    {
        if (current_pos_local.pose.position.z >= base_pos_local.pose.position.z + takeoff_altitude- DIST_RANGE )
        {
            cout << "eDrone_control_node: TAKEOFF completed!" << endl;
            cur_phase.phase = "READY";
        }

        return;
    }

    if ( (cur_phase.phase.compare("GOTO")==0) || (cur_phase.phase.compare("ORBIT")==0) )
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
                        printf("pos_cb_local(): The UAV reached to the target position");

                        printf("current_position: (%f, %f, %f \n)", current_pos_local.pose.position.x, current_pos_local.pose.position.y, current_pos_local.pose.position.z);
                    }


                }

            }
        }

    }
}

void pos_cb_global(const sensor_msgs::NavSatFix::ConstPtr& msg){

    current_pos_global = *msg;
    static int idx2 = 0;
    if (idx2++ % 10 ==0)
    {
        printf("\t\t current_position (WGS84): (%f, %f, %f ) (%s) \n",
               current_pos_global.latitude,
               current_pos_global.longitude,
               current_pos_global.altitude,
               cur_phase.phase.c_str());

    }
}



void homePosition_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
    HOME_LAT = msg->geo.latitude;
    HOME_LON = msg->geo.longitude;
    HOME_ALT = msg->geo.altitude;

    static int i = 0;

    if (i++ < 10 )
    {
        printf("control_node: home position: (%lf, %lf, %lf) \n\n", HOME_LAT, HOME_LON, HOME_ALT);

    }

}

void noflyZones_cb(const eDrone_msgs::NoflyZones::ConstPtr& msg)
{
    // 현재 목적지 도달 여부 확인
    //printf("eDrone_control_node: noflyZones_cb(): \n");
    nfZones = *msg;
}

// callback 함수 (서비스 제공) 정의
void geofence_cb(const eDrone_msgs::Geofence::ConstPtr& msg)
{   
    geofence = *msg;
}

bool srv_arming_cb(eDrone_msgs::Arming::Request &req, eDrone_msgs::Arming::Response &res )
{
    printf("arming_cb(): ARMing request received\n");
    arming_cmd.request.value = true; // 서비스 요청 메시지 필드 설정

    //// Arming
    while (ros::ok() ) // 서비스 요청 메시지 전달
    {
        printf("eDrone_control_node: send Arming command ...\n");

        if (!arming_client.call(arming_cmd))
        {
            ros::spinOnce();
        }
        else break;
    }

    printf("ARMing command was sent\n");
    cur_phase.phase = "ARMED";

    return true;
}

bool srv_takeoff_cb(eDrone_msgs::Takeoff::Request &req, eDrone_msgs::Takeoff::Response &res)
{
    takeoff_altitude = req.takeoff_altitude; // 이륙 고도 저장

    printf("takeoff_cb(): takeoff request received\n");

    takeoff_cmd.request.altitude = 1;
    printf(" HOME_ALT: %lf, req.takeoff_altitude: %lf", HOME_ALT, req.takeoff_altitude);
    takeoff_cmd.request.latitude = current_pos_global.latitude;
    takeoff_cmd.request.longitude = current_pos_global.longitude;
    takeoff_cmd.request.min_pitch = 0;

    base_pos_local.pose.position.x = current_pos_local.pose.position.x;
    base_pos_local.pose.position.y = current_pos_local.pose.position.y;
    base_pos_local.pose.position.z = current_pos_local.pose.position.z;

    target_position.pos_local.x = base_pos_local.pose.position.x;
    target_position.pos_local.y = base_pos_local.pose.position.y;
    target_position.pos_local.z = base_pos_local.pose.position.z + takeoff_altitude;
    target_position.reached = false;
    target_position.ref_system = "ENU";

    while (ros::ok() )
    {
        printf("send Takeoff command ...\n");

        if (!takeoff_client.call(takeoff_cmd))
        {
            sleep(1);
        }
        else break;
    }
    cur_phase.phase = "TAKEOFF";
    printf("Takeoff command was sent\n");

    return true;
}

bool srv_landing_cb(eDrone_msgs::Landing::Request &req, eDrone_msgs::Landing::Response &res)
{

    printf("Landing request received\n");
    //// 서비스 요청 메시지 필드 설정

    landing_cmd.request.altitude = 10;
    landing_cmd.request.latitude = HOME_LAT;
    landing_cmd.request.longitude = HOME_LON;
    landing_cmd.request.min_pitch = 0;
    landing_cmd.request.yaw = 0;

    while (ros::ok() )
    {
        printf("send Landing command ...\n");

        if (!landing_client.call(landing_cmd))
        {
            sleep(1);
        }
        else break;

    }
    printf("Landing command was sent\n");

    return true;
}

bool srv_modeChange_cb(eDrone_msgs::ModeChange::Request &req, eDrone_msgs::ModeChange::Response &res)
{

    std::cout << "srv_modeChange_cb(): change the mode to "
              << req.modeChange_mode << endl;

    modeChange_cmd.request.base_mode = 0;

    modeChange_cmd.request.custom_mode.assign(req.modeChange_mode);

    if (modeChange_client.call(modeChange_cmd)==true)
    {
        std::cout << " modeChange cmd was sent!\n " << endl;
    }
    else
    {
        cout << " modeChange cmd failed!\n " << endl;
        return false;
    }

    return true;
}

bool srv_rtl_cb(eDrone_msgs::RTL::Request &req, eDrone_msgs::RTL::Response &res)
{
    std::cout << "srv_rtl_cb():return to home"  << endl;

    modeChange_cmd.request.base_mode = 0;

    modeChange_cmd.request.custom_mode.assign("AUTO.RTL");

    if (modeChange_client.call(modeChange_cmd)==true)
    {
        cout << " modeChange cmd was sent!\n " << endl;
    }
    else
    {
        cout << " modeChange cmd failed!\n " << endl;
        return false;
    }
    cur_phase.phase = "RTL";


    return true;
}


bool srv_goto_cb(eDrone_msgs::Goto::Request &req, eDrone_msgs::Goto::Response &res)
{
    Target_Position requested_target; // we use local variable, not a global varialble

    printf("eDrone_control_node: Goto request received\n");

    cout<< "req.goto_point.x_lat: " << req.goto_point.x_lat << ", req.goto_point.y_long: " << req.goto_point.y_long << endl;

    /* goto 서비스 처리 절차 */

    // 좌표계 종류 확인 (ex. WGS84, ENU) & 목적지 좌표 (지역, 전역) 저장
    if (req.goto_ref_system.compare("WGS84")==0) // 전역 좌표인 경우
    {
        requested_target.pos_global.latitude = req.goto_point.x_lat;
        requested_target.pos_global.longitude = req.goto_point.y_long;
        requested_target.pos_global.altitude = req.goto_point.z_alt;

        // ENU로 좌표변환
        Point point = convertGeoToENU(req.goto_point.x_lat,
                                      req.goto_point.y_long,
                                      req.goto_point.z_alt,
                                      HOME_LAT,
                                      HOME_LON,
                                      HOME_ALT );

        requested_target.pos_local.x = point.x;
        requested_target.pos_local.y = point.y;
        requested_target.pos_local.z = point.z;
    }

    else if (req.goto_ref_system.compare("ENU")==0)
    {
        requested_target.pos_local.x = base_pos_local.pose.position.x + req.goto_point.x_lat;
        requested_target.pos_local.y = base_pos_local.pose.position.y +req.goto_point.y_long;
        requested_target.pos_local.z = base_pos_local.pose.position.z +req.goto_point.z_alt;

        // WGS84로 좌표 변환
        GeoPoint geoPoint = convertENUToGeo(requested_target.pos_local.x,
                                            requested_target.pos_local.y,
                                            requested_target.pos_local.z,
                                            HOME_LAT, HOME_LON, HOME_ALT );

        requested_target.pos_global.latitude = geoPoint.latitude;
        requested_target.pos_global.longitude = geoPoint.longitude;
        requested_target.pos_global.altitude = geoPoint.altitude;

    }

    // Geofence 검사
    double distance_to_home = sqrt ( pow ( (double) requested_target.pos_local.x, (double) 2) + pow ( (double) requested_target.pos_local.y , (double) 2) );

    if (distance_to_home > geofence.geofence_radius)
    {
        cout << "control_node-  goto_cb(): target position is outside of the geofence!!" << endl;
        cout << "distance_to_home: " << distance_to_home << ", geofence_radius: " << geofence.geofence_radius << endl;
        res.value = false; // 목적지가 비행금지구역 내에 있으면 goto 명령 거부
        return true; //
    }

    // NoflyZone 검사　－　비행 금지 구역과 src-dst 간 직선경로가 겹치는 경우, path 재설정 필요
    printf("eDrone_control_node: noflyZoneCheck");

    src.ref_system = "ENU";
    src.x_lat = current_pos_local.pose.position.x;
    src.y_long =  current_pos_local.pose.position.y;
    src.z_alt =  current_pos_local.pose.position.z;

    dest.ref_system = "ENU";
    dest.x_lat = requested_target.pos_local.x;
    dest.y_long =  requested_target.pos_local.y;
    dest.z_alt =  requested_target.pos_local.z;

    noflyZoneCheck_cmd.request.noflyZoneCheck_src =src;
    noflyZoneCheck_cmd.request.noflyZoneCheck_dest.ref_system = dest.ref_system; // req.goto_ref_system;
    noflyZoneCheck_cmd.request.noflyZoneCheck_dest.x_lat = req.goto_point.x_lat;
    noflyZoneCheck_cmd.request.noflyZoneCheck_dest.y_long = req.goto_point.y_long;
    noflyZoneCheck_cmd.request.noflyZoneCheck_dest.z_alt = req.goto_point.z_alt;

    if (noflyZoneCheck_client.call(noflyZoneCheck_cmd))
    {
        cout << " noflyZoneCheck API was called " << endl;
        // printf("noflyZoneCheck result: %s ", noflyZoneCheck_cmd.response.result.c_str() );
        if (noflyZoneCheck_cmd.response.result == "DST_IN_NF")
        {
            printf("noflyZoneCheck result: DST_IN_NF ");
            res.value = false; // 목적지가 비행금지구역 내에 있으면 goto 명령 거부
            return true; //
        }

        else if (noflyZoneCheck_cmd.response.result ==  "PATH_OVERLAP")
        {
            printf("noflyZoneCheck result: PATH_OVERLAP ");

            // (2019.04.24)
            cur_phase.phase = "PLANNING_GOTO";

            res.value = true;
        }
        else
        {
            printf("noflyZoneCheck result: NONE ");
            // path에 목적지 정보 저장
            res.value = true;
            requested_target.reached = false;
            printf("goto_cb(): requested_target 'push'! (%lf,%lf,%lf)\n " , requested_target.pos_local.x, requested_target.pos_local.y, requested_target.pos_local.z );
            // （０４１０）　출력문　추가
            path.push_back (requested_target);
        }
    }
    else
    {
        cout << " noflyZoneCheck API call failed! " << endl;
        return false;
    }

    return true;
}

bool srv_gotoPath_cb(eDrone_msgs::GotoPath::Request &req, eDrone_msgs::GotoPath::Response &res)
{	

    printf("eDrone_control_node: GotoPath request received\n");
    cout << "gotoPath_cb(): ref_system: " << req.gotoPath_ref_system << endl;

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

            target_position.reached = false;

            // path에 목적지 또는 부분 경로 추가
            path.push_back (target_position);
        }
    }
    else if (req.gotoPath_ref_system.compare("ENU") == 0) // 지역 좌표
    {
        for (vector<Target>::iterator it = req.gotoPath_pts.begin(); it != req.gotoPath_pts.end(); it++)
        {
            Target target = *it;
            target_position.ref_system = "ENU";
            target_position.pos_local.x = target.x_lat;
            target_position.pos_local.y = target.y_long;
            target_position.pos_local.z = target.z_alt;

            // WGS84로 좌표변환
            GeoPoint geoPoint = convertENUToGeo(target.x_lat, target.y_long, target.z_alt, HOME_LAT, HOME_LON, HOME_ALT );
            target_position.pos_global.latitude = geoPoint.latitude;
            target_position.pos_global.longitude = geoPoint.longitude;
            target_position.pos_global.altitude = geoPoint.latitude;
            target_position.reached = false;
            path.push_back (target_position);  // path에 목적지 또는 부분 경로 추가
            printf("gotoPath_cb(): target_position 'push'! (%lf,%lf,%lf)\n " , target_position.pos_local.x, target_position.pos_local.y, target_position.pos_local.z );
        }
    }

    return true;
}

bool srv_surveyArea_cb(eDrone_msgs::SurveyArea::Request &req, eDrone_msgs::SurveyArea::Response &res)
{

    printf ("eDrone_control_node: surveyArea service was called");

    cur_phase.phase = "PLANNING_SURVEY";

    survey_ref_system = req.surveyArea_ref_system;
    survey_points = req.surveyArea_pts;
    survey_altitude = req.surveyArea_altitude;
    survey_interval =  req.surveyArea_interval;

    return true;
}

bool srv_orbit_cb(eDrone_msgs::Orbit::Request &req,
                  eDrone_msgs::Orbit::Response &res)
{
    printf ("eDrone_control_node: Orbit service was called");

    orbit_center.ref_system = req.orbit_ref_system;
    orbit_center  = req.orbit_center;
    orbit_radius = req.orbit_radius;
    orbit_req_cnt = req.orbit_req_cnt;

    // path computation
    vector<Target_Position> computedPath = getOrbitPath();
    printf("computed path:");
    printPath(computedPath);

    for(vector<Target_Position>::iterator it = computedPath.begin(); it != computedPath.end(); it++ )
    {
        int c=0;
        Target_Position pos = *it;
        orbit_path.push_back(pos);
        c++;
    }

    target_position = computedPath[0];
    target_position.ref_system = "ENU";
    target_position.reached = false;

    cur_phase.phase = "ORBIT";

    return true;
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
    geofence_sub = nh.subscribe<eDrone_msgs::Geofence> ("eDrone_msgs/geofence", 10, geofence_cb );

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

    //// 서비스 클라이언트 선언

    arming_client = nh.serviceClient<mavros_msgs::CommandBool> ("mavros/cmd/arming"); // service client 선언
    takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/takeoff"); // 서비스 클라이언트 선언
    landing_client = nh.serviceClient<mavros_msgs::CommandTOL> ("mavros/cmd/land"); // 서비스 클라이언트 선언
    noflyZoneCheck_client = nh.serviceClient<eDrone_msgs::NoflyZoneCheck> ("srv_noflyZoneCheck" );
    geofenceCheck_client = nh.serviceClient<eDrone_msgs::GeofenceCheck> ("srv_geofenceCheck" );
    rtl_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
    modeChange_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
    setHome_client = nh.serviceClient<mavros_msgs::CommandHome> ("/mavros/cmd/set_home" ); // (2019.04.10) setHOme client
    target_position.reached = false; // 목적지 도착 여부를 false로 초기화

    //변수　초기화
    target_pos_local.pose.position.x = 0;
    target_pos_local.pose.position.y = 0;
    target_pos_local.pose.position.z = 0;

    current_pos_local.pose.position.x = 0;
    current_pos_local.pose.position.y = 0;
    current_pos_local.pose.position.z = 0;

    cur_phase.phase = "UNARMED";

    while (ros::ok() )
    {
        // 현재 phase 토픽 출판
        cur_phase_pub.publish (cur_phase);

        ros::spinOnce();
        rate.sleep();

        if (cur_phase.phase.compare ("UNARMED") ==0)
        {
            // 시동 명령 대기
        }
        else if (cur_phase.phase.compare ("ARMED") ==0)
        {
            // 이륙 명령 대기
        }
        else if (cur_phase.phase.compare ("TAKEOFF") == 0) // 현재 이륙 중인 경우
        {
            //cout << "eDrone_control_node: publish Topic to takeoff - altitude " << takeoff_altitude <<  endl;

            // (2019.04.24) publish setpoint messages before mode change to 'TAKEOFF'
            // please refer to the following example
            // https://dev.px4.io/en/ros/mavros_offboard.html

            // publish topic

            if (  current_pos_local.pose.position.x !=0 || // current_pos_local　값　초기화　여부　확인
                  current_pos_local.pose.position.y !=0 ||
                  current_pos_local.pose.position.z !=0)
            {
                if (base_pos_local.pose.position.z !=0) // base_pos_local 값　초기화　여부　확인
                {
                    target_pos_local.pose.position.x = current_pos_local.pose.position.x;
                    target_pos_local.pose.position.y = current_pos_local.pose.position.y;
                    target_pos_local.pose.position.z= base_pos_local.pose.position.z + takeoff_altitude;

                    for (int i = 100; ros::ok() && i >0; --i)
                    {
                        pos_pub_local.publish (target_pos_local);
                    }
                }
            }

            // change mode to offboard
            modeChange_cmd.request.base_mode = 0;
            modeChange_cmd.request.custom_mode = "OFFBOARD";

            if (modeChange_client.call(modeChange_cmd))
            {

                if (modeChange_cmd.response.mode_sent)
                {
                    cout << "(takeoff phase) offboard enabled" << endl;
                }

            }
            else
            {
                cout << "mode change failed!" << endl;
            }

        }
        else if (cur_phase.phase.compare ("READY") ==0)
        {
            // 이륙 후, 또는 직전 위치 이동 명령 수행 후, 다음 명령을 대기

            if (!path.empty() )
            {
                target_position = path[0];
                path.erase(path.begin());
                target_position.reached = false; // (2019.04.10)
                cur_phase.phase = "GOTO";
                printf("main(): target_position 'pop'! (%lf,%lf,%lf)\n " , target_position.pos_local.x, target_position.pos_local.y, target_position.pos_local.z );

            }
            else
            {
                // 갱신된 target 정보 publish (응용 프로그램에 타겟 정보 전달)
                cur_target.ref_system = target_position.ref_system;
                cur_target.x_lat = target_position.pos_local.x;
                cur_target.y_long = target_position.pos_local.y;
                cur_target.z_alt = target_position.pos_local.z;
                cur_target.reached = target_position.reached;
                cur_target_pub.publish(cur_target);

                if (  target_position.pos_local.x != 0 || // target_position　값　초기화　여부　확인
                      target_position.pos_local.y != 0 ||
                      target_position.pos_local.z != 0)
                {
                    // 현재 목적지 위치 publish (실제 위치 이동)
                    target_pos_local.pose.position.x = target_position.pos_local.x;
                    target_pos_local.pose.position.y = target_position.pos_local.y;
                    target_pos_local.pose.position.z = target_position.pos_local.z;
                    pos_pub_local.publish(target_pos_local);
                }
            }

        }
        else if (cur_phase.phase.compare ("GOTO") ==0)
        {
            // (2019.04.22)
            // OFFBOARD 모드로　비행　모드　변환

            modeChange_cmd.request.base_mode = 0;
            modeChange_cmd.request.custom_mode = "OFFBOARD";
            if (modeChange_client.call(modeChange_cmd))
            {
                if (modeChange_cmd.response.mode_sent)
                {
                    cout << "OFFBOARD mode enabled" << endl;
                }
            }
            else
            {
                cout << "mode change failed!" << endl;
            }


            // target position으로 현재 이동 중이면 위치 정보 publish
            if (target_position.reached != true)
            {

                // 현재 목적지 정보 publish (응용 프로그램에 target 정보 제공)
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
                /*
                if (  target_pos_local.pose.position.x !=0 || // target_position　값　초기화　여부　확인
                      target_pos_local.pose.position.y !=0 ||
                      target_pos_local.pose.position.z !=0)*/
                {
                    // 현재 목적지 위치 publish (실제 위치 이동)
                    target_pos_local.pose.position.x = target_position.pos_local.x;
                    target_pos_local.pose.position.y = target_position.pos_local.y;
                    target_pos_local.pose.position.z = target_position.pos_local.z;
                    pos_pub_local.publish (target_pos_local);
                }
            }

            else// target position에 도착했으면 path 검사
            {
                // path에 목적지 정보가 있으면 첫 번째 데이터를 읽어 와서 target position 갱신
                if (!path.empty() )
                {
                    target_position = path[0];
                    printf("main(): target_position 'pop'! (%lf,%lf,%lf)\n " , target_position.pos_local.x, target_position.pos_local.y, target_position.pos_local.z );
                    path.erase(path.begin());
                    target_position.reached = false;

                    // 갱신된 target 정보 publish (응용 프로그램에 타겟 정보 전달)

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


                }

                // path가 비어 있으면 READY phase로 전이
                else
                {
                    cout << "eDrone_control_node: Goto phase: path is empty" <<endl;
                    cur_phase.phase = "READY";
                }
            }
        }
        // (2019.04.29) phase 추가　（PLANNING_GOTO)

        else if (cur_phase.phase.compare ("PLANNING_GOTO") ==0) // (201９.04.22)
        {
            printf ("PHASE = PLANNING_GOTO");

            // change mode to loiter
            modeChange_cmd.request.base_mode = 0;
            modeChange_cmd.request.custom_mode = "AUTO.LOITER";
            if (modeChange_client.call(modeChange_cmd))
            {
                if (modeChange_cmd.response.mode_sent)
                {
                    cout << "AUTO.LOITER enabled" << endl;
                }
            }
            else
            {
                cout << "mode change failed!" << endl;
            }

            // 경로　계산
            vector<Target_Position> indirectPath = getIndirectPath(src, dest);
            printPath(indirectPath);
            for( vector<Target_Position>::iterator it = indirectPath.begin();
                 it != indirectPath.end(); it++ )
            {
                Target_Position waypoint = *it;
                waypoint.reached = false;
                path.push_back(waypoint);
            } // path에 indirectPath 추가
            //　경로　계산　완료

            // OFFBOARD 모드로　비행　모드　변환

            // change mode to offboard
            modeChange_cmd.request.base_mode = 0;
            modeChange_cmd.request.custom_mode = "OFFBOARD";
            if (modeChange_client.call(modeChange_cmd))
            {
                if (modeChange_cmd.response.mode_sent)
                {
                    cout << "OFFBOARD enabled" << endl;
                }
            }
            else
            {
                cout << "mode change failed!" << endl;
            }

            cur_phase.phase = "GOTO";
        }

        else if (cur_phase.phase.compare ("PLANNING_SURVEY") ==0) // (201９.04.22)
        {
            printf ("PHASE = PLANNING_SURVEY");

            // AUTO.LOITER 모드로　비행　모드　변환

            // change mode to loiter
            modeChange_cmd.request.base_mode = 0;
            modeChange_cmd.request.custom_mode = "AUTO.LOITER";
            if (modeChange_client.call(modeChange_cmd))
            {
                if (modeChange_cmd.response.mode_sent)
                {
                    cout << "AUTO.LOITER enabled" << endl;
                }
            }
            else
            {
                cout << "mode change failed!" << endl;
            }


            // 탐색　경로　계산

            vector<Target_Position> coveragePath;

            if (survey_ref_system != "" &&  survey_ref_system == "ENU")
            {
                if (survey_points.empty())
                {
                    cout << "(phase == PLANNING_SURVEY): coverage path cannot be computed! no survey point" << endl;
                }

                if (survey_altitude==0 || survey_interval==0)
                {
                    cout << "(phase == PLANNING_SURVEY): coverage path cannot be computed! (altitude or interval not initialized" << endl;
                }

                coveragePath = getCoveragePath(survey_points, survey_altitude, survey_interval);
            }
            else if (survey_ref_system == "WGS84")
            {
                // ENU 좌표로 변환
                vector<Target> temp_pts;

                for (vector<Target>::iterator it = survey_points.begin(); it != survey_points.end(); it++)
                {
                    Target target = *it;

                    // ENU로 좌표변환
                    Point point = convertGeoToENU(target.x_lat, target.y_long, HOME_ALT, HOME_LAT, HOME_LON, HOME_ALT );
                    target.x_lat = point.x;
                    target.y_long = point.y;
                    target.ref_system = "ENU";
                    temp_pts.push_back(target);
                }

                survey_points = temp_pts;

                if (survey_points.empty())
                {
                    cout << "(phase == PLANNING_SURVEY): coverage path cannot be computed! no survey point" << endl;
                }

                if (survey_altitude==0 || survey_interval==0)
                {
                    cout << "(phase == PLANNING_SURVEY): coverage path cannot be computed! (altitude or interval not initialized" << endl;
                }

                coveragePath = getCoveragePath(survey_points, survey_altitude, survey_interval);
            }

            printPath(coveragePath);

            for (vector<Target_Position>::iterator it = coveragePath.begin(); it != coveragePath.end(); it++ )
            {
                Target_Position target_position = *it;
                target_position.reached = false;
                path.push_back(target_position);
            } // path에 coveragePath 추가

            // OFFBOARD 모드로　비행　모드　변환

            // change mode to offboard
            modeChange_cmd.request.base_mode = 0;
            modeChange_cmd.request.custom_mode = "OFFBOARD";
            if (modeChange_client.call(modeChange_cmd))
            {
                if (modeChange_cmd.response.mode_sent)
                {
                    cout << "OFFBOARD enabled" << endl;
                }
            }
            else
            {
                cout << "mode change failed!" << endl;
            }

            cur_phase.phase = "GOTO";


        }

        else if (cur_phase.phase.compare ("ORBIT") ==0) // (2018.10.05)
        {
            printf ("PHASE = ORBIT");
            double theta = 0 ;
            {
                if (target_position.reached != true) // 현재 목적지에 도착 전이면 해당 좌표 출판 (pub)
                {
                    // 현재 목적지 정보 publish (응용 프로그램에 target 정보 제공)
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

                    if (  target_pos_local.pose.position.x !=0 || // target_position　값　초기화　여부　확인
                          target_pos_local.pose.position.y !=0 ||
                          target_pos_local.pose.position.z !=0)
                    {
                        // 현재 목적지 위치 publish (실제 위치 이동)
                        target_pos_local.pose.position.x = target_position.pos_local.x;
                        target_pos_local.pose.position.y = target_position.pos_local.y;
                        target_pos_local.pose.position.z = target_position.pos_local.z;
                        pos_pub_local.publish (target_pos_local);
                    }

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

                        if (  target_pos_local.pose.position.x !=0 || // target_position　값　초기화　여부　확인
                              target_pos_local.pose.position.y !=0 ||
                              target_pos_local.pose.position.z !=0)
                        {
                            pos_pub_local.publish (target_pos_local);
                        }

                        cur_phase.phase = "ORBIT";
                        cur_phase_pub.publish (cur_phase);
                        ros::spinOnce();
                        rate.sleep();
                    }
                    else // 선회 비행을 마쳤으면 READY 단계로 전이
                    {
                        cout << "orbit path is empty. Go to READY phase" << endl;
                        cur_phase.phase = "READY";
                    }
                }
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

