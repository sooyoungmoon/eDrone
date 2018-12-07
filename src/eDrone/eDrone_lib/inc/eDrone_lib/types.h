


// (2018.10.18)
// Sooyoung Moon
// types.h
// eDrone API & library 구현에  필요한 데이터 타입 선언

//#include <params.h>
#include <vector>
#include <eDrone_msgs/Target.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace eDrone_msgs;

typedef struct _str_target_position
{
	int target_seq_no; // 목적지 순번 (1, 2, 3, ...)
	bool is_global; // 좌표 유형 (true: global coord, false: local coord)
	string ref_system; // 좌표 유형 - is_global 변수 대체  
	
	geometry_msgs::Point pos_local; // (x, y, z)
	geographic_msgs::GeoPoint pos_global; // (lat, long, alt)

	bool reached; // 목적지 도달 여부 (true: 목적지 도착, false: 목적지로 이동 중)
	bool geofence_violation; // geofence 영역 위반 여부 (true: geofence 영역 밖, false: 영역 내) 

	bool noflyZone_violation; // 비행 금지 구역 위반 여부 (true: 비행 금지 구역 내, false: 비행 금지 구역 외)
	
	string noflyZone_status; // 비행 금지 구역과 Src, Dst 간 위치 관계
				// 1) SRC_IN_NF_ZONE
				// 2) DST_IN_NF_ZONE
				// 3) PATH_OVERLAP_NF_ZONE

	bool takeshot; // 사진 촬영 여부 (True/False)

} Target_Position;


typedef struct c_str_vector
{
        double x;
        double y;
        double z;

} Vector_type;


//(2018.11.09)
/*
typedef struct c_str_nofly_Zone
{
	string ref_system;
	vector<Target> nfZone_pts;
} Nofly_Zone; // 1개의 비행금지구역을 저장할 수 있는 자료형 
*/

/* path planning 관련 데이터 타입 */


// cell

	
	
typedef struct str_cell
{
  int index_x;
  int index_y;
  double x; // x: 원점 기준으로 동쪽 방향의 상대 좌표
  double y; // y: 원점 기준으로 북쪽 방향의 상대 좌표 
  double z; // z: 원점 기준으로 z축 방향 좌표 
  double lat;
  double lon;
  double alt; // 고도 
  
  bool visited;
  bool noflyZone; // noflyZone (T/F)
  bool occupied; // 장애물 존재 여부 
  int obstacle_height; // 장애물의 높이 (m)
  bool observed; 
  double rel_altitude_real;
  double rel_altitude_estimated; 
  bool includedInPath; // src-dst 경로에 포함 여부 
} Cell;


// mental map

typedef struct str_mental_map
{
  Cell** grid; // A Grid G (n-by-m cells)
  vector<Cell*> visitedCells; // the set of visited cells  
  vector<Cell*> freeCells; // the set of free cells
  vector<Cell*> occupiedCells; // the set of occupied cells
  int area_width;
  int area_height;
} Mental_Map; 


// wavefront map
typedef struct str_wavefront_map
{
  int* label[]; // label (the next goal cell 기준)
} Wavefront_Map; 

// 4.
 


