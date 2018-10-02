


// (2018.05.25)
// Sooyoung Moon
// types.h
// RUS Problem Solving 알고리즘 구현에  필요한 데이터 타입 선언

//#include <params.h>
#include <vector>

using namespace std;

/* 데이터 타입 선언 순서 */


// 1. cell

	
	
typedef struct str_cell
{
  int index_x;
  int index_y;
  double x; // x: 원점 기준으로 동쪽 방향의 상대 좌표
  double y; // y: 원점 기준으로 북쪽 방향의 상대 좌표 
  double lat;
  double lon;
  bool visited;
  bool occupied; // 장애물 존재 여부 
  int obstacle_height; // 장애물의 높이 (m)
  bool observed; 
  double rel_altitude_real;
  double rel_altitude_estimated; 
} Cell;


// 2. mental map

typedef struct str_mental_map
{
  Cell** grid; // A Grid G (n-by-m cells)
  vector<Cell*> visitedCells; // the set of visited cells  
  vector<Cell*> freeCells; // the set of free cells
  vector<Cell*> occupiedCells; // the set of occupied cells
  int area_width;
  int area_height;
} Mental_Map; 


// 3. wavefront map
typedef struct str_wavefront_map
{
  int* label[]; // label (the next goal cell 기준)
} Wavefront_Map; 

// 4.
 


