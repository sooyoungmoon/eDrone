


#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <eDrone_lib/Vehicle.h>
#include <eDrone_lib/GeoUtils.h>
#include <eDrone_lib/types.h>
#include <eDrone_msgs/NoflyZoneCheck.h>

using namespace std;
using namespace Mission_API;

/*
// test 함수 
void printMentalMap (Mental_Map* mental_map_ptr, const int AREA_WIDTH, const int AREA_HEIGHT);
//void printAltPath (std::vector<Target_Position> altPath);

// 경로 계산 함수
//std::vector<Target_Position> getAltPath(Target_Position src, Target_Position dst);
//std::vector<Target_Position> getCoveragePath(vector<geometry_msgs::Point> points, double altitude, double interval);

//void initCell (Cell* cell_ptr, int index_x, int index_y, const int CELL_WITDH, const int CELL_HEIGHT, const double base_x, const double base_y)
void updateMap(Mental_Map* mental_map_ptr, int curCell_x, int curCell_y);
bool isOccupiedCell (Cell* cell_ptr) ;
*/

bool isOccupiedCell (Cell* cell_ptr) { return false;}

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
}



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

	for (int index_y=area_height-1; index_y >= 0; index_y--)
	{
		for (int index_x = 0; index_x < area_width; index_x++)
		{
			cout << " " << m[index_x][index_y] << " ";
		}
		cout << endl;
	}
	sleep(10);
  
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
std::vector<Target_Position> getCoveragePath(vector<geometry_msgs::Point> points, double altitude, double interval)
{
	std::vector<Target_Position> path;


 	return path;
}

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


