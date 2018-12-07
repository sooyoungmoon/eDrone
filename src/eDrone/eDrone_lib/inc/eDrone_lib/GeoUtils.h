/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <geographic_msgs/GeoPoint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <limits>
#include <vector>

using namespace geometry_msgs;

#ifndef GeoUtils
#define GeoUtils

#define PI 3.14159265358979323846

//** 다각형 영역 내부 점 판단
bool isInside(Point point, std::vector<Point> polygon_area);




//// WGS84 (위도/경도/고도) <=> ENU 좌표 변환 코드 (QGC 코드 수정 )

// These defines are private
#define M_DEG_TO_RAD (M_PI / 180.0)

#define M_RAD_TO_DEG (180.0 / M_PI)

#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C		1.225f			/* kg/m^3		*/
#define CONSTANTS_AIR_GAS_CONST				287.1f 			/* J/(kg * K)		*/
#define CONSTANTS_ABSOLUTE_NULL_CELSIUS			-273.15f		/* °C			*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000			/* meters (m)		*/


using namespace geographic_msgs;
using namespace geometry_msgs;
using namespace std;



static const float epsilon = std::numeric_limits<double>::epsilon();




Point convertGeoToENU(double coord_lat, double coord_long, double coord_alt, double home_lat, double home_long, double home_alt);

GeoPoint convertENUToGeo(double x, double y, double z, double home_lat, double home_long, double home_altitude);

//Point convertGeoToPoint(double coord_lat, double coord_long, double coord_alt, double home_lat, double home_long, double home_alt);

//GeoPoint convertPointToGeo(double x, double y, double z, double home_lat, double home_long, double home_altitude);




//// 좌표 변환 코드




//// 거리 계산 함수 (http://www.geodatasource.com/developers/c ) (LGPLv3)


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/



double deg2rad(double deg);

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/


double rad2deg(double rad);

double distance(double lat1, double lon1, double lat2, double lon2, char unit);

#endif

