
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
#include <eDrone_lib/Vehicle.h>
#include <eDrone_lib/GeoUtils.h>
#include <eDrone_lib/types.h>

using namespace std;
using namespace eDrone;

Vector_type crossproduct (Vector_type a, Vector_type b)
{
    Vector_type product;
    product.x = 0;
    product.y = 0;
    product.z = (a.x*b.y) - (a.y*b.x);
    return product;
}


bool pathOverlap(Point src, Point dest, const vector<Point> polygon_area)
{
    bool result = false;

    // 1) vector declaration
    Vector_type directPath;
    directPath.x = dest.x - src.x;
    directPath.y = dest.y - src.y;
    directPath.z = 0;

    int pCnt = 0;
    vector<Point> polygon = polygon_area;

    for (vector<Point>::iterator it = polygon.begin(); it!= polygon.end(); it++)
    {
        Point p = *it;
        pCnt++;
    }

    Vector_type polygon_line;

    // 2) check if src-dest path overlaps the polygon_area

    bool crossing = false;
    bool overlap = false;

    for (int i =0; i < polygon_area.size(); i++)
    {
        Point p1 = polygon_area[i];
        Point p2 = polygon_area[(i+1)%(polygon_area.size())];
        polygon_line.x = p2.x - p1.x; // polygon line connects point#i and point#(i+1)
        polygon_line.y = p2.y - p1.y;

        // check if src-dest path overlaps i-th polygon_line
        Vector_type vec1, vec2;
        vec1.x = p1.x - src.x; // (src->p1)
        vec1.y = p1.y - src.y;
        vec2.x = p2.x - src.x; // (src->p2)
        vec2.y = p2.y - src.y;

        Vector_type product1 =  crossproduct(directPath, vec1);
        Vector_type product2 =  crossproduct(directPath, vec2);

        Vector_type vec3, vec4;

        vec3.x = src.x - p1.x; // (p1->src)
        vec3.y = src.y - p1.y;

        vec4.x = dest.x - p1.x; // (p1->dest)
        vec4.y = dest.y - p1.y;

        Vector_type product3 = crossproduct(polygon_line, vec3);
        Vector_type product4 = crossproduct(polygon_line, vec4);

        if ( (product1.z*product2.z < 0) && ((product3.z*product4.z) < 0) )
        {
            crossing = true;
            overlap = true;
            break;
        }
    }

    // 3) return result

    if (overlap == true)
    {
        result = true;
    }
    return result;
}



//** 다각형 영역 내부 점 판단
bool isInside(Point point, const vector<Point> polygon_area)
{      
    vector<Point> polygon = polygon_area;
    for (vector<Point>::iterator it = polygon.begin();
         it != polygon.end();
         it++)
    {
        Point point = *it;
    }

    bool result = false;

    int crosses = 0; // point에서 오른쪽 방향 반직선과 polygon_area 간 교점 개수

    for (int i = 0; i < polygon_area.size(); i++)
    {
        int j = (i+1)%polygon_area.size();

        // point가 i번째 점과 j번째 점의 y좌표 사이에 위치하는지 확인

        if ( (polygon_area[i].y > point.y) != (polygon_area[j].y >point.y) )
        {
            // atX는 점 point를 지나는 반직선과 선분 (p[i], p[j]) 사이의 교점의 x 좌표

            // double atX = (polygon_area[i].y-point.y)*(polygon_area[j].x - polygon_area[i].x)/(polygon_area[j].y-polygon_area[i].y) + polygon_area[i].x;

            double atX = ((polygon_area[i].y-point.y)*((polygon_area[j].x - polygon_area[i].x)/(polygon_area[j].y-polygon_area[i].y))) + polygon_area[i].x;

            if (point.x < atX)
            {
                crosses++;
            }
        }

    }

    if ( (crosses%2) > 0)
    {
        result = true;
    }
    else
    {
        result = false;
    }

    return result;

}

//** GeoPoint (위도/경도/고도) <=> ENU 좌표 변환 코드 (QGC에서 발췌)

Point convertGeoToENU(double coord_lat, double coord_long, double coord_alt, double home_lat, double home_long, double home_alt) {

    double lat_rad = coord_lat * M_DEG_TO_RAD;
    double lon_rad = coord_long* M_DEG_TO_RAD;

    double ref_lon_rad = home_long * M_DEG_TO_RAD;
    double ref_lat_rad = home_lat * M_DEG_TO_RAD;

    double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double cos_d_lon = cos(lon_rad - ref_lon_rad);

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double c = acos( (ref_sin_lat * sin_lat) + (ref_cos_lat * cos_lat * cos_d_lon) );
    double k = (fabs(c) < epsilon) ? 1.0 : (c / sin(c));

    Point point;

    point.x = k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;
    point.y = k * ( (ref_cos_lat * sin_lat) - (ref_sin_lat * cos_lat * cos_d_lon) ) * CONSTANTS_RADIUS_OF_EARTH;

    point.z = coord_alt;
    // point.z = (coord_alt - home_alt);

    return point;
}

GeoPoint convertENUToGeo(double x, double y, double z, double home_lat, double home_long, double home_altitude) {

    double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
    double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
    double c = sqrtf( (x_rad * x_rad) + (y_rad * y_rad) );
    double sin_c = sin(c);
    double cos_c = cos(c);

    double ref_lon_rad = home_long * M_DEG_TO_RAD;
    double ref_lat_rad = home_lat * M_DEG_TO_RAD;

    double ref_sin_lat = sin(ref_lat_rad);
    double ref_cos_lat = cos(ref_lat_rad);

    double lat_rad;
    double lon_rad;

    if (fabs(c) > epsilon) {

        lat_rad = asin( (cos_c * ref_sin_lat) +   ( ((y_rad * sin_c) * ref_cos_lat) /c) ); // (0528)
        lon_rad = (ref_lon_rad + atan2(x_rad * sin_c, ( (c * ref_cos_lat) * cos_c) - ( (y_rad * ref_sin_lat) * sin_c)));

    } else {
        lat_rad = ref_lat_rad;
        lon_rad = ref_lon_rad;
    }

    GeoPoint geoPoint;



    double latitude = lat_rad * M_RAD_TO_DEG;
    
    double longitude = lon_rad * M_RAD_TO_DEG;

    double altitude = z + home_altitude;

    cout << fixed;


    //cout << "(" << latitude << ", " << longitude << ", " << altitude << ") " << endl;

    geoPoint.latitude  = latitude;
    geoPoint.longitude = longitude;
    geoPoint.altitude = altitude;

    return geoPoint;

}

//// 거리 계산 함수 (http://www.geodatasource.com/developers/c )


/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts decimal degrees to radians             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/



double deg2rad(double deg) {
    return (deg * PI / 180);
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/*::  This function converts radians to decimal degrees             :*/
/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/


double rad2deg(double rad) {
    return (rad * 180 / PI);
}

double distance(double lat1, double lon1, double lat2, double lon2, char unit) {
    double theta, dist;
    theta = lon1 - lon2;
    dist = (sin(deg2rad(lat1)) * sin(deg2rad(lat2))) + (cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(theta)));
    dist = acos(dist);
    dist = rad2deg(dist);
    dist = dist * 60 * 1.1515;
    switch(unit) {
    case 'm': // meter
        dist = dist * 1609.344;
        break;

    case 'M':
        break;
    case 'K':
        dist = dist * 1.609344;
        break;
    case 'N':
        dist = dist * 0.8684;
        break;

    default: // (2019.05.22) default statement
        break;
    }
    return (dist);
}
