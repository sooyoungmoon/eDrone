
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
#include <geometry_msgs/Point.h>
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
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
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
eDrone_msgs::NoflyZones nfZones; // 비행금지구역 (다수)
eDrone_msgs::Geofence geofence; // 가상울타리 
mavros_msgs::State current_state; // 무인기 상태 정보
geometry_msgs::PoseStamped current_pos_local; // 현재 위치 및 자세 정보 (지역 좌표)
sensor_msgs::NavSatFix current_pos_global; // 현재 위치 정보 (전역 좌표)

// ROS Service servers
ros::ServiceServer geofenceSet_srv_server;
ros::ServiceServer geofenceCheck_srv_server;
ros::ServiceServer geofenceReset_srv_server;
ros::ServiceServer noflyZoneSet_srv_server;
ros::ServiceServer noflyZoneReset_srv_server;
ros::ServiceServer noflyZoneCheck_srv_server;

// publisher 선언
ros::Publisher noflyZones_pub;
ros::Publisher geofence_pub;

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


// 현재 상태 확인 
void state_cb(const mavros_msgs::State::ConstPtr& msg){

    current_state = *msg;

}

// 현재 위치 (지역좌표) 확인 
void pos_cb_local(const geometry_msgs::PoseStamped::ConstPtr& msg){

    current_pos_local = *msg;

    double DIST_RANGE = 0.5;

    // geofence check

    double distance_home = 0;

    distance_home = ((current_pos_local.pose.position.x) * (current_pos_local.pose.position.x)) + ((current_pos_local.pose.position.y) * (current_pos_local.pose.position.y));

    distance_home = sqrt(distance_home);

    if (distance_home >  geofence.geofence_radius)
    {
        cout << "geofence violation!!" << endl;
    }
}

// 현재 위치 (전역 좌표) 확인 

void pos_cb_global(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    current_pos_global = *msg;
}

void homePosition_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
    HOME_LAT = msg->geo.latitude;
    HOME_LON = msg->geo.longitude;
    HOME_ALT = msg->geo.altitude;
}



bool srv_noflyZoneSet_cb(eDrone_msgs::NoflyZoneSet::Request &req, eDrone_msgs::NoflyZoneSet::Response &res)
{

    // reference system: WGS84 지원

    printf ("eDrone_safety_node: NoflyZoneSet service was called");


    if ((req.noflyZoneSet_ref_system.compare("WGS84") ==0)||
            (req.noflyZoneSet_ref_system.compare("ENU") ==0))
    {
        NoflyZone nofly_zone;
        nofly_zone.noflyZone_ref_system = req.noflyZoneSet_ref_system;
        nofly_zone.noflyZone_pts = req.noflyZoneSet_pts;
        nfZones.noflyZones.push_back(nofly_zone);
    }
}


bool srv_noflyZoneReset_cb(eDrone_msgs::NoflyZoneReset::Request &req, eDrone_msgs::NoflyZoneReset::Response &res)
{	    
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
    cout << "(" <<  req.noflyZoneCheck_dest.x_lat << ", " << req.noflyZoneCheck_dest.y_long << ", " << req.noflyZoneCheck_dest.z_alt << ")" <<endl;
    cout << "target type: " << req.noflyZoneCheck_dest.ref_system << endl;

    //#2 비행금지구역　검사

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

    // 경계점 목록의 좌표계를 ENU로 변환 i


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


    //#３ 결과값 반환
    return true;
}

bool srv_geofenceSet_cb(eDrone_msgs::GeofenceSet::Request &req, eDrone_msgs::GeofenceSet::Response &res)
{
    cout << " safety_node - geofenceSet_cb():" << endl;
    cout << " geofence radius:" << geofence.geofence_radius;
    geofence.geofence_radius = req.geofenceSet_radius;
    cout << " -> " << geofence.geofence_radius << endl;

    return true;
}


bool srv_geofenceReset_cb(eDrone_msgs::GeofenceReset::Request &req, eDrone_msgs::GeofenceReset::Response &res)
{
    //(2019.04.23)
    geofence.geofence_radius = GEOFENCE_RADIUS;
    return true;
}


bool srv_geofenceCheck_cb(eDrone_msgs::GeofenceCheck::Request &req, eDrone_msgs::GeofenceCheck::Response &res)
{
    cout << " safety_node - geofenceCheck_cb():" << endl;
    cout << " geofence radius:" << geofence.geofence_radius;

    if (req.geofence_ref_system == "ENU")
    {
        double distance_to_home = sqrt ( pow ( (double) req.geofence_arg1, (double) 2) + pow ( (double) req.geofence_arg2 , (double) 2) );

        if (distance_to_home > geofence.geofence_radius)
        {
            res.violation = true;
            return true;
        }
    }
    else if (req.geofence_ref_system == "WGS84")
    {
        Point point = convertGeoToENU(req.geofence_arg1, req.geofence_arg2, req.geofence_arg3, HOME_LAT, HOME_LON, HOME_ALT );
        double distance_to_home = sqrt ( pow ( (double) point.x , (double) 2) + pow ( (double) point.y , (double) 2) );

        if (distance_to_home > geofence.geofence_radius)
        {
            res.violation = true;
            return true;
        }

    }

    res.violation = false;

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
    geofenceSet_srv_server = nh.advertiseService ("srv_geofenceSet", srv_geofenceSet_cb ); // (2019.04.23)geofenceSet service server initialization
    geofenceCheck_srv_server = nh.advertiseService ("srv_geofenceCheck", srv_geofenceCheck_cb );
    noflyZoneSet_srv_server = nh.advertiseService ( "srv_noflyZoneSet", srv_noflyZoneSet_cb );
    noflyZoneReset_srv_server = nh.advertiseService ( "srv_noflyZoneReset", srv_noflyZoneReset_cb );
    noflyZoneCheck_srv_server = nh.advertiseService ( "srv_noflyZoneCheck", srv_noflyZoneCheck_cb );

    // 서비스 클라이언트 초기화

    // 반복적으로 NoflyZones topic publish
    while ( ros::ok() ) // API 피 호출 시, 콜백 함수를 통해 해당 기능 처리 후 응답
    {
        // noflyZones topic publish
        noflyZones_pub.publish(nfZones);

        if (geofence.geofence_radius > 0)
        {
            geofence_pub.publish(geofence);
        }
        else
        {
            cout << "geofence radius was not initialized!!" << endl;
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
