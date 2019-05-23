
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <std_msgs/String.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mavlink/v2.0/common/mavlink.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/WaypointClear.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <eDrone_msgs/Arming.h> // 시동 서비스 헤더 파일 포함
#include <eDrone_msgs/Takeoff.h> // 이륙 서비스
#include <eDrone_msgs/Landing.h> // 착륙 서비스 	"
#include <eDrone_msgs/MissionAddItem.h> // 미션 아이템 추가 서비스 헤더 파일 포함
#include <eDrone_msgs/MissionUpload.h> // 미션 업로드 서비스 헤더 파일 포함
#include <eDrone_msgs/MissionDownload.h> // 미션 다운로드 서비스 헤더 파일 포함
#include <eDrone_msgs/MissionClear.h> // 미션 제거 서비스 헤더 파일 포함
#include <eDrone_msgs/CheckHome.h>  // 홈 위치 확인 서비스 헤더 파일 포함
#include <eDrone_msgs/Geofence.h> // 가상울타리 정보
#include <eDrone_msgs/GeofenceCheck.h> // 가상 울타리 확인
#include <eDrone_msgs/NoflyZoneSet.h> // 비행 금지 구역 설정
#include <eDrone_msgs/NoflyZoneReset.h> // 비행 금지 구역 해제
#include <eDrone_msgs/NoflyZoneCheck.h> // 비행 금지 구역 확인
#include <eDrone_lib/GeoUtils.h> // 좌표 변환 라이브러리 헤더 파일

using namespace std;
using namespace mavros_msgs;
using namespace geographic_msgs;
using namespace geometry_msgs;

mavros_msgs::HomePosition home_position; // Home 위치 획득에 필요한 메시지 변수
vector<mavros_msgs::Waypoint> waypoints; // 웨이포인트 정보
mavros_msgs::WaypointList waypointList; // 웨이포인트 목록
mavros_msgs::WaypointPush waypointPush_cmd; // 미션 업로드 요청 메시지
mavros_msgs::WaypointPull waypointPull_cmd; // 미션 다운로드 요청 메시지
mavros_msgs::WaypointClear waypointClear_cmd; // 미션 제거 요청 메시지
mavros_msgs::SetMode modeChange_cmd; //상태 변경 요청 메시지
eDrone_msgs::CheckHome checkHome_cmd; // home 위치 확인 서비스 요청 메시지
eDrone_msgs::Geofence geofence;

// subscriber 선언
ros::Subscriber wpList_sub;
ros::Subscriber home_sub;

// 서비스 서버 선언
ros::ServiceServer missionAddItem_srv_server;
ros::ServiceServer missionUpload_srv_server;
ros::ServiceServer missionDownload_srv_server;
ros::ServiceServer missionClear_srv_server;

// 서비스 클라이언트 선언
ros::ServiceClient waypointAdd_client;
ros::ServiceClient waypointPush_client;
ros::ServiceClient waypointPull_client;
ros::ServiceClient waypointClear_client;
ros::ServiceClient commandLong_client;
ros::ServiceClient modeChange_client; // 모드 변경 서비스 클라이언트
ros::ServiceClient checkHome_client; // home 위치 확인 서비스 클라이언트
ros::ServiceClient noflyZone_client; // 비행금지구역 확인 서비스 클라이언트

// Home 위치 변수
float HOME_LAT;
float HOME_LON;
float HOME_ALT;

void homePosition_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
    home_position = *msg;
    HOME_LAT = home_position.geo.latitude;
    HOME_LON = home_position.geo.longitude;
    HOME_ALT = home_position.geo.altitude;
}

void print_waypoints (vector<mavros_msgs::Waypoint> waypoints) // 웨이포인트 정보
{
    for (int i = 0; i < waypoints.size(); i++)
    {
        cout << "waypoint[" << i << "]: "<< endl;
        cout << waypoints[i] << endl;
    }
}

void wpList_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    waypointList = *msg;
}

bool srv_missionAddItem_cb(eDrone_msgs::MissionAddItem::Request &req, eDrone_msgs::MissionAddItem::Response &res)
{
    ros::NodeHandle nh; // (2019.05.23)

    printf("eDrone_autoflight_node: MissionAddItem request received\n");

    // 웨이포인트 추가
    mavros_msgs::Waypoint waypoint;
    waypoint = req.missionAddItem_waypoint;
    double distance_home;


    switch (waypoint.command)
    {
    case MAV_CMD_NAV_TAKEOFF:

        waypoint.x_lat = HOME_LAT;
        waypoint.y_long = HOME_LON;
        waypoint.z_alt = req.missionAddItem_waypoint.z_alt;
        break;

    case MAV_CMD_NAV_WAYPOINT:
        waypoint.frame = req.missionAddItem_waypoint.frame;
        waypoint.command = req.missionAddItem_waypoint.command;

        if (waypoint.frame == waypoint.FRAME_LOCAL_ENU) // 지역 좌표인 경우
        {
            GeoPoint geoPoint = convertENUToGeo( waypoint.x_lat, waypoint.y_long, waypoint.z_alt, HOME_LAT, HOME_LON, HOME_ALT);

            waypoint.x_lat = geoPoint.latitude;
            waypoint.y_long = geoPoint.longitude;
            waypoint.frame = waypoint.FRAME_GLOBAL_REL_ALT;
            waypoint.z_alt = req.missionAddItem_waypoint.z_alt;

            printf("missionAddItem_cb(): (lat: %lf, lon: %lf, alt: %lf) HOME_ALT:%lf \n", waypoint.x_lat, waypoint.y_long, waypoint.z_alt, HOME_ALT);

        }

        break;

    case MAV_CMD_NAV_LAND:

        waypoint.x_lat = HOME_LAT;
        waypoint.y_long = HOME_LON;
        break;

    case MAV_CMD_NAV_RETURN_TO_LAUNCH:
        break;

    default: // (2019.05.23) Default statement added
        break;
    }


    /* Geofence check */

    bool geofence_violation = false;

    eDrone_msgs::GeofenceCheck geofenceCheck_cmd;
    ros::ServiceClient geofenceCheck_client = nh.serviceClient<eDrone_msgs::GeofenceCheck> ("srv_geofenceCheck"); // geofence 확인 서비스 클라이언트

    geofenceCheck_cmd.request.geofence_ref_system = "WGS84";
    geofenceCheck_cmd.request.geofence_arg1= waypoint.x_lat;
    geofenceCheck_cmd.request.geofence_arg2= waypoint.y_long;

    printf("eDrone_autoflight_node: trying to call GeofenceCheck service");


    if (geofenceCheck_client.call (geofenceCheck_cmd) == true)
    {

        if (geofenceCheck_cmd.response.value == true )
        {
            if (geofenceCheck_cmd.response.violation == true)
            {
                geofence_violation = true;
                printf("eDrone_autoflight_node: missionAddItem service rejected: geofence violation!\n");
                res.value = false;
                return true;
            }
            else
            {
                geofence_violation = false;
            }
        }

    }
    else // (2019.04.30) 예외　처리
    {
        cout << "GeofenceCheck request failed!! " << endl;
        return false;
    }

    if (geofence_violation!=true)
    {
        waypoints.push_back(waypoint);
        printf("eDrone_autoflight_node: missionAddItem service accepted: new WP was added to the wp list.\n");
        res.value = true;
    }

    return true;
}


bool srv_missionUpload_cb(eDrone_msgs::MissionUpload::Request &req, eDrone_msgs::MissionUpload::Response &res)
{
    printf("MissionUpload request received\n");

    // 웨이포인트 업로드 메시지 설정
    waypointPush_cmd.request.start_index = 0;
    waypointPush_cmd.request.waypoints = waypoints;

    //// waypointPush

    printf("send WaypointPush command ...\n");

    if (!waypointPush_client.call(waypointPush_cmd))
    {
        printf("WaypointPush request failed!");
        return false;
    }

    printf("WaypointPush command was sent\n");

    print_waypoints(waypoints);
}

bool srv_missionDownload_cb(eDrone_msgs::MissionDownload::Request &req, eDrone_msgs::MissionDownload::Response &res)
{
    printf("MissionDownload request received\n");
    res.waypoints = waypointList.waypoints;
}


bool srv_missionClear_cb(eDrone_msgs::MissionClear::Request &req, eDrone_msgs::MissionClear::Response &res)
{
    printf("MIssionClear request received\n");
    waypoints.clear();

    if (!waypointClear_client.call(waypointClear_cmd))
    {
        printf("WaypointClear_client request failed!");
        return false;
    }

    printf("WaypointClear command was sent\n");

    return true;
}

int main(int argc, char** argv)

{
    ros::init(argc, argv, "eDrone_msgs");
    ros::NodeHandle nh;
    ros::Rate rate (20.0);

    // subscriber 초기화
    wpList_sub = nh.subscribe<mavros_msgs::WaypointList> ("mavros/mission/waypoints", 10, wpList_cb);
    home_sub = nh.subscribe<mavros_msgs::HomePosition> ("mavros/home_position/home", 10, homePosition_cb);

    // 서비스 서버 선언
    missionAddItem_srv_server = nh.advertiseService("srv_missionAddItem", srv_missionAddItem_cb);
    missionUpload_srv_server = nh.advertiseService("srv_missionUpload", srv_missionUpload_cb);
    missionDownload_srv_server = nh.advertiseService("srv_missionDownload", srv_missionDownload_cb);
    missionClear_srv_server = nh.advertiseService("srv_missionClear", srv_missionClear_cb);

    //// 서비스 클라이언트 초기화

    waypointPush_client = nh.serviceClient<mavros_msgs::WaypointPush> ("mavros/mission/push"); // 서비스 클라이언트 선언
    waypointPull_client = nh.serviceClient<mavros_msgs::WaypointPull> ("mavros/mission/pull"); // 서비스 클라이언트 선언
    waypointClear_client = nh.serviceClient<mavros_msgs::WaypointClear> ("mavros/mission/clear"); //
    modeChange_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");

    while ( ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
