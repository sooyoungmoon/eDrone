
#ifndef ROS_TYPES
#define ROS_TYPES
#include <iostream>

// 파라미터 초기값 

const std::string GEOFENCECHECK_REF_SYSTEM = "WGS84";


const double IS_GLOBAL = false;

const double NOFLY_ZONE_LAT_MIN = 47.3982540;
const double NOFLY_ZONE_LAT_MAX = 47.3986305;
const double NOFLY_ZONE_LON_MIN = 8.5467605;
const double NOFLY_ZONE_LON_MAX = 8.5476337;

const double TAKEOFF_1_ALTITUDE = 20;

const double GOTO_1_X_LAT = 0;
const double GOTO_1_Y_LONG = 100;
const double GOTO_1_Z_ALT = 50;

const double GOTO_2_X_LAT = 100;
const double GOTO_2_Y_LONG = 100;
const double GOTO_2_Z_ALT = 50;

const double GOTO_3_X_LAT = 100;
const double GOTO_3_Y_LONG = 0;
const double GOTO_3_Z_ALT = 50;

const double STARTING_POS_X_LAT = 20;
const double STARTING_POS_Y_LON = 20;

const double WP_1_X_LAT= 20;
const double WP_1_Y_LON = 120;

const double WP_2_X_LAT = 70;
const double WP_2_Y_LON = 120;

const double WP_3_X_LAT = 70;
const double WP_3_Y_LON = 20;

const double WP_4_X_LAT = 120;
const double WP_4_Y_LON = 20;

const double WP_5_X_LAT = 120;
const double WP_5_Y_LON = 120;
#endif

