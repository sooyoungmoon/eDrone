

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <eDrone_lib/Vehicle.h>
#include <eDrone_lib/GeoInfo.h>

using namespace std;
using namespace eDrone;

GeoInfo* GeoInfo::instance = NULL;

GeoInfo* GeoInfo::getInstance()
{
    if (!instance)
    {
        instance = new GeoInfo();

    }

    return instance;
}

void GeoInfo::setLocalPosition(Point pos_local)
{
    position_local.x = pos_local.x;
    position_local.y = pos_local.y;
    position_local.z = pos_local.z;
}

Point GeoInfo::getLocalPosition()
{
    return position_local;
}

void GeoInfo::setGlobalPosition(GeoPoint pos_geo)
{
    position_geo.latitude = pos_geo.latitude;
    position_geo.longitude = pos_geo.longitude;
    position_geo.altitude = pos_geo.altitude;

}

GeoPoint GeoInfo::getGlobalPosition()
{
    return position_geo;
}

void GeoInfo::setHomePosition(GeoPoint homePos)
{
    homePosition.latitude = homePos.latitude;
    homePosition.longitude = homePos.longitude;
    homePosition.altitude  = homePos.altitude;

}

GeoPoint GeoInfo::getHomePosition()
{
    return homePosition;
}
