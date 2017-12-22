

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "Vehicle.h"
#include "GeoInfo.h"

using namespace std;
using namespace Mission_API;

bool GeoInfo::instanceFlag = false;

GeoInfo GeoInfo::instance;

int GeoInfo::cnt = 0;

GeoInfo& GeoInfo::getInstance()
{
		instanceFlag = true;
		return instance;
	
/*
	if ( (GeoInfo::instance == NULL) && (GeoInfo::instanceFlag == false) )
	{
		GeoInfo::instance = new GeoInfo();
		GeoInfo::instanceFlag = true;

	}

	return instance;
*/
}


void GeoInfo::add()
{
	printf("cnt is %d \n", cnt);
	cnt++;
}

void GeoInfo::sub()
{
	printf("cnt is %d \n", cnt);
	cnt--;
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
