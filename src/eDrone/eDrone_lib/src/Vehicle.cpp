


#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <eDrone_lib/Vehicle.h>

using namespace Mission_API;
using namespace mavros_msgs;


Vehicle* Vehicle::instance = NULL;

Vehicle* Vehicle::getInstance()
{

	if (!instance)
	{
		instance = new Vehicle();

	}

	return instance;
}

/*
void Vehicle::setGlobalPosition(double lat, double lon, double alt)
{
  this->latitude = lat;
  this->longitude = lon;
  this->altitude = alt;
}

GeoPoint Vehicle::getGlobalPosition()
{
  GeoPoint globalPosition;
  globalPosition.latitude = latitude;  
  globalPosition.longitude = longitude;  
  globalPosition.altitude  = altitude; 

  return globalPosition; 
}

void Vehicle::setLocalPosition(double x, double y, double z)
{
  this->local_x = x;
  this->local_y = y;
  this->local_z = z;
  
}   

Point Vehicle::getLocalPosition()
{
  Point localPosition;
  localPosition.x = local_x;
  localPosition.y = local_y;
  localPosition.z = local_z;

  return localPosition;
}

void Vehicle::setHomePosition(GeoPoint home)
{
  this->homePosition = home;
}

GeoPoint Vehicle::getHomePosition()
{
  return this->homePosition;
}
*/

void Vehicle::setState(State newState)
{

  this->cur_state.armed = newState.armed;
  this->cur_state.connected = newState.connected;
  this->cur_state.flight_mode = newState.mode;
}

VehicleState Vehicle::getState()
{
  return this->cur_state;
}
  





