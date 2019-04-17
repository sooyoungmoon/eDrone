


#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <eDrone_lib/Vehicle.h>

using namespace eDrone;
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






