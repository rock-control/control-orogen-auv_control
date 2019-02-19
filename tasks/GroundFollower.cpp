/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GroundFollower.hpp"
//#include <math.h>

using namespace auv_control;

GroundFollower::GroundFollower(std::string const& name, TaskCore::TaskState initial_state)
    : GroundFollowerBase(name, initial_state)
{
}

GroundFollower::GroundFollower(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : GroundFollowerBase(name, engine, initial_state)
{
}

GroundFollower::~GroundFollower()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See GroundFollower.hpp for more detailed
// documentation about them.

bool GroundFollower::configureHook()
{
    if (! GroundFollowerBase::configureHook())
        return false;

    new_altimeter_timeout = base::Timeout(_altimeter_timeout.get());
    new_depth_timeout = base::Timeout(_depth_timeout.get());
    altimeter_dropout_timeout = base::Timeout(_altimeter_dropout_timeout.get());

    last_valid_ground_position = NAN;

    distance_to_ground_cmd = _distance_to_ground.get();
    if(distance_to_ground_cmd <= 0){
        return false;
    }
    return true;
}
bool GroundFollower::startHook()
{
    if (! GroundFollowerBase::startHook())
        return false;

    new_altimeter_timeout.restart();
    new_depth_timeout.restart();
    altimeter_dropout_timeout.restart();
    return true;
}
void GroundFollower::updateHook()
{
    GroundFollowerBase::updateHook();

    RTT::FlowStatus altimeter_status = _altimeter.readNewest(altimeter);
    RTT::FlowStatus depth_status = _depth.readNewest(depth);

    if(altimeter_status == RTT::NoData){ // if no data recieved at all, wait for timeout then go in exception
        if (state() != NO_ALTIMETER_READING)
            state(NO_ALTIMETER_READING);
        if(new_altimeter_timeout.elapsed())
            exception(ALTIMETER_TIMEOUT);
        return;
    }
    else if (altimeter_status == RTT::OldData) { // if no new data received, write out command using the old data, then go to exception after timeout
        if(new_altimeter_timeout.elapsed()) {
            exception(ALTIMETER_TIMEOUT);
            return;
            }
    }
    else
        new_altimeter_timeout.restart();

    if(depth_status == RTT::NoData){
        if (state() != NO_DEPTH_READING)
            state(NO_DEPTH_READING);
        if(new_depth_timeout.elapsed())
            exception(DEPTH_TIMEOUT);
        return;
    }
    else if (depth_status == RTT::OldData) {
        if(new_depth_timeout.elapsed()) {
            exception(DEPTH_TIMEOUT);
            return;
            }
    }
    else
        new_depth_timeout.restart();

    if(!depth.hasValidPosition(2)){
        exception(INVALID_DEPTH_READING);
        return;
    }
    if(altimeter.position[2] <= 0){
        exception(INVALID_NEGATIVE_ALTIMETER_READING);
        return;
    }

    base::LinearAngular6DCommand cmd;
    cmd.time = base::Time::now();


    if(altimeter.position[2] > 0.0f && !base::isUnset<double>(altimeter.position[2])){
        last_valid_ground_position = depth.position[2] - altimeter.position[2];
        if(state() != RUNNING)
            state(RUNNING);
        altimeter_dropout_timeout.restart();
    }
    else { //a dropout can be a sample with nan
        if(state() != ALTIMETER_DROPOUT)
            state(ALTIMETER_DROPOUT);
        if(altimeter_dropout_timeout.elapsed())
            exception(ALTIMETER_DROPOUT_TIMEOUT);
            return;
    }


    if(base::isUnset<double>(last_valid_ground_position)){
        // last valid distance is not initialized
        state(NO_VALID_GROUND_DISTANCE);
        return;
    }
    else if(depth.position[2] - last_valid_ground_position < _safety_distance.get()){
        state(WARNING_LOW_ALTITUDE);
    }

    cmd.linear[2] = last_valid_ground_position + distance_to_ground_cmd;

    _floor_position.write(last_valid_ground_position);
    _cmd_out.write(cmd);
}
void GroundFollower::errorHook()
{
    GroundFollowerBase::errorHook();
}
void GroundFollower::stopHook()
{
    GroundFollowerBase::stopHook();
}
void GroundFollower::cleanupHook()
{
    GroundFollowerBase::cleanupHook();
}
