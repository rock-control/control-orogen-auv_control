/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "GroundFollower.hpp"
#include <iostream>

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
    return true;
}
bool GroundFollower::startHook()
{
    if (! GroundFollowerBase::startHook())
        return false;

    last_valid_ground_position = 0.0;
    return true;
}
void GroundFollower::updateHook()
{
    GroundFollowerBase::updateHook();

    RTT::FlowStatus altimeter_status = _altimeter.readNewest(altimeter);
    RTT::FlowStatus depth_status = _depth.readNewest(depth);

    if(altimeter_status == RTT::NoData){
        return state(NO_ALTIMETER_READING);
    }

    if(depth_status == RTT::NoData){
        return state(NO_DEPTH_READING);
    }
    if(!depth.hasValidPosition(2)){
        return exception(INVALID_DEPTH_READING);
    }
    if(altimeter.position[2] <= 0){
        return exception(INVALID_NEGATIVE_ALTIMETER_READING);
    }

    double distance_to_ground_cmd = _distance_to_ground.get();
    if(distance_to_ground_cmd <= 0){
        return exception(INVALID_TARGET_DEPTH_CONFIG);
    }

    base::LinearAngular6DCommand cmd;
    cmd.time = base::Time::now();


    if(altimeter.position[2] > 0.0f && !base::isUnset<double>(altimeter.position[2])){
        last_valid_ground_position = depth.position[2] - altimeter.position[2];
        if(state() != RUNNING)
            state(RUNNING);
    }
    else {
        if(state() != ALTIMETER_DROPOUT)
            state(ALTIMETER_DROPOUT);
    }


    if(last_valid_ground_position >= 0){
        // last valid distance is not initialized
        cmd.linear[2] = 0.0;
    }
    else if(depth.position[2] - last_valid_ground_position < _safety_distance.get()){
        cmd.linear[2] = 0.0;
        state(WARNING_LOW_ALTITUDE);
    }
    else{
        cmd.linear[2] = last_valid_ground_position + distance_to_ground_cmd;
    }


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
