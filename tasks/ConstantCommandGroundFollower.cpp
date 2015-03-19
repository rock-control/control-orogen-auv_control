/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ConstantCommandGroundFollower.hpp"

using namespace auv_control;

ConstantCommandGroundFollower::ConstantCommandGroundFollower(std::string const& name, TaskCore::TaskState initial_state)
    : ConstantCommandGroundFollowerBase(name, initial_state)
{
    last_valid_ground_position = 0;
}

ConstantCommandGroundFollower::ConstantCommandGroundFollower(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ConstantCommandGroundFollowerBase(name, engine, initial_state)
{
    last_valid_ground_position = 0;
}

ConstantCommandGroundFollower::~ConstantCommandGroundFollower()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ConstantCommandGroundFollower.hpp for more detailed
// documentation about them.

bool ConstantCommandGroundFollower::configureHook()
{
    if (! ConstantCommandGroundFollowerBase::configureHook())
        return false;
    return true;
}
bool ConstantCommandGroundFollower::startHook()
{
    if (! ConstantCommandGroundFollowerBase::startHook())
        return false;
    return true;
}
void ConstantCommandGroundFollower::updateHook()
{
    ConstantCommandGroundFollowerBase::updateHook();
    if(_altimeter.readNewest(altimeter) == RTT::NoData){
        if (state() != NO_ALTIMETER_READING){
            state(NO_ALTIMETER_READING);
        }
        return;
    }
    if(_depth.readNewest(depth) == RTT::NoData){
        if (state() != NO_DEPTH_READING){
            state(NO_DEPTH_READING);
        }
        return;
    }
    if(!depth.hasValidPosition(2)){
        if (state() != INVALID_DEPTH_READING){
            exception(INVALID_DEPTH_READING);
        }
        return;
    }
    if(altimeter.position[2] <= 0){
        exception(INVALID_NEGATIVE_ALTIMETER_READING);
        return;
    }

    base::LinearAngular6DCommand cmd;
    if(!_use_as_constant_task.get()){
        if(_cmd_in.readNewest(cmd) == RTT::NoData){
            return;
        }
    }else{
        cmd = _cmd.get();
        cmd.time = base::Time::now();
    }
    
    if(!isValidCmd(cmd.linear[2])){
        return exception(INVALID_TARGET_DEPTH_CONFIG);
    }
    
    if(altimeter.position[2] > 0.0f && !base::isUnset<double>(altimeter.position[2])){
        last_valid_ground_position = depth.position[2] - altimeter.position[2];
    }

    cmd.linear[2] = calculateDepth(cmd.linear[2]);

    if(state() != RUNNING){
        state(RUNNING);
    }
    _floor_position.write(last_valid_ground_position);
    _cmd_out.write(cmd);
}

double ConstantCommandGroundFollower::calculateDepth(double cmd_depth){
    return last_valid_ground_position + cmd_depth;
}

bool ConstantCommandGroundFollower::isValidCmd(double depth){
    return depth > 0;
}

void ConstantCommandGroundFollower::errorHook()
{
    ConstantCommandGroundFollowerBase::errorHook();
}
void ConstantCommandGroundFollower::stopHook()
{
    ConstantCommandGroundFollowerBase::stopHook();
}
void ConstantCommandGroundFollower::cleanupHook()
{
    ConstantCommandGroundFollowerBase::cleanupHook();
}
