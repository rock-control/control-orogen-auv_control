/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ConstantCommandGroundAvoidance.hpp"

using namespace auv_control;

ConstantCommandGroundAvoidance::ConstantCommandGroundAvoidance(std::string const& name, TaskCore::TaskState initial_state)
    : ConstantCommandGroundAvoidanceBase(name, initial_state)
{
}

ConstantCommandGroundAvoidance::ConstantCommandGroundAvoidance(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ConstantCommandGroundAvoidanceBase(name, engine, initial_state)
{
}

ConstantCommandGroundAvoidance::~ConstantCommandGroundAvoidance()
{
}

double ConstantCommandGroundAvoidance::calculateDepth(double cmd_depth){
    double lowest_allowed_position =  last_valid_ground_position + _minimal_ground_distance.get();
    //Take care here lowest_allowed_position is negative
    if( lowest_allowed_position <  cmd_depth){
        //All Valid we are higher than the ground keeping distance
        return cmd_depth;
    }else{
        return lowest_allowed_position;
    }
}

//Assuming every depth command is valid, even positive ones.... Could be discussed
bool ConstantCommandGroundAvoidance::isValidCmd(double depth){
    return true;

}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ConstantCommandGroundAvoidance.hpp for more detailed
// documentation about them.

bool ConstantCommandGroundAvoidance::configureHook()
{
    if (! ConstantCommandGroundAvoidanceBase::configureHook())
        return false;
    return true;
}
bool ConstantCommandGroundAvoidance::startHook()
{
    if (! ConstantCommandGroundAvoidanceBase::startHook())
        return false;
    return true;
}
void ConstantCommandGroundAvoidance::updateHook()
{
    ConstantCommandGroundAvoidanceBase::updateHook();
}
void ConstantCommandGroundAvoidance::errorHook()
{
    ConstantCommandGroundAvoidanceBase::errorHook();
}
void ConstantCommandGroundAvoidance::stopHook()
{
    ConstantCommandGroundAvoidanceBase::stopHook();
}
void ConstantCommandGroundAvoidance::cleanupHook()
{
    ConstantCommandGroundAvoidanceBase::cleanupHook();
}
