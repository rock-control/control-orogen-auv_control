/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OptimalHeadingController.hpp"

using namespace auv_control;

OptimalHeadingController::OptimalHeadingController(std::string const& name)
    : OptimalHeadingControllerBase(name)
{
}

OptimalHeadingController::OptimalHeadingController(std::string const& name, RTT::ExecutionEngine* engine)
    : OptimalHeadingControllerBase(name, engine)
{
}

OptimalHeadingController::~OptimalHeadingController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OptimalHeadingController.hpp for more detailed
// documentation about them.

bool OptimalHeadingController::configureHook()
{
    if (! OptimalHeadingControllerBase::configureHook())
        return false;
    return true;
}
bool OptimalHeadingController::startHook()
{
    if (! OptimalHeadingControllerBase::startHook())
        return false;
    return true;
}
void OptimalHeadingController::updateHook()
{
    if (_orientation_samples.read(orientation_sample) == RTT::NoData){
        if(state() != WAIT_FOR_ORIENTATION_SAMPLE){
            error(WAIT_FOR_ORIENTATION_SAMPLE);
        }
        return;
    }
    
    OptimalHeadingControllerBase::updateHook();
}

void OptimalHeadingController::errorHook()
{
    if( state() == WAIT_FOR_ORIENTATION_SAMPLE){
        base::samples::RigidBodyState orientation_sample;
        if (_orientation_samples.read(orientation_sample) != RTT::NoData){
            recover();
        }
    }

    OptimalHeadingController::errorHook();
}

void OptimalHeadingController::keep(){
}

bool OptimalHeadingController::calcOutput(){
    base::LinearAngular6DCommand output_command;
    double opt_heading;
    double opt_distance;

    opt_heading = _optimal_heading.get();
    opt_distance = _optimal_heading_distance.get();

    output_command = merged_command;
    
    //Set z to 0, to use only x and y fpr the distance
    merged_command.linear(2) = 0;
    if(merged_command.linear.norm() > opt_distance){
        output_command.angular(2) = base::Angle::normalizeRad(atan2(merged_command.linear(1), merged_command.linear(0))
                //+base::getYaw(orientation_sample.orientation) 
                + opt_heading);
    }
    
    //write the command
    _cmd_out.write(output_command);

    return true;
}
