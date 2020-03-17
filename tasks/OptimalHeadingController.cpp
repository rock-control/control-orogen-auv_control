/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OptimalHeadingController.hpp"
#include <base/Angle.hpp>

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

    new_orientation_samples_timeout = base::Timeout(_timeout_in.value());

    return true;
}
bool OptimalHeadingController::startHook()
{
    if (! OptimalHeadingControllerBase::startHook())
        return false;

    new_orientation_samples_timeout.restart();

    return true;
}
void OptimalHeadingController::updateHook()
{
    RTT::FlowStatus status = _orientation_samples.readNewest(orientation_sample);
    if (status == RTT::NoData){
        if(state() != WAIT_FOR_ORIENTATION_SAMPLE){
            error(WAIT_FOR_ORIENTATION_SAMPLE);
        }
        return;
    }
    else if (status == RTT::OldData && new_orientation_samples_timeout.elapsed()){
        if (state() != WAIT_FOR_ORIENTATION_SAMPLE){
            error(WAIT_FOR_ORIENTATION_SAMPLE);
        }
        return;
    }
    else{
        new_orientation_samples_timeout.restart();
    }

    OptimalHeadingControllerBase::updateHook();
}

void OptimalHeadingController::errorHook()
{
    if( state() == WAIT_FOR_ORIENTATION_SAMPLE){
        base::samples::RigidBodyState orientation_sample;
        if (_orientation_samples.readNewest(orientation_sample) == RTT::NewData){
            recover();
        }
    }

    OptimalHeadingControllerBase::errorHook();
}

void OptimalHeadingController::keep(){
}

bool OptimalHeadingController::calcOutput(const LinearAngular6DCommandStatus &merged_command){
    base::LinearAngular6DCommand output_command;
    double opt_heading;
    double opt_distance;

    opt_heading = _optimal_heading.get();
    opt_distance = _optimal_heading_distance.get();

    output_command = merged_command.command;

    // distance on the xy plane
    if(merged_command.command.linear.head<2>().norm() > opt_distance){
        output_command.angular(2) = base::Angle::normalizeRad(atan2(merged_command.command.linear(1), merged_command.command.linear(0))
                + opt_heading);
    }

    //write the command
    _cmd_out.write(output_command);

    return true;
}
