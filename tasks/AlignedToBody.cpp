/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AlignedToBody.hpp"

using namespace auv_control;

AlignedToBody::AlignedToBody(std::string const& name)
    : AlignedToBodyBase(name)
{
}

AlignedToBody::AlignedToBody(std::string const& name, RTT::ExecutionEngine* engine)
    : AlignedToBodyBase(name, engine)
{
}

AlignedToBody::~AlignedToBody()
{
}

static bool validateInputExpectations(bool* array, std::string const& type);

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AlignedToBody.hpp for more detailed
// documentation about them.

bool AlignedToBody::configureHook()
{
    if (! AlignedToBodyBase::configureHook())
        return false;
    return true;
}
bool AlignedToBody::startHook()
{
    if (! AlignedToBodyBase::startHook())
        return false;
    
    auv_control::ExpectedInputs expect(_expected_inputs.get());
    return validateInputExpectations(expect.linear, "linear") &&
        validateInputExpectations(expect.angular, "angular");
}
bool AlignedToBody::calcOutput()
{
    RTT::FlowStatus status = _orientation_samples.read(orientation_sample);

    if(status == RTT::NoData){
        if (state() != WAIT_FOR_ORIENTATION_SAMPLE){
            state(WAIT_FOR_ORIENTATION_SAMPLE);
        }
        return false;
    }
 
    base::LinearAngular6DCommand output_command = merged_command;

    double yaw = base::getYaw(orientation_sample.orientation);
    Eigen::Quaterniond orientation_pr =
        Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) *
        orientation_sample.orientation;
    output_command.linear = orientation_pr * output_command.linear;

    _cmd_out.write(output_command);
    return true;
}

void AlignedToBody::updateHook()
{
    AlignedToBodyBase::updateHook();
}
void AlignedToBody::errorHook()
{
    AlignedToBodyBase::errorHook();
}
void AlignedToBody::stopHook()
{
    AlignedToBodyBase::stopHook();
}
void AlignedToBody::cleanupHook()
{
    AlignedToBodyBase::cleanupHook();
}

static bool validateInputExpectations(bool* array, std::string const& type)
{
    bool any = false, all = true;
    for (int i = 0; i < 3; ++i)
    {
        any = any || array[i];
        all = all && array[i];
    }
    if (any && !all)
    {
        RTT::log(RTT::Error) << "the " << type << " part of the command must have either all or none of the axis set" << RTT::endlog();
        return false;
    }
    return true;
}
