/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BasePIDController.hpp"

using namespace auv_control;

BasePIDController::BasePIDController(std::string const& name, TaskCore::TaskState initial_state)
    : BasePIDControllerBase(name, initial_state)
{
}

BasePIDController::BasePIDController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : BasePIDControllerBase(name, engine, initial_state)
{
}

BasePIDController::~BasePIDController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BasePIDController.hpp for more detailed
// documentation about them.

bool BasePIDController::configureHook()
{
    if (! BasePIDControllerBase::configureHook())
        return false;

    for (int i = 0; i < 3; ++i)
    {
        mLinearPIDs[i].setPIDSettings(_pid_settings.get().linear[i]);
        mAngularPIDs[i].setPIDSettings(_pid_settings.get().angular[i]);
    }

    return true;
}
bool BasePIDController::startHook()
{
    if (! BasePIDControllerBase::startHook())
        return false;
    return true;
}
void BasePIDController::updateHook()
{
    BasePIDControllerBase::updateHook();
}
void BasePIDController::errorHook()
{
    BasePIDControllerBase::errorHook();
}
void BasePIDController::stopHook()
{
    BasePIDControllerBase::stopHook();
}
void BasePIDController::cleanupHook()
{
    BasePIDControllerBase::cleanupHook();
}
bool BasePIDController::calcOutput()
{
    // We start by copying merged_command so that we can simply ignore the unset
    // values
    LinearAngular6DCommand output_command = merged_command;
    for (int i = 0; i < 3; ++i)
    {
        if (!base::isUnset(merged_command.linear(i)))
        {
            output_command.linear(i) =
                mLinearPIDs.update(currentLinear(i),
                                   merged_command.linear(i),
                                   merged_command.time.toSeconds());
        }
        if (!base::isUnset(merged_command.angular(i)))
        {
            output_command.angular(i) =
                mAngularPIDs.update(currentAngular(i),
                                   merged_command.angular(i),
                                   merged_command.time.toSeconds());
        }
    }
    _cmd_out.write(output_command);
    return true;
}

