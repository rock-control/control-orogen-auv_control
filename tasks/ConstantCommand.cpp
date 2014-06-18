/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ConstantCommand.hpp"

using namespace auv_control;

ConstantCommand::ConstantCommand(std::string const& name, TaskCore::TaskState initial_state)
    : ConstantCommandBase(name, initial_state)
{
}

ConstantCommand::ConstantCommand(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : ConstantCommandBase(name, engine, initial_state)
{
}

ConstantCommand::~ConstantCommand()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ConstantCommand.hpp for more detailed
// documentation about them.

bool ConstantCommand::configureHook()
{
    if (! ConstantCommandBase::configureHook())
        return false;
    return true;
}
bool ConstantCommand::startHook()
{
    if (! ConstantCommandBase::startHook())
        return false;
    return true;
}
void ConstantCommand::updateHook()
{
    base::LinearAngular6DCommand cmd = _cmd.get();
    cmd.time = base::Time::now();
    _cmd_out.write(cmd);
    ConstantCommandBase::updateHook();
}
void ConstantCommand::errorHook()
{
    ConstantCommandBase::errorHook();
}
void ConstantCommand::stopHook()
{
    ConstantCommandBase::stopHook();
}
void ConstantCommand::cleanupHook()
{
    ConstantCommandBase::cleanupHook();
}
