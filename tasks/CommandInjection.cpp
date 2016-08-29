/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CommandInjection.hpp"

using namespace auv_control;

CommandInjection::CommandInjection(std::string const& name)
    : CommandInjectionBase(name)
{
}

CommandInjection::CommandInjection(std::string const& name, RTT::ExecutionEngine* engine)
    : CommandInjectionBase(name, engine)
{
}

CommandInjection::~CommandInjection()
{
}

bool CommandInjection::calcOutput(const LinearAngular6DCommandStatus &merged_command)
{
    base::LinearAngular6DCommand output_command = merged_command.command;

    base::LinearAngular6DCommand cmd_injection;
    if(receiveCommandInjection(cmd_injection))
    {
        const ExpectedInputs& expected_inputs = _expected_inputs.value();
        for(unsigned i = 0; i < 3; i++)
        {
            if(!base::isNaN(cmd_injection.linear[i]) && expected_inputs.linear[i])
                output_command.linear[i] = cmd_injection.linear[i];
        }

        for(unsigned i = 0; i < 3; i++)
        {
            if(!base::isNaN(cmd_injection.angular[i]) && expected_inputs.angular[i])
                output_command.angular[i] = cmd_injection.angular[i];
        }
    }

    _cmd_out.write(output_command);
    return true;
}

bool CommandInjection::receiveCommandInjection(base::LinearAngular6DCommand& cmd_injection)
{
    RTT::FlowStatus status = _cmd_injection.readNewest(cmd_injection);
    if(status == RTT::OldData && (base::Time::now() - newest_injection_sample).toSeconds() <= _cmd_injection_timeout.value())
    {
        return true;
    }
    else if(status == RTT::NewData)
    {
        newest_injection_sample = base::Time::now();
        return true;
    }
    return false;
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CommandInjection.hpp for more detailed
// documentation about them.

bool CommandInjection::configureHook()
{
    if (! CommandInjectionBase::configureHook())
        return false;

    newest_injection_sample.microseconds = 0;

    return true;
}
bool CommandInjection::startHook()
{
    if (! CommandInjectionBase::startHook())
        return false;
    return true;
}
void CommandInjection::updateHook()
{
    CommandInjectionBase::updateHook();
}
void CommandInjection::errorHook()
{
    CommandInjectionBase::errorHook();
}
void CommandInjection::stopHook()
{
    CommandInjectionBase::stopHook();
}
void CommandInjection::cleanupHook()
{
    CommandInjectionBase::cleanupHook();
}
