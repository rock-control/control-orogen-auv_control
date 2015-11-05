/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CommandInjection.hpp"

using namespace auv_control;

CommandInjection::CommandInjection(std::string const& name, TaskCore::TaskState initial_state)
    : CommandInjectionBase(name, initial_state)
{
}

CommandInjection::CommandInjection(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : CommandInjectionBase(name, engine, initial_state)
{
}

CommandInjection::~CommandInjection()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See CommandInjection.hpp for more detailed
// documentation about them.

bool CommandInjection::configureHook()
{
    if (! CommandInjectionBase::configureHook())
        return false;
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

    base::LinearAngular6DCommand cmd;
    _cmd_cascade.readNewest(cmd);

    if(_cmd_injection.connected())
    {
        base::LinearAngular6DCommand cmd_injection;
        if(_cmd_injection.readNewest(cmd_injection) == RTT::NewData)
            newest_injection_sample = base::Time::now();

        if((base::Time::now() - newest_injection_sample).toSeconds() <= _cmd_injection_timeout.value())
        {
            for(unsigned i = 0; i < 3; i++)
            {
                if(!base::isNaN(cmd_injection.linear[i]))
                    cmd.linear[i] = cmd_injection.linear[i];
            }

            for(unsigned i = 0; i < 3; i++)
            {
                if(!base::isNaN(cmd_injection.angular[i]))
                    cmd.angular[i] = cmd_injection.angular[i];
            }
        }
    }

    _cmd_out.write(cmd);
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
