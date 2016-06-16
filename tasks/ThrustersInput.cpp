/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrustersInput.hpp"
#include "base/commands/Joints.hpp"

using namespace auv_control;

ThrustersInput::ThrustersInput(std::string const& name)
    : ThrustersInputBase(name)
{
}

ThrustersInput::ThrustersInput(std::string const& name, RTT::ExecutionEngine* engine)
    : ThrustersInputBase(name, engine)
{
}

ThrustersInput::~ThrustersInput()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ThrustersInput.hpp for more detailed
// documentation about them.

bool ThrustersInput::configureHook()
{
    if (! ThrustersInputBase::configureHook())
        return false;
    return true;
}

bool ThrustersInput::startHook()
{
    if (! ThrustersInputBase::startHook())
        return false;
    return true;
}
void ThrustersInput::updateHook()
{
    ThrustersInputBase::updateHook();

    base::commands::Joints thrusterForces;
    while(_cmd_in.read(thrusterForces) == RTT::NewData)
    {
        if(ThrustersBase::checkControlInput(thrusterForces, base::JointState::EFFORT))
        {
            base::commands::Joints thrusterCommands = calcOutput(thrusterForces);
            _cmd_out.write(thrusterCommands);
        }
    }
}
void ThrustersInput::errorHook()
{
    ThrustersInputBase::errorHook();
}
void ThrustersInput::stopHook()
{
    ThrustersInputBase::stopHook();
}
void ThrustersInput::cleanupHook()
{
    ThrustersInputBase::cleanupHook();
}

base::samples::Joints ThrustersInput::calcOutput(base::samples::Joints const &thrusterForces) const
{
    base::samples::Joints thrusterCommands;
    thrusterCommands.elements.resize(numberOfThrusters);
    for (uint i = 0; i < numberOfThrusters; i++)
    {
        // Force = Cv * Speed * |Speed|
        if(controlModes[i] == base::JointState::SPEED)
        {
            if(thrusterForces.elements[i].effort >= 0)
                thrusterCommands.elements[i].speed = + sqrt(fabs(thrusterForces.elements[i].effort) / coeffPos[i]);
            else
                thrusterCommands.elements[i].speed = - sqrt(fabs(thrusterForces.elements[i].effort) / coeffNeg[i]);
        }
        // Force = Cv * V * |V|
        // V = pwm * thrusterVoltage
        else if(controlModes[i] == base::JointState::RAW)
        {
            if(thrusterForces.elements[i].effort >= 0)
                thrusterCommands.elements[i].raw = + sqrt(fabs(thrusterForces.elements[i].effort / coeffPos[i])) / thrusterVoltage;
            else
                thrusterCommands.elements[i].raw = - sqrt(fabs(thrusterForces.elements[i].effort / coeffNeg[i])) / thrusterVoltage;
        }
    }
    thrusterCommands.time = thrusterForces.time;
    thrusterCommands.names = thrusterForces.names;
    return thrusterCommands;
}

