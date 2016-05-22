/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrustersFeedback.hpp"
#include "base/commands/Joints.hpp"

using namespace auv_control;

ThrustersFeedback::ThrustersFeedback(std::string const& name)
    : ThrustersFeedbackBase(name)
{
}

ThrustersFeedback::ThrustersFeedback(std::string const& name, RTT::ExecutionEngine* engine)
    : ThrustersFeedbackBase(name, engine)
{
}

ThrustersFeedback::~ThrustersFeedback()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ThrustersFeedback.hpp for more detailed
// documentation about them.

bool ThrustersFeedback::configureHook()
{
    if (! ThrustersFeedbackBase::configureHook())
        return false;
    return true;
}
bool ThrustersFeedback::startHook()
{
    if (! ThrustersFeedbackBase::startHook())
        return false;
    return true;
}
void ThrustersFeedback::updateHook()
{
    base::commands::Joints joint_samples;
    if(_joint_samples_in.read(joint_samples) == RTT::NewData)
    {
        if(ThrustersInput::checkControlInput(joint_samples, controlModes[0]))
        {
            base::commands::Joints output = calcOutput(joint_samples);
            _forces_out.write(output);
        }
    }
}
void ThrustersFeedback::errorHook()
{
    ThrustersFeedbackBase::errorHook();
}
void ThrustersFeedback::stopHook()
{
    ThrustersFeedbackBase::stopHook();
}
void ThrustersFeedback::cleanupHook()
{
    ThrustersFeedbackBase::cleanupHook();
}

base::samples::Joints ThrustersFeedback::calcOutput(base::samples::Joints const &joint_samples) const
{
    base::samples::Joints forces;
    forces.elements.resize(numberOfThrusters);
    base::VectorXd feedback = base::VectorXd::Zero(numberOfThrusters);

    for (uint i = 0; i < numberOfThrusters; i++)
    {
        // Force = Cv * Speed * |Speed|
        if(controlModes[i] == base::JointState::SPEED)
        {
            feedback[i] = joint_samples.elements[i].speed;
            if(joint_samples.elements[i].speed >= 0)
                forces.elements[i].effort = coeffPos[i] * feedback[i] * fabs(feedback[i]);
            else
                forces.elements[i].effort = coeffNeg[i] * feedback[i] * fabs(feedback[i]);
        }
        // Force = Cv * V * |V|
        // V = pwm * thrusterVoltage
        else if(controlModes[i] == base::JointState::RAW)
        {
            feedback[i] = joint_samples.elements[i].raw * thrusterVoltage;
            if(joint_samples.elements[i].speed >= 0)
                forces.elements[i].effort = coeffPos[i] * feedback[i] * fabs(feedback[i]);
            else
                forces.elements[i].effort = coeffNeg[i] * feedback[i] * fabs(feedback[i]);
        }
    }

    forces.time = joint_samples.time;
    forces.names = joint_samples.names;

    return forces;
}
