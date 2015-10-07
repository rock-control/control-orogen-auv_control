/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrustersInput.hpp"
#include "base/commands/Joints.hpp"
#include "base/Logging.hpp"

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

    thrusterVoltage		= _thruster_voltage.get();
    coeffPos 			= _thruster_coeff_pos.get();
    coeffNeg 			= _thruster_coeff_neg.get();
    controlModes		= _control_modes.get();

    numberOfThrusters = controlModes.size();

    // In case the control_modes vector was set
    if(numberOfThrusters != 0)
    {
        // Checks if the three arrays have the same length
        if(coeffPos.size() != numberOfThrusters || coeffNeg.size() != numberOfThrusters)
        {
            LOG_ERROR("The size of control_modes, thruster_coeff_pos and "
                    "thruster_coeff_neg do not agree (control_modes: %i, "
                    "thruster_coeff_pos: %i, thruster_coeff_neg: %i)",
                    numberOfThrusters, coeffPos.size(), coeffNeg.size());
            return false;
        }
    }
    // In case control_modes wasn't set, checks if the other two arrays have the same length
    else if(coeffPos.size() != coeffNeg.size())
    {
        LOG_ERROR("The size of control_modes, thruster_coeff_pos and "
                "thruster_coeff_neg do not agree (control_modes: %i, "
                "thruster_coeff_pos: %i, thruster_coeff_neg: %i)",
                numberOfThrusters, coeffPos.size(), coeffNeg.size());
        return false;
    }
    // In case control_modes wasn't set but the other two arrays have the same length
    else
    {
        LOG_WARN("The control_modes is being automatically set to RAW, "
                 "since it wasn't previously set.");
        numberOfThrusters = coeffPos.size();
        controlModes.resize(numberOfThrusters, base::JointState::RAW);
        _control_modes.set(controlModes);
    }

    for (uint i = 0; i < numberOfThrusters; i++)
    {
        // A different JointState other than SPEED and RAW was chosen
        if(controlModes[i] !=  base::JointState::SPEED && controlModes[i] !=  base::JointState::RAW)
        {
            LOG_ERROR("Control mode should be either SPEED or RAW.");
            return false;
        }
        // When using RAW control mode, the calculation demands that the thruster voltage
        // is set, and the latest can only assume positive values
        if(controlModes[i] ==  base::JointState::RAW && thrusterVoltage <= 0)
        {
            LOG_ERROR("The control mode was set to RAW, but no positive value was assigned "
                    "to thruster_voltage.");
            return false;
        }
        // The thruster coefficients can only assume positive values
        if (coeffPos[i] <= 0 || coeffNeg[i] <= 0)
        {
            LOG_ERROR("The thrusters coefficients (both positive and negative) should only "
                    "have positive values.");
            return false;
        }
    }

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

    if(_cmd_in.read(thrusterForces) == RTT::NewData)
    {
        if(checkControlInput(thrusterForces))
        {
            calcOutput(thrusterForces);
            _cmd_out.write(thrusterForces);
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

bool ThrustersInput::checkControlInput(base::samples::Joints &cmd_in)
{
    std::string textElement;

    // No match between input.size and the expected number of thrusters
    if(cmd_in.elements.size() != numberOfThrusters)
    {
        LOG_ERROR("The input vector should have a size equal to %i, but actually it "
                "has size equal to %i. Check configuration. ", numberOfThrusters, cmd_in.elements.size());
        exception(UNEXPECTED_THRUSTER_INPUT);
        return false;
    }

    for (uint i = 0; i < numberOfThrusters; i++)
    {
        // Verify if the EFFORT mode is a valid input or a nan
        if (!cmd_in.elements[i].hasEffort())
        {
            // Define how to call the problematic thruster
            std::string textThruster;

            // Check whether names were specified for the thrusters
            if(cmd_in.names.size() == numberOfThrusters)
                textThruster = cmd_in.names[i];
            else
            {
                std::stringstream number;
                number << i;
                textThruster = number.str();
            }

            LOG_ERROR("The field effort of the thruster %s was not set.", textThruster.c_str());
            exception(UNSET_THRUSTER_INPUT);
            return false;
        }

    }
    // In case of all inputs have valid values
    return true;
}

void ThrustersInput::calcOutput(base::samples::Joints &cmd_out)
{
    for (uint i = 0; i < numberOfThrusters; i++)
    {
        // Force = Cv * Speed * |Speed|
        if(controlModes[i] == base::JointState::SPEED)
        {
            if(cmd_out.elements[i].effort >= 0)
                cmd_out.elements[i].speed = + sqrt(fabs(cmd_out.elements[i].effort) / coeffPos[i]);
            else
                cmd_out.elements[i].speed = - sqrt(fabs(cmd_out.elements[i].effort) / coeffNeg[i]);
        }
        // Force = Cv * V * |V|
        // V = pwm * thrusterVoltage
        else if(controlModes[i] == base::JointState::RAW)
        {
            if(cmd_out.elements[i].effort >= 0)
                cmd_out.elements[i].raw = + sqrt(fabs(cmd_out.elements[i].effort / coeffPos[i])) / thrusterVoltage;
            else
                cmd_out.elements[i].raw = - sqrt(fabs(cmd_out.elements[i].effort / coeffNeg[i])) / thrusterVoltage;
        }
    }
}
