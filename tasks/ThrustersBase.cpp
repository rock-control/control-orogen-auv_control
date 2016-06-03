/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrustersBase.hpp"
#include "base/Logging.hpp"

using namespace auv_control;

ThrustersBase::ThrustersBase(std::string const& name)
    : ThrustersBaseBase(name)
{
}

ThrustersBase::ThrustersBase(std::string const& name, RTT::ExecutionEngine* engine)
    : ThrustersBaseBase(name, engine)
{
}

ThrustersBase::~ThrustersBase()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ThrustersBase.hpp for more detailed
// documentation about them.

bool ThrustersBase::configureHook()
{
    if (! ThrustersBaseBase::configureHook())
        return false;

    thrusterVoltage     = _thruster_voltage.get();
    coeffPos            = _thruster_coeff_pos.get();
    coeffNeg            = _thruster_coeff_neg.get();
    controlModes        = _control_modes.get();
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

bool ThrustersBase::startHook()
{
    if (! ThrustersBaseBase::startHook())
        return false;
    return true;
}
void ThrustersBase::updateHook()
{
    ThrustersBaseBase::updateHook();
}
void ThrustersBase::errorHook()
{
    ThrustersBaseBase::errorHook();
}
void ThrustersBase::stopHook()
{
    ThrustersBaseBase::stopHook();
}
void ThrustersBase::cleanupHook()
{
    ThrustersBaseBase::cleanupHook();
}
bool ThrustersBase::checkControlInput(base::samples::Joints const &cmd_in, base::JointState::MODE mode)
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
        // Verify if the desired mode field is a NaN
        if (base::isNaN(cmd_in.elements[i].getField(mode)))
        {
            // Define how to call the problematic thruster
            std::string textMode;
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

            if(mode == base::JointState::SPEED)
                textMode = "SPEED";
            else if(mode == base::JointState::RAW)
                textMode = "RAW";
            else if(mode == base::JointState::EFFORT)
                textMode = "EFFORT";
            else
                textMode = "unsupported mode";

            LOG_ERROR("The field %s of the thruster %s was not set.", textMode.c_str(), textThruster.c_str());
            exception(UNSET_THRUSTER_INPUT);
            return false;
        }
    }
    return true;
}
