/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrustersInput.hpp"

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

    // Let's assume control_modes was set with the right number of thrusters
    numberOfThrusters = controlModes.size();

    // Those coefficients need to be set
    if(coeffPos.size() == 0 || coeffNeg.size() == 0)
    {
        RTT::log(RTT::Error) << "Thruster coefficients were not set" << RTT::endlog();
        exception(WRONG_SIZE_OF_THRUSTER_COEFFICIENTS);
    }


    // Check size of parameters. They should met among them.
    if(coeffPos.size() != numberOfThrusters || coeffNeg.size() != numberOfThrusters )
    {
        if(coeffPos.size() != coeffNeg.size())
        {	// If coefficients sizes are different among them and controm_modes size, one of the coeff were not set right
            RTT::log(RTT::Error) << "Coefficients were not set with "<< numberOfThrusters <<" as predicted" << RTT::endlog();
            exception(WRONG_SIZE_OF_THRUSTER_COEFFICIENTS);
        }
        else if(!controlModes.empty())
        {
            // Well, if the thruster_coeff vectors have the same size, looks like the control_modes is the one with wrong size
            exception(WRONG_SIZE_OF_CONTROLMODES);
        }
        else if(controlModes.empty())
        {
            // In this case, control_modes is empty and coeffPos has the right number of thrusters
            // Let's set control_modes with default values RAW
            numberOfThrusters = coeffPos.size();
            //resize the controlModes to number of Thrusters
            controlModes.resize(numberOfThrusters);
            //set undefined controlModes to RAW
            for(uint i = 0; i < numberOfThrusters; i++)
            {
                controlModes[i] = base::JointState::RAW;
            }
        }
    }

    // Check control_modes
    for (uint i=0; i < numberOfThrusters; i++)
    {
        if(controlModes[i] !=  base::JointState::SPEED && controlModes[i] !=  base::JointState::RAW)
        {
            RTT::log(RTT::Error) << "Control Mode should be SPEED or RAW" << RTT::endlog();
            exception(WRONG_SET_OF_CONTROLMODES);
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

    while (_cmd_in.read(thrusterForces) == RTT::NewData)
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

    // No match of input.size with properties.size
    if(cmd_in.elements.size() != numberOfThrusters)
    {
        RTT::log(RTT::Error) << "The input has not "<< numberOfThrusters <<
                " thruster as predicted. Actual size is "<< cmd_in.elements.size() << ". Check configuration. "<< RTT::endlog();
        exception(UNEXPECTED_THRUSTER_INPUT);
        return false;
    }

    for (uint i = 0; i < numberOfThrusters; i++)
    {
        // Define how to call the problematic thruster
        std::string textThruster;
        if(cmd_in.size() == numberOfThrusters)
            textThruster = cmd_in.names[i];
        else
        {
            std::stringstream number;
            number << i;
            textThruster = number.str();
        }

        // Verify if the EFFORT mode is a valid input or a nan
        if (!cmd_in.elements[i].hasEffort())
        {
            textElement = "effort";
            RTT::log(RTT::Error) << "The field "<< textElement << " of thruster "<<
                    textThruster << " was not set." << RTT::endlog();
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
                cmd_out.elements[i].speed = sqrt(fabs(cmd_out.elements[i].effort) / coeffPos[i]);
            else
                cmd_out.elements[i].speed = sqrt(fabs(cmd_out.elements[i].effort) / coeffNeg[i]) * -1.0;
        }
        // Force = Cv * V * |V|
        // V = pwm * thrusterVoltage
        else if(controlModes[i] == base::JointState::RAW)
        {
            if(cmd_out.elements[i].effort >= 0)
                cmd_out.elements[i].raw = sqrt(fabs(cmd_out.elements[i].effort / coeffPos[i])) / thrusterVoltage;
            else
                cmd_out.elements[i].raw = sqrt(fabs(cmd_out.elements[i].effort / coeffNeg[i])) * -1.0 / thrusterVoltage;
        }
    }
}
