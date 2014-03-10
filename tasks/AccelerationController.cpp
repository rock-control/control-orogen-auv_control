/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AccelerationController.hpp"
#include <base/JointState.hpp>

using namespace auv_control;

AccelerationController::AccelerationController(std::string const& name)
    : AccelerationControllerBase(name)
{
}

AccelerationController::AccelerationController(std::string const& name, RTT::ExecutionEngine* engine)
    : AccelerationControllerBase(name, engine)
{
}

AccelerationController::~AccelerationController()
{
}

bool AccelerationController::configureHook()
{
    if (! AccelerationControllerBase::configureHook()){
        return false;
    }
    thrusterMatrix = _matrix.get().transpose();
    inputVector = Eigen::VectorXd::Zero(6);
    cmdVector = Eigen::VectorXd::Zero(thrusterMatrix.rows());
    names = _names.get();
    if(names.size() != thrusterMatrix.rows() && names.size() != 0){
        exception(WRONG_SIZE_OF_NAMES);
    }
    
    if(_limits.get().size() != thrusterMatrix.rows() && !_limits.get().empty()){
        exception(WRONG_SIZE_OF_LIMITS);
    }

    controlModes = _control_modes.get();
    if(controlModes.size() == 0){
        //resize the controlModes to number of Thrustes
        controlModes.resize(thrusterMatrix.rows());
        //set undefined controlModes to RAW
        for(unsigned int i = 0; i < thrusterMatrix.rows(); i++){
            controlModes[i] = base::JointState::RAW;
        }
    } else if(controlModes.size() != thrusterMatrix.rows()){
        exception(WRONG_SIZE_OF_CONTROLMODES);
    }

    jointCommand = base::commands::Joints();
    jointCommand.names = names;
    jointCommand.elements.resize(thrusterMatrix.rows());
    
    return true;
}

bool AccelerationController::startHook()
{
    if (! AccelerationControllerBase::startHook())
        return false;

    return true;
}
bool AccelerationController::calcOutput()
{
    inputVector << merged_command.linear, merged_command.angular;
    for (int i = 0; i < 6; ++i)
    {
        if (base::isUnset(inputVector(i)))
            inputVector(i) = 0;
    }
    cmdVector = thrusterMatrix * inputVector;
    for (unsigned int i = 0; i < jointCommand.size(); ++i){
        
        //Cutoff cmdValue at the JointLimits
	if(!_limits.get().empty()){
            if(cmdVector(i) > _limits.get()[i].max.getField(controlModes[i])){
	        cmdVector(i) = _limits.get()[i].max.getField(controlModes[i]);
	    }
            if(cmdVector(i) < _limits.get()[i].min.getField(controlModes[i])){
	        cmdVector(i) = _limits.get()[i].min.getField(controlModes[i]);
	    }
        }
	
	jointCommand[i].setField(controlModes[i], cmdVector(i));
    }
    _cmd_out.write(jointCommand);
    return true;
}
