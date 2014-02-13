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
    controlModes = _control_modes.get();
    if(controlModes.size() == 0){
        //resize the controlModes to number of Thrustes
        unsigned int tmp_size = controlModes.size();
        controlModes.resize(thrusterMatrix.rows());
        //set undefined controlModes to RAW
        for(unsigned int i = tmp_size; i < thrusterMatrix.rows(); i++){
            controlModes[i] = base::JointState::RAW;
        }
    } else if(controlModes.size() != thrusterMatrix.rows()){
        exception(WRONG_SIZE_OF_CONTROLMODES);
    }

    jointCommand = base::commands::Joints();
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
        jointCommand[i].setField(controlModes[i], cmdVector(i));
    }
    _cmd_out.write(jointCommand);
    return true;
}
