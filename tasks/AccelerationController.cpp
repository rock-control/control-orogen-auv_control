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

bool AccelerationController::startHook()
{
    if (! AccelerationControllerBase::startHook())
        return false;

    thrusterMatrix = _matrix.get();
    inputVector = Eigen::VectorXd::Zero(6);
    cmdVector = Eigen::VectorXd::Zero(thrusterMatrix.cols());
    controlModes = _control_modes.get();
    //resize the controlModes to number of Thrustes
    unsigned int tmp_size = controlModes.size();
    controlModes.resize(thrusterMatrix.cols());
    //set undefined controlModes to RAW
    for(unsigned int i = tmp_size; i < thrusterMatrix.cols(); i++){
        controlModes[i] = base::JointState::RAW;
    }

    jointCommand = base::commands::Joints();
    jointCommand.elements.resize(thrusterMatrix.rows());
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
    cmdVector = thrusterMatrix.transpose() * inputVector;
    for (unsigned int i = 0; i < jointCommand.size(); ++i)
        jointCommand[i].setField(controlModes[i], cmdVector(i));
    _cmd_out.write(jointCommand);
    return true;
}
