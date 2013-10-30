/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AccelerationController.hpp"

using namespace auv_control;

AccelerationController::AccelerationController(std::string const& name, TaskCore::TaskState initial_state)
    : AccelerationControllerBase(name, initial_state)
{
}

AccelerationController::AccelerationController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AccelerationControllerBase(name, engine, initial_state)
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
    cmdVector = Eigen::VectorXd::Zero(thrusterMatrix.rows());
    controlModes = _control_modes.get();

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
    cmdVector = thrusterMatrix * inputVector;
    for (unsigned int i = 0; i < jointCommand.size(); ++i)
        jointCommand[i].setField(controlModes[i], cmdVector(i));
    _cmd_out.write(jointCommand);
    return true;
}

