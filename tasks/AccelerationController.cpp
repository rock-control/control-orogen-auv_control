/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AccelerationController.hpp"
#include <base/JointState.hpp>
#include <base/NamedVector.hpp>

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
    thrusterMatrix = _matrix.get();
    int numberOfThrusters = thrusterMatrix.cols();
    inputVector = Eigen::VectorXd::Zero(6);
    cmdVector = Eigen::VectorXd::Zero(numberOfThrusters);
    names = _names.get();

    // Calculates the SVD decomposition of the TCM
    svd.reset(new Eigen::JacobiSVD<Eigen::MatrixXd>(thrusterMatrix,  Eigen::ComputeThinU | Eigen::ComputeThinV));

    if(names.size() != numberOfThrusters && names.size() != 0){
        exception(WRONG_SIZE_OF_NAMES);
        return false;
    }
    
    if(_limits.get().size() != numberOfThrusters && !_limits.get().empty()){
        exception(WRONG_SIZE_OF_LIMITS);
        return false;
    } else {
        limits = _limits.get();
    }

    if(names.size() && limits.names.size()){
        for(unsigned i = 0; i < names.size(); i++){
            try{
                limits.mapNameToIndex(names[i]);
            } catch (std::exception& e){
                std::cout << "error:" << e.what() << std::endl;
                exception(INVALID_NAME_IN_LIMITS);
                return false;
            }
        }
    }

    controlModes = _control_modes.get();
    if(controlModes.size() == 0){
        //resize the controlModes to number of Thrusters
        controlModes.resize(numberOfThrusters);
        //set undefined controlModes to RAW
        for(unsigned int i = 0; i < numberOfThrusters; i++){
            controlModes[i] = base::JointState::RAW;
        }
    } else if(controlModes.size() != numberOfThrusters){
        exception(WRONG_SIZE_OF_CONTROLMODES);
    }

    jointCommand = base::commands::Joints();
    jointCommand.names = names;
    jointCommand.elements.resize(numberOfThrusters);
    
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

    if(_svd_calculation.get())
    {
    	/* The output vector is calculated through the SVD solution, which is
    	   similar to the pseudo-inverse solution */
    	cmdVector = svd->solve(inputVector);
    }
    else
    {
    	cmdVector = thrusterMatrix.transpose()*inputVector;
    }

    for (unsigned int i = 0; i < jointCommand.size(); ++i){
        //Cutoff cmdValue at the JointLimits
        if(!_limits.get().empty()){
            if(names.size() && _limits.get().hasNames()){
                if(cmdVector(i) > _limits.get()[names[i]].max.getField(controlModes[i])){
                    cmdVector(i) = _limits.get()[names[i]].max.getField(controlModes[i]);
                }
                if(cmdVector(i) < _limits.get()[names[i]].min.getField(controlModes[i])){
                    cmdVector(i) = _limits.get()[names[i]].min.getField(controlModes[i]);
                }
            } else {
                if(cmdVector(i) > _limits.get()[i].max.getField(controlModes[i])){
                    cmdVector(i) = _limits.get()[i].max.getField(controlModes[i]);
                }
                if(cmdVector(i) < _limits.get()[i].min.getField(controlModes[i])){
                    cmdVector(i) = _limits.get()[i].min.getField(controlModes[i]);
                }
            }
        }
        jointCommand[i].setField(controlModes[i], cmdVector(i));
        jointCommand.time = merged_command.time;
    }
    _cmd_out.write(jointCommand);
    return true;
}
