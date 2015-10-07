/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AccelerationController.hpp"
#include <base/JointState.hpp>
#include <base/NamedVector.hpp>
#include <base/Logging.hpp>

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
    unsigned int numberOfThrusters = thrusterMatrix.cols();
    inputVector = Eigen::VectorXd::Zero(6);
    cmdVector = Eigen::VectorXd::Zero(numberOfThrusters);
    expectedEffortVector = Eigen::VectorXd::Zero(6);
    names = _names.get();
    base::MatrixXd weighingMatrix;
    base::VectorXd thrustersWeights = _thrusters_weights.get();

    if (thrustersWeights.size() != 0 && _svd_calculation == false)
    {
        LOG_ERROR("The thrusters weights vector should be empty if svd_calculation is FALSE.");
        return false;
    }
    else if(thrustersWeights.size() == 0 && _svd_calculation == true)
    {
        LOG_WARN("The svd_calculation is TRUE, but the thrusters weights were not set. "
                 "Setting the thrusters weights to a vector of ones...");
        thrustersWeights = Eigen::VectorXd::Ones(numberOfThrusters);
        _thrusters_weights.set(thrustersWeights);
    }
    else if(thrustersWeights.size() != 0 && thrustersWeights.size() != numberOfThrusters)
    {
        LOG_ERROR("The thrusters weights vector's size should be equal to the number of thrusters (%i), "
                  "but it currently has a size equal to %i.", numberOfThrusters, thrustersWeights.size());
        return false;
    }
    else
    {
        for(int i = 0; i < thrustersWeights.size(); i++)
        {
            if(thrustersWeights[i] <= 0)
            {
                LOG_ERROR("All the thrusters weights should have positive values. "
                          "Please check the index %i element.", i);
                return false;
            }
        }
    }

    weighingMatrix = thrustersWeights.asDiagonal();

    if(_svd_calculation.get())
    {
        base::MatrixXd auxPseudoInverse;
        svd.reset(new Eigen::JacobiSVD<Eigen::MatrixXd>(thrusterMatrix*weighingMatrix.inverse()*thrusterMatrix.transpose(),  Eigen::ComputeThinU | Eigen::ComputeThinV));

        // Pseudo-inverse of the svd matrix
        auxPseudoInverse = svd->solve(Eigen::MatrixXd::Identity(thrusterMatrix.rows(), thrusterMatrix.rows()));

        // Weighted pseudo-inverse used for optimal distribution of propulsion and control forces according to Fossen (1994, p. 98)
        weightedPseudoInverse = weighingMatrix.inverse()*thrusterMatrix.transpose()*auxPseudoInverse;

        // SVD of the weighted pseudo-inverse to calculate the expectedEffortVector
        svd.reset(new Eigen::JacobiSVD<Eigen::MatrixXd>(weightedPseudoInverse,  Eigen::ComputeThinU | Eigen::ComputeThinV));
    }
    else
        svd.reset(new Eigen::JacobiSVD<Eigen::MatrixXd>(thrusterMatrix.transpose(),  Eigen::ComputeThinU | Eigen::ComputeThinV));

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
        cmdVector = weightedPseudoInverse * inputVector;
    }
    else
    {
        cmdVector = thrusterMatrix.transpose()*inputVector;
    }

    for (unsigned int i = 0; i < jointCommand.size(); ++i){
        // scale values if a scaling is available
        if(i < _scalings.value().size())
        {
            if(cmdVector(i) > 0)
                cmdVector(i) = + sqrt(abs(_scalings.value()[i] * cmdVector(i)));
            else
                cmdVector(i) = - sqrt(abs(_scalings.value()[i] * cmdVector(i)));
        }

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

    base::LinearAngular6DCommand expectedEffort;
    expectedEffortVector = svd->solve(cmdVector);;
    expectedEffort.linear.x() = expectedEffortVector(0);
    expectedEffort.linear.y() = expectedEffortVector(1);
    expectedEffort.linear.z() = expectedEffortVector(2);
    expectedEffort.angular.x() = expectedEffortVector(3);
    expectedEffort.angular.y() = expectedEffortVector(4);
    expectedEffort.angular.z() = expectedEffortVector(5);
    _expected_effort.write(expectedEffort);

    return true;
}
