/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVForceTorqueController.hpp"

using namespace auv_control;

AUVForceTorqueController::AUVForceTorqueController(std::string const& name, TaskCore::TaskState initial_state)
    : AUVForceTorqueControllerBase(name, initial_state)
{
}

AUVForceTorqueController::AUVForceTorqueController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVForceTorqueControllerBase(name, engine, initial_state)
{
}

AUVForceTorqueController::~AUVForceTorqueController()
{
}

bool AUVForceTorqueController::startHook()
{
    AUVForceTorqueControllerBase::startHook();

    //take the calibration-matrix from the calibration property
    calibration = _calibration.get();
    return true;
}
void AUVForceTorqueController::updateHook()
{
    AUVForceTorqueControllerBase::updateHook();

    base::Vector6d input_vector;
    base::VectorXd output_vector;
    base::LinearAngular6DCommand cmd;
    base::actuators::Command actuators_command;
    
    //if ther are datas on the Input port, make one vector with force and torque
    if(_cascade.read(cmd) != RTT::NoData){ 
        input_vector(0) = cmd.linear(0);
        input_vector(1) = cmd.linear(1);
        input_vector(2) = cmd.linear(2);
        input_vector(3) = cmd.angular(0);
        input_vector(4) = cmd.angular(1);
        input_vector(5) = cmd.angular(2);
        
        //check for unset values and set it on 0, else the following multiplication with
        //the  matrix would not work.
        for(int i = 0; i < 6; i++){
            if(base::isUnset(input_vector(i))){
                input_vector(i) = 0;
            }
        }

        output_vector = (input_vector.transpose() * calibration).transpose();
        std::vector<double> cut_off = _cut_off.get();
        int cut_off_size = cut_off.size();
        cut_off.resize(output_vector.size());
        for(int i = cut_off_size; i < output_vector.size(); i++){
            cut_off[i] = 1;
        }

        //Cut values at 1 (or -1) if they are over 1 (or under -1). That shault not happend if the calibration
        //matrix is right.
        for(int i = 0; i < output_vector.size(); i++){
            if(output_vector(i) > cut_off[i]){
                output_vector(i) = cut_off[i];
            } else if(output_vector(i) < -cut_off[i]){
                output_vector(i) = -cut_off[i];
            }
        }
        
        //Resize the actuatos command on the Size of the output_vector. The size of the output_vetor are
        //the same like the calibration-matrix have collumns.
        actuators_command.resize(output_vector.size());
        for(int i = 0; i < output_vector.size(); i++){
            actuators_command.mode.at(i) = base::actuators::DM_PWM;
            actuators_command.target.at(i) = output_vector(i);
        }
        
        //write the command on the output port
        _motor_command.write(actuators_command);
    }
    return;
}

