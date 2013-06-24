/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVWorldController.hpp"

using namespace auv_control;

AUVWorldController::AUVWorldController(std::string const& name, TaskCore::TaskState initial_state)
    : AUVWorldControllerBase(name, initial_state)
{
}

AUVWorldController::AUVWorldController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVWorldControllerBase(name, engine, initial_state)
{
}

AUVWorldController::~AUVWorldController()
{
}



bool AUVWorldController::startHook()
{
    auv_control::Base::startHook();
    return true;
}
void AUVWorldController::updateHook()
{
    auv_control::Base::updateHook();
    return;
}

void AUVWorldController::holdPosition(){
    output_command.stamp = pose_sample.time;

    output_command.linear(0) = pose_sample.position(0); 
    output_command.linear(1) = pose_sample.position(1); 
    output_command.linear(2) = pose_sample.position(2);

    output_command.angular(0) = base::getRoll(pose_sample.orientation);
    output_command.angular(1) = base::getPitch(pose_sample.orientation);
    output_command.angular(2) = base::getYaw(pose_sample.orientation);

    for(int i = 0; i < 3; i++){
        if(!_expected_inputs.get().linear[i]){
            output_command.linear(i) = base::unset<double>();
        }
        if(!_expected_inputs.get().angular[i]){
            output_command.angular(i) = base::unset<double>();
        }
    }
    //write the command
    _cmd_out.write(output_command);
}

bool AUVWorldController::calcOutput(){
    base::Quaterniond rotation;

    double yaw;
    bool z_nan = false;
    
    output_command.stamp = merged_command.stamp;
        
    //can only rotate, if ther are an value for x and y
    if( !(base::isUnset(merged_command.linear(0)) && base::isUnset( merged_command.linear(1)))){
        yaw = base::getYaw(pose_sample.orientation);
        rotation = base::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        //if z is NaN set it on 0. Else evry point is after the rotation NaN
            
        if(base::isUnset(merged_command.linear(2))){
            merged_command.linear(2) = 0;
            z_nan = true;
        }
        //rotate the target around the current position to get the target position in aligned frame
        output_command.linear = rotation.conjugate() * (merged_command.linear - pose_sample.position);
        //if z was NaN: Set it back to NaN
        if(z_nan){
            output_command.linear(2) = base::unset<double>();
        }
    } else{
        //set x and y to unset
        output_command.linear(0) = base::unset<double>();
        output_command.linear(1) = base::unset<double>();
        //calculate the diferenc betwen the wished and the current z
        if(base::isUnset(merged_command.linear(2))){
            output_command.linear(2) = base::unset<double>();
        } else{
            output_command.linear(2) = merged_command.linear(2) - pose_sample.position(2);
        }
    }
    //output angles are the same like the inputs
    output_command.angular = merged_command.angular;
        
    output_command.stamp = merged_command.stamp;
    return true;
}
