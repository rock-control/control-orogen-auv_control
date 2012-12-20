/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVAlignedController.hpp"
#include <math.h>

using namespace auv_control;

AUVAlignedController::AUVAlignedController(std::string const& name, TaskCore::TaskState initial_state)
    : AUVAlignedControllerBase(name, initial_state)
{
}

AUVAlignedController::AUVAlignedController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVAlignedControllerBase(name, engine, initial_state)
{
}

AUVAlignedController::~AUVAlignedController()
{
}

bool AUVAlignedController::startHook()
{
    on_start = true;
    //reset the pids and set the pid-settings from the property
    for(int i = 0; i < 3; i++){
        linear_pid[i].reset();
        linear_pid[i].setPIDSettings(_pid_settings.get().linear[i]);
        //linear_pid[i].enableIntegral();
        //linear_pid[i].enableDerivative();

        angular_pid[i].reset();
        angular_pid[i].setPIDSettings(_pid_settings.get().angular[i]);
        //angular_pid[i].enableIntegral();
        //angular_pid[i].enableDerivative();
    }
    return true;
}

void AUVAlignedController::updateHook()
{
    base::LinearAngular6DCommand output_command;
    base::samples::RigidBodyState pose_sample;
    double delta_time;

    //if it excists no BodyState or on the first  update
    if(_pose_sample.read(pose_sample) == RTT::NoData || on_start){
        if(on_start){
            on_start = false;
        }else{
            state(POSE_SAMPLE_MISSING);
        }
        output_command = this->dontMove();
        //write the command
        _cmd_out.write(output_command);
        return;
    }

    //if the input command is vallid
    if (this->gatherInputCommand()){
        
        //the time since the last reglementation
        delta_time = ((pose_sample.time - last_pose_sample_time).toSeconds());
        //the time of the last reglementation
        last_pose_sample_time = pose_sample.time;

        //if the value is set in the input command reglement the value for the output
        //by update the pid. Else set the value of the output command unset. 
        for(int i = 0; i < 3; i++){
            if(base::isUnset(merged_command.linear(i))){
                output_command.linear(i) = base::unset<double>();
            } else{
                output_command.linear(i) = linear_pid[i].update(0.0, merged_command.linear(i), delta_time);
            }
        }

        //Reglementation for Roll
        if(base::isUnset(merged_command.angular(0))){
            output_command.angular(0) = base::unset<double>();
        } else{ 
            output_command.angular(0) = angular_pid[0].update(base::getRoll(pose_sample.orientation), merged_command.angular(0), delta_time);
        }

        //Reglementation for Pitch
        if(base::isUnset(merged_command.angular(1))){
            output_command.angular(1) = base::unset<double>();
        } else{
            output_command.angular(1) = angular_pid[1].update(-(base::getPitch(pose_sample.orientation)), merged_command.angular(1), delta_time);
        }


        //reglementation for Yaw, use the shortest way.
        if(base::isUnset(merged_command.angular(2))){
            output_command.angular(2) = base::unset<double>();
        } else{
            double current = base::getYaw(pose_sample.orientation);
            if(merged_command.angular(2) - current > M_PI){
                current = (2 * M_PI) - current;
            } else if(merged_command.angular(2) - current < -M_PI){
                current = (-2 * M_PI) - current;
            }
            output_command.angular(2) = angular_pid[2].update(current, merged_command.angular(2), delta_time);
        }
    } else{
        //if the input command are not vallid, dont move!
        output_command = this->dontMove();
    }
    output_command.stamp = merged_command.stamp;
    //write the command on the output port
    _cmd_out.write(output_command);
}

base::LinearAngular6DCommand AUVAlignedController::dontMove(){
    base::LinearAngular6DCommand output_command;
    for(int i = 0; i < 3; i++){
        if(_expected_inputs.get().linear[i]){
            output_command.linear(i) = 0;
        } else{
            output_command.linear(i) = base::unset<double>();
        }

        if(_expected_inputs.get().angular[i]){
            output_command.angular(i) = 0;
        } else{
            output_command.angular(i) = base::unset<double>();
        }
    }
    return output_command;

}
