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
    AUVAlignedControllerBase::startHook();

    on_start = true;
    
    //reset the pids and set the pid-settings from the property
    setPIDSettings(_pid_settings.get());
    
    for (int i = 0; i < 6; i++){
        avg[i] = 0;
        cnt[i] = 0;
    }

    return true;

}

void AUVAlignedController::updateHook()
{
    AUVAlignedControllerBase::updateHook();

    base::LinearAngular6DCommand output_command;
    base::samples::RigidBodyState pose_sample;
    double delta_time;

    base::LinearAngular6DPIDSettings new_pid_settings = _pid_settings.get();

    if(last_pid_settings != new_pid_settings){
        setPIDSettings(new_pid_settings);
    }


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

    //std::cout << "ALIGNED" << std::endl;
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
            output_command.angular(1) = angular_pid[1].update(base::getPitch(pose_sample.orientation), merged_command.angular(1), delta_time);
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
        for(int i = 0; i < 3; i++){
            //Calculate the avarage Periode
            if (output_command.linear(i) > 0 && !last[i]){
                if (cnt[i] > 1){
                    avg[i] = (avg[i]*(cnt[i]-1) + (base::Time::now() - pos_start[i]).toSeconds())/(cnt[i]) ;
                }
                cnt[i] ++;
                pos_start[i] = base::Time::now();
                last[i] = true;
            } else if (output_command.linear(i) <= 0){
                last[i] = false;
            }
            
            if (output_command.angular(i) > 0 && !last[3+i]){
                if (cnt[3+i] > 1){
                    avg[3+i] = (avg[3+i]*(cnt[3+i]-1) + (base::Time::now() - pos_start[3+i]).toSeconds())/(cnt[3+i]) ;
                }
                cnt[3+i] ++;
                pos_start[3+i] = base::Time::now();
                last[3+i] = true;
            } else if (output_command.linear(i) <= 0){
                last[3+i] = false;
            }
            
        }
    } else{
        //std::cout << "ELSE ALIGNED" << std::endl;
        //if the input command are not vallid, dont move!
        output_command = this->dontMove();
    }
    output_command.stamp = merged_command.stamp;
    //write the command on the output port
    _cmd_out.write(output_command);
    state(RUNNING);
    
    base::LinearAngular6DCommand avg_out;
    
    for(int i = 0; i < 3; i++){
        avg_out.linear(i) = avg[i];
        avg_out.angular(i) = avg[3+i];
    }

    _avg_periode.write(avg_out);
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

void AUVAlignedController::setPIDSettings(base::LinearAngular6DPIDSettings new_settings){
    //reset the pids and set the pid-settings from the property
    std::cout << "Change PID-Settings in AUVAlignedController" << std::endl;
    for(int i = 0; i < 3; i++){
        
        linear_pid[i].reset();
        linear_pid[i].setPIDSettings(new_settings.linear[i]);
        
        if(new_settings.linear[i].Ti != 0){
            linear_pid[i].enableIntegral();
        } else{
            linear_pid[i].disableIntegral();
        }

        if(new_settings.linear[i].Td != 0){
            linear_pid[i].enableDerivative();
        } else{
            linear_pid[i].disableDerivative();
        }
        

        angular_pid[i].reset();
        angular_pid[i].setPIDSettings(new_settings.angular[i]);
        
        if(new_settings.angular[i].Ti != 0){
            angular_pid[i].enableIntegral();
        } else{
            angular_pid[i].disableIntegral();
        }

        if(new_settings.linear[i].Td != 0){
            angular_pid[i].enableDerivative();
        } else{
            angular_pid[i].disableDerivative();
        }
    }

    last_pid_settings = new_settings;
    return;
}   
