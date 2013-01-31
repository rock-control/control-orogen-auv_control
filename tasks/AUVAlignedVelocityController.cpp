#include "AUVAlignedVelocityController.hpp"

using namespace auv_control;

AUVAlignedVelocityController::AUVAlignedVelocityController(std::string const& name, TaskCore::TaskState initial_state)
    : AUVAlignedVelocityControllerBase(name, initial_state)
{
}

AUVAlignedVelocityController::AUVAlignedVelocityController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVAlignedVelocityControllerBase(name, engine, initial_state)
{
}

AUVAlignedVelocityController::~AUVAlignedVelocityController()
{
}


bool AUVAlignedVelocityController::startHook()
{
    AUVAlignedVelocityControllerBase::startHook();

    on_start = true;
    //reset the Pids and set the settings from the property
    setPIDSettings(_pid_settings.get());
    
    return true;
}
void AUVAlignedVelocityController::updateHook()
{
    AUVAlignedVelocityControllerBase::updateHook();

    base::LinearAngular6DCommand output_command;
    base::samples::RigidBodyState pose_sample;
    double delta_time;

    base::LinearAngular6DPIDSettings new_pid_settings = _pid_settings.get();
	
    if(last_pid_settings != new_pid_settings){
        setPIDSettings(new_pid_settings);
    }

    //hold current position if ther are no BodyState or at firts time
    if(_pose_sample.read(pose_sample) == RTT::NoData || on_start){
        if(on_start){
            on_start = false;
        }else{
            state(POSE_SAMPLE_MISSING);
        }
        //set the force and torque on all axsis to zero.
        for(int i = 0; i < 3; i++){
            output_command.linear(i) = 0;
            output_command.angular(i) = 0;
        } 
        //write the command
        _cmd_out.write(output_command);
        return;
    } else{
        state(RUNNING);
    }

    //if the inputs are valid
    if(this->gatherInputCommand()){
        //time since the last reglementation    
        delta_time = ((pose_sample.time - last_pose_sample_time).toSeconds());
        //time of the last reglementation
        last_pose_sample_time = pose_sample.time;
    
        //set unset valus from the input command in teh output comman unset too.
        //else reglementate the output command by update the pids
        for(int i = 0; i < 3; i++){
            if(base::isUnset(merged_command.linear(i))){
                output_command.linear(i) = base::unset<double>();
            } else{
                output_command.linear(i) = linear_pid[i].update(pose_sample.velocity(i), merged_command.linear(i), delta_time);
            }

            if(base::isUnset(merged_command.angular(i))){
                output_command.angular(i) = base::unset<double>();
            } else{
                output_command.angular(i) = angular_pid[i].update(pose_sample.angular_velocity(i), merged_command.angular(i), delta_time);
            }
        }
    } else{
        //dont move if the command is not valid
        for(int i = 0; i < 3; i++){
            output_command.linear(i) = 0;
            output_command.angular(i) = 0;
        } 
    }
    output_command.stamp = merged_command.stamp;
    //write the command
    _cmd_out.write(output_command);
}

void AUVAlignedVelocityController::setPIDSettings(base::LinearAngular6DPIDSettings new_settings){
    //reset the pids and set the pid-settings from the property
    std::cout << "Aendere PID-Settings in AUVAlignedVelocityController" << std::endl;
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
