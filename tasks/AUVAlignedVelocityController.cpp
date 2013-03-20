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

    for (int i = 0; i < 6; i++){
        avg[i] = 0;
        cnt[i] = 0;
    }
    
    return true;
}
void AUVAlignedVelocityController::updateHook()
{
    AUVAlignedVelocityControllerBase::updateHook();

    base::LinearAngular6DCommand output_command;
    base::samples::RigidBodyState pose_sample;
    double delta_time;

    base::LinearAngular6DPIDSettings new_pid_settings = _pid_settings.get();
    double roll;
    double pitch;
    base::Quaterniond rotation;


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

        //rotate the linear Vector, to get better Values 
        //roll = base::getRoll(pose_sample.orientation);
        //pitch = base::getPitch(pose_sample.orientation);
        //rotation = base::Quaterniond(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())) * base::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        //std::cout << "Vorher:" << merged_command.linear << std::endl;
        //merged_command.linear = rotation.conjugate() * merged_command.linear;
        //std::cout << "Nachher:" << merged_command.linear << std::endl;
    
        //set unset valus from the input command in teh output comman unset too.
        //else reglementate the output command by update the pids
        for(int i = 0; i < 3; i++){
            if(base::isUnset(merged_command.linear(i))){
                output_command.linear(i) = base::unset<double>();
            } else{
                //to use the PIDs in drive_simple wihout speeds
                if (i == 2){
                
                    output_command.linear(i) = linear_pid[i].update(pose_sample.velocity(i), merged_command.linear(i), delta_time);
                    
                } else{
                    output_command.linear(i) = linear_pid[i].update(0, merged_command.linear(i), delta_time);
                }
            }

            if(base::isUnset(merged_command.angular(i))){
                output_command.angular(i) = base::unset<double>();
            } else{
               /* if (i == 1){
                    output_command.angular(i) = angular_pid[i].update(-pose_sample.angular_velocity(i), merged_command.angular(i), delta_time);
            
                } else {
*/
                    output_command.angular(i) = angular_pid[i].update(pose_sample.angular_velocity(i), merged_command.angular(i), delta_time);
  //              }
            }
           
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
        //dont move if the command is not valid
        for(int i = 0; i < 3; i++){
            output_command.linear(i) = 0;
            output_command.angular(i) = 0;
        } 
    }
    output_command.stamp = merged_command.stamp;
    //write the command
    _cmd_out.write(output_command);

    base::LinearAngular6DCommand avg_out;
    
    for(int i = 0; i < 3; i++){
        avg_out.linear(i) = avg[i];
        avg_out.angular(i) = avg[3+i];
    }

    _avg_periode.write(avg_out);
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
