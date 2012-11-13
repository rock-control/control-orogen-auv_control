/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVAlignedVelocityControler.hpp"

using namespace auv_control;

AUVAlignedVelocityControler::AUVAlignedVelocityControler(std::string const& name, TaskCore::TaskState initial_state)
    : AUVAlignedVelocityControlerBase(name, initial_state)
{
}

AUVAlignedVelocityControler::AUVAlignedVelocityControler(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVAlignedVelocityControlerBase(name, engine, initial_state)
{
}

AUVAlignedVelocityControler::~AUVAlignedVelocityControler()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AUVAlignedVelocityControler.hpp for more detailed
// documentation about them.

bool AUVAlignedVelocityControler::configureHook(){
     return true;
}

bool AUVAlignedVelocityControler::startHook() {
     //reset the PIDS
     x_pid.reset();
     y_pid.reset();
     z_pid.reset();
     yaw_pid.reset();
     pitch_pid.reset();
     roll_pid.reset();
     
     //set PID settings
     control::AlignedVelocityController6D controller = _controller.get();
     x_pid.setPIDSettings(controller.x_settings);
     y_pid.setPIDSettings(controller.y_settings);
     z_pid.setPIDSettings(controller.z_settings);
     yaw_pid.setPIDSettings(controller.yaw_settings);
     pitch_pid.setPIDSettings(controller.pitch_settings);
     roll_pid.setPIDSettings(controller.roll_settings);
     
     //set targets on zero
     target_x = 0;
     target_y = 0;
     target_z = 0;
     target_yaw = 0;
     target_pitch = 0;
     target_roll = 0;
     
     //set calibration
     calibration = _calibration.get();
     
     return true;
}
void AUVAlignedVelocityControler::updateHook()
{
    base::actuators::Command force_command;
    control::AlignedVelocityCommand6D direct_new;
    control::AlignedVelocityCommand6D aligned_new;
    control::AlignedVelocityCommand6D next_command;
    base::samples::RigidBodyState body_state;
    
    bool data_on_aligned;
    bool data_on_direct;
    
    double delta_time;
    
    if(_position_sample.read(body_state)!=RTT::NoData){
        //ERROR: no new BodyState
        exception(INPUT_ERROR);

    }
    
    
    
    data_on_aligned = _aligned_velocity_command_aligned.read(aligned_new)!=RTT::NoData;
    data_on_direct = _aligned_velocity_command_direct.read(direct_new)!=RTT::NoData;  
    
    
    if(data_on_aligned && !data_on_direct){
        next_command = aligned_new;
    } else if (!data_on_aligned && data_on_direct){
        next_command = direct_new;
    } else if (data_on_aligned && data_on_direct){
        next_command = this->mergeCommands(aligned_new, direct_new);
    } else{
        //ERROR: no input
        exception(INPUT_ERROR);

    }
    
    if(next_command.AllSet()){
        //Error
        exception(COMMAND_ERROR);

    }
    
    
    delta_time = ((body_state.time - last_body_state_time).toSeconds());
    this->genVector(next_command, body_state.velocity, body_state.angular_velocity, delta_time);
    
    
    force_command = this->genMotionCommand();
    force_command.time = body_state.time;
    last_body_state_time = body_state.time;
    
    _force_command.write(force_command);

    
}
void AUVAlignedVelocityControler::errorHook()
{
}
void AUVAlignedVelocityControler::stopHook()
{
}
void AUVAlignedVelocityControler::cleanupHook()
{
}


control::AlignedVelocityCommand6D AUVAlignedVelocityControler::mergeCommands(control::AlignedVelocityCommand6D command1, control::AlignedVelocityCommand6D command2){
    control::AlignedVelocityCommand6D merged_command;
    
    //Merge X
    if(command1.x_mode != control::NOT_SET && command2.x_mode == control::NOT_SET){
        merged_command.x = command1.x;
        merged_command.x_mode = command1.x_mode;
    } else if(command1.x_mode == control::NOT_SET && command2.x_mode != control::NOT_SET){
        merged_command.x = command2.x;
        merged_command.x_mode = command2.x_mode;
    } else{
        //ERROR cant merge
        exception(MERGE_ERROR);
        
    }
    
    //Merge Y
    if(command1.y_mode != control::NOT_SET && command2.y_mode == control::NOT_SET){
        merged_command.y = command1.y;
        merged_command.y_mode = command1.y_mode;
    } else if(command1.y_mode == control::NOT_SET && command2.y_mode != control::NOT_SET){
        merged_command.y = command2.y;
        merged_command.y_mode = command2.y_mode;
    } else{
        //ERROR cant merge
        exception(MERGE_ERROR);
    }
    
    //Merge Z
    if(command1.z_mode != control::NOT_SET && command2.z_mode == control::NOT_SET){
        merged_command.z = command1.z;
        merged_command.z_mode = command1.z_mode;
    } else if(command1.z_mode == control::NOT_SET && command2.z_mode != control::NOT_SET){
        merged_command.z = command2.z;
        merged_command.z_mode = command2.z_mode;
    } else{
        //ERROR cant merge
        exception(MERGE_ERROR);
    }
    
    //Merge Yaw
    if(command1.yaw_mode != control::NOT_SET && command2.yaw_mode == control::NOT_SET){
        merged_command.yaw = command1.yaw;
        merged_command.yaw_mode = command1.yaw_mode;
    } else if(command1.yaw_mode == control::NOT_SET && command2.yaw_mode != control::NOT_SET){
        merged_command.yaw = command2.yaw;
        merged_command.yaw_mode = command2.yaw_mode;
    } else{
        //ERROR cant merge
        exception(MERGE_ERROR);
    }
    
    //Merge Pitch
    if(command1.pitch_mode != control::NOT_SET && command2.pitch_mode == control::NOT_SET){
        merged_command.pitch = command1.pitch;
        merged_command.pitch_mode = command1.pitch_mode;
    } else if(command1.pitch_mode == control::NOT_SET && command2.pitch_mode != control::NOT_SET){
        merged_command.pitch = command2.pitch;
        merged_command.pitch_mode = command2.pitch_mode;
    } else{
        //ERROR cant merge
        exception(MERGE_ERROR);
    }
    
    //Merge Roll
    if(command1.roll_mode != control::NOT_SET && command2.roll_mode == control::NOT_SET){
        merged_command.roll = command1.roll;
        merged_command.roll_mode = command1.roll_mode;
    } else if(command1.roll_mode == control::NOT_SET && command2.roll_mode != control::NOT_SET){
        merged_command.roll = command2.roll;
        merged_command.roll_mode = command2.roll_mode;
    } else{
        //ERROR cant merge
        exception(MERGE_ERROR);
    }
    
    return merged_command;
}
void AUVAlignedVelocityControler::lastTarget(control::AlignedVelocityCommand6D command){
    command.x_mode     == control::LAST_TARGET ? command.x     = target_x     : target_x = command.x;
    command.y_mode     == control::LAST_TARGET ? command.y     = target_y     : target_y = command.y;
    command.z_mode     == control::LAST_TARGET ? command.z     = target_z     : target_z = command.z;
    command.yaw_mode   == control::LAST_TARGET ? command.yaw   = target_yaw   : target_yaw = command.yaw;
    command.pitch_mode == control::LAST_TARGET ? command.pitch = target_pitch : target_pitch = command.pitch;
    command.roll_mode  == control::LAST_TARGET ? command.roll  = target_roll  : target_roll = command.roll;
}

void AUVAlignedVelocityControler::holdPosition(control::AlignedVelocityCommand6D command){
    if(command.x_mode     == control::HOLD_SETTING) command.x     = current.x;
    if(command.y_mode     == control::HOLD_SETTING) command.y     = current.y;
    if(command.z_mode     == control::HOLD_SETTING) command.z     = current.z;
    if(command.yaw_mode   == control::HOLD_SETTING) command.yaw   = current.yaw;
    if(command.pitch_mode == control::HOLD_SETTING) command.pitch = current.pitch;
    if(command.roll_mode  == control::HOLD_SETTING) command.roll  = current.roll;
}

void AUVAlignedVelocityControler::genVector(control::AlignedVelocityCommand6D command, 
                                            base::Vector3d velocity, 
                                            base::Vector3d angel_velocity,
                                            double delta_time){
    vector_command(6);
     
    vector_command[0] = x_pid.update(velocity(0) ,command.x, delta_time);
    vector_command[1] = y_pid.update(velocity(1) ,command.y, delta_time);
    vector_command[2] = z_pid.update(velocity(2) ,command.z, delta_time);
    vector_command[3] = yaw_pid.update(velocity(0) ,command.yaw, delta_time);
    vector_command[4] = pitch_pid.update(velocity(1) ,command.pitch, delta_time);
    vector_command[5] = roll_pid.update(velocity(2) ,command.roll, delta_time);
}

base::actuators::Command AUVAlignedVelocityControler::genMotionCommand(){
    Eigen:: VectorXd result = ((vector_command.transpose()) * calibration).transpose();
    
    base::actuators::Command command;
    
    if(_isASV.get()){
        command.resize(4);
        for (int i = 0; i<4; i++){
            command.mode.at(i)=base::actuators::DM_PWM;
            command.target.at(i)=result(i);
        }
    } else {
        command.resize(6);
        for (int i = 0; i<6; i++){
            command.mode.at(i)=base::actuators::DM_PWM;
            command.target.at(i)=result(i);
        }
    }
    return command;
    
}

