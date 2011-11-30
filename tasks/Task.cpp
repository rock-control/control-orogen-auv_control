/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace auv_control;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
}

double Task::correct_pwm_value(double value, int motorIndex) const
{
    //todo more linearization work needed;

    double dead_zone = _thruster_death_zones.get()[motorIndex];
    if (value > 0)
        return  dead_zone + (value * (1 - dead_zone));
    else if (value < 0)
        return -dead_zone + (value * (1 - dead_zone));
    else return value;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     if (! TaskBase::configureHook())
//         return false;
//     return true;
// }
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    x_pid.reset();
    y_pid.reset();
    z_pid.reset();
    yaw_pid.reset();
    pitch_pid.reset();
    roll_pid.reset();

    x_pid.setPIDSettings(_controller_x.get());
    y_pid.setPIDSettings(_controller_y.get());
    z_pid.setPIDSettings(_controller_z.get());
    
    yaw_pid.setPIDSettings(_controller_yaw.get());
    pitch_pid.setPIDSettings(_controller_pitch.get());
    roll_pid.setPIDSettings(_controller_roll.get());

    //inital_heading = std::numeric_limits<double>::infinity();
    body_state.invalidate();
    //last_state = RUNNING;

    //last_body_time = 0;
    //last_motor_command_time = 0;
    
    
    motorCommands.resize(6);
    if(_thruster_death_zones.get().size() != 6){
        return false;
    }
    return true;
}


void Task::updateHook()
{
    TaskBase::updateHook();
    while(_pose_samples.read(body_state) == RTT::NewData);
    
    while(_motion_commands.read(motion_command) == RTT::NewData){
        last_motion_command_time = base::Time::now(); //TODO until motion command has no timestamp
    }

    //if(motion_command.time == base::Time::fromSeconds(0)){ //TODO timestamping
    if(last_motion_command_time == base::Time::fromSeconds(0)){
        state(WAITING_FOR_COMMAND);
        return;
    }else if(body_state.time == base::Time::fromSeconds(0)){ //TODO timestamping
        state(WAITING_FOR_RBS);
        return;
    //}else if((base::Time::now() - motion_command.time).toSeconds() > _timeout.get()){
    }else if((base::Time::now() - last_motion_command_time).toSeconds() > _timeout.get()){
        error(TIMEOUT_COMMAND); 
    }else if((base::Time::now() - body_state.time).toSeconds() > _timeout.get()){
        error(TIMEOUT_RBS); 
    }else{
        state(RUNNING);
    }

    Eigen::Matrix<double,6,6> controlMatrix = _thruster_control_matrix.get();
/*
    
                            //Links     //Rechts    //Dive  //Strave    //Turn  //Pitch
    controlMatrix.row(0) << 1,          1,          0,      0,          0,      0;      //X
    controlMatrix.row(1) << 0,          0,          0,      1,          0.2,     0;      //Y
    controlMatrix.row(2) << 0,          0,          1,      0,          0,      0.2;    //Z
    controlMatrix.row(3) << 0,          0,          0,      -0.2,       1,      0;      //Yaw
    controlMatrix.row(4) << 0,          0,          0,      0,          0,      1;      //Pitch
    controlMatrix.row(5) << 0,          0,          0,      0,          0,      0;      //Roll
    Eigen::Quaterniond current_orientation = body_state.orientation;

    


    Eigen::Quaterniond target_orientation = Eigen::AngleAxisd(motion_command.heading, Eigen::Vector3d::UnitX())
                                            * Eigen::AngleAxisd(_pitch_target.get(), Eigen::Vector3d::UnitY())
                                            * Eigen::AngleAxisd(_roll_target.get(), Eigen::Vector3d::UnitZ());
  
*/
    base::Angle currentYaw      = base::Angle::fromRad(body_state.getYaw());
    base::Angle currentPitch    = base::Angle::fromRad(body_state.getPitch());
    base::Angle currentRoll     = base::Angle::fromRad(body_state.getRoll());

    Eigen::Vector3d currentSpeed;
    if(_use_rbs_for_speed.get()){
        if(!body_state.hasValidVelocity()){
            error(INVALID_RBS);
        }
        currentSpeed=body_state.velocity;
    }else{
        currentSpeed.setZero();
    }

    //Need to non-Normalize the yaw/heading values to prevent strange behavior on 0° and 180° 
    double currentHeading = currentYaw.getRad();
    if (currentHeading - motion_command.heading > M_PI){
        currentHeading -= 2.0*M_PI;
    }else if (currentHeading - motion_command.heading < -M_PI){
        currentHeading += 2.0*M_PI;
    }

    Eigen::Vector3d rotationVector;
    rotationVector(0) = yaw_pid.update(currentHeading,motion_command.heading);
    rotationVector(1) = pitch_pid.update(currentPitch.getRad(),_pitch_target.get());
    rotationVector(2) = roll_pid.update(currentRoll.getRad(),_roll_target.get());

    Eigen::Vector3d translationVector;
    translationVector(0) = x_pid.update(currentSpeed(0),motion_command.x_speed); 
    translationVector(1) = y_pid.update(currentSpeed(1),motion_command.y_speed);  
    translationVector(2) = z_pid.update(body_state.position[2],motion_command.z); 
    
    if(_pitch_roll_comensation.get()){
        Eigen::Quaterniond orientation_without_heading = base::removeYaw(body_state.orientation);
        translationVector= orientation_without_heading * translationVector;
    }

    //printf("New Movemtn Vector: %3.3f %3.3f %3.3f\n",translationVector[0],translationVector[1],translationVector[2]);


    Eigen::Matrix<double,6,1> movementVector;
    movementVector.block(0,0,3,1) = translationVector;
    movementVector.block(3,0,3,1) = rotationVector;
    Eigen::Matrix<double,6,1> resultingThrusterValue = controlMatrix.transpose() * movementVector ;
    
    printf("Resulting Vector:  %+1.3f, %+1.3f, %+1.3f, %+1.3f, %+1.3f, %+1.3f\n",
            resultingThrusterValue[0],
            resultingThrusterValue[1],
            resultingThrusterValue[2],
            resultingThrusterValue[3],
            resultingThrusterValue[4],
            resultingThrusterValue[5]
    );

    for(int i=0;i<6;i++){
        motorCommands.mode[i] = base::actuators::DM_PWM;
        resultingThrusterValue[i] = correct_pwm_value(resultingThrusterValue[i], i);
	
	//Cutoff
	if(resultingThrusterValue[i] > _cutoff.value()){
		resultingThrusterValue[i] = _cutoff.value();
	}else if(resultingThrusterValue[i] < -_cutoff.value()){
		resultingThrusterValue[i] = -_cutoff.value();
	}
        motorCommands.target[i] = resultingThrusterValue[i];
    }
    motorCommands.time = base::Time::now();
    _motor_commands.write(motorCommands);

}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

