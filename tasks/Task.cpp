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

//    x_pid.reset();
//    y_pid.reset();
    z_pid.reset();
    yaw_pid.reset();
    pitch_pid.reset();
    roll_pid.reset();

//    x_pid.setPIDSettings(_controller_x.get());
//    y_pid.setPIDSettings(_controller_y.get());
    z_pid.setPIDSettings(_controller_z.get());
    
    yaw_pid.setPIDSettings(_controller_yaw.get());
    pitch_pid.setPIDSettings(_controller_pitch.get());
    roll_pid.setPIDSettings(_controller_roll.get());

    //inital_heading = std::numeric_limits<double>::infinity();
    body_state.invalidate();
    //last_state = RUNNING;

    //last_body_time = 0;
    //last_motor_command_time = 0;

    return true;
}


void Task::updateHook()
{
    TaskBase::updateHook();
    while(_pose_samples.read(body_state) == RTT::NewData);
    while(_motion_commands.read(motion_command) == RTT::NewData);

    if(motion_command.timestamp == base::Time::fromSeconds(0)){
        state(WAITING_FOR_COMMAND);
        return;
    }else if(body_state.time == base::Time::fromSeconds(0)){
        state(WAITING_FOR_RBS);
        return;
    }else if((base::Time::now() - motion_command.timestamp).toSeconds() > _timeout.get()){
        printf("Last Command: %f\n",(base::Time::now() - motion_command.timestamp).toSeconds());
        error(TIMEOUT_COMMAND); 
    }else if((base::Time::now() - body_state.time).toSeconds() > _timeout.get()){
        printf("Last Body: %f\n",(base::Time::now() - body_state.time).toSeconds());
        error(TIMEOUT_RBS); 
    }else{
        state(RUNNING);
    }

    Eigen::Matrix<double,6,6> controlMatrix;

    
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
    
    base::Angle currentYaw      = base::Angle::fromRad(body_state.getYaw());
    base::Angle currentPitch    = base::Angle::fromRad(body_state.getPitch());
    base::Angle currentRoll     = base::Angle::fromRad(body_state.getRoll());

    //Need to non-Normalize the yaw/heading values to prevent strange behavior on 0° and 180° 
    double current_heading = currentYaw.getRad();
    if (current_heading - motion_command.heading > M_PI){
        current_heading -= 2.0*M_PI;
    }else if (current_heading - motion_command.heading < -M_PI){
        current_heading += 2.0*M_PI;
    }

    Eigen::Vector3d rotationVector;
    rotationVector(0) = yaw_pid.update(current_heading,motion_command.heading);
    rotationVector(1) = yaw_pid.update(currentPitch.getRad(),_pitch_target.get());
    rotationVector(2) = yaw_pid.update(currentRoll.getRad(),_roll_target.get());

    Eigen::Vector3d translationVector(motion_command.x_speed,motion_command.y_speed,(motion_command.z-body_state.position[2]));
    if(_pitch_roll_comensation.get()){
        Eigen::Quaterniond orientation_without_heading = base::removeYaw(body_state.orientation);
        translationVector= orientation_without_heading * translationVector;
    }

    //printf("New Movemtn Vector: %3.3f %3.3f %3.3f\n",translationVector[0],translationVector[1],translationVector[2]);


    Eigen::Matrix<double,6,1> movementVector;
    movementVector.block(0,0,3,1) = translationVector;
    movementVector.block(3,0,3,1) = rotationVector;
    Eigen::Matrix<double,6,1> resulingThrusterValue = controlMatrix.transpose() * movementVector ;
    //std::cout << rotationVector << std::endl;
    
    printf("Resulting Vector:  %+1.3f, %+1.3f, %+1.3f, %+1.3f, %+1.3f, %+1.3f\n",
            resulingThrusterValue[0],
            resulingThrusterValue[1],
            resulingThrusterValue[2],
            resulingThrusterValue[3],
            resulingThrusterValue[4],
            resulingThrusterValue[5]
          );




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

