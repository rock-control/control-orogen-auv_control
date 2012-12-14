/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVAlignedController.hpp"
#define PI 3.1415926535897932384626433832795


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



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AUVAlignedController.hpp for more detailed
// documentation about them.

// bool AUVAlignedController::configureHook()
// {
//     if (! AUVAlignedControllerBase::configureHook())
//         return false;
//     return true;
// }
bool AUVAlignedController::startHook()
{
    on_start = true;
    //reset the pids and set the pid-settings from the property
    for(int i = 0; i < 3; i++){
        linear_pid[i].reset();
        linear_pid[i].setPIDSettings(_pid_settings.get().linear[i]);
        linear_pid[i].enableIntegral();
        linear_pid[i].enableDerivative();

        angular_pid[i].reset();
        angular_pid[i].setPIDSettings(_pid_settings.get().angular[i]);
        angular_pid[i].enableIntegral();
        angular_pid[i].enableDerivative();
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
        on_start = false;
        for(int i = 0; i < 3; i++){
            //no movment
            output_command.linear(i) = 0;
            output_command.angular(i) = 0;
        }
        //write the command
        _cmd_out.write(output_command);
        return;
    }

    //if the input command is vallid
    if (this->gatherInputCommand()){
        std::cout << merged_command.linear << std::endl;
        std::cout << merged_command.angular << std::endl;
        
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

        std::cout << "Current: "  << -base::getPitch(pose_sample.orientation) << std::endl;
        std::cout << "wished: "  << merged_command.angular(1) << std::endl;
        //Reglementation for Pitch
        if(base::isUnset(merged_command.angular(1))){
            output_command.angular(1) = base::unset<double>();
        } else{
            output_command.angular(1) = angular_pid[1].update(-(base::getPitch(pose_sample.orientation)), merged_command.angular(1), delta_time);
        }
        std::cout << "output: "  << output_command.angular(1) << std::endl;

        //angular_pid[1].printCoefficients();

        //reglementation for Yaw, use the shortest way.
        if(base::isUnset(merged_command.angular(2))){
            output_command.angular(2) = base::unset<double>();
        } else{
            double current = base::getYaw(pose_sample.orientation);
            if(merged_command.angular(2) - current > PI){
                current = (2*PI)-current;
            } else if(merged_command.angular(2) - current < -PI){
                current = (-2*PI)-current;
            }
            output_command.angular(2) = angular_pid[2].update(current, merged_command.angular(2), delta_time);
        }
    } else{
        //if the input command are not vallid, dont move!
        for(int i = 0; i < 3; i++){
            output_command.linear(i) = 0;
            output_command.angular(i) = 0;
        }
    }
    output_command.stamp = merged_command.stamp;
    //write the command on the output port
    _cmd_out.write(output_command);
}
// void AUVAlignedController::errorHook()
// {
//     AUVAlignedControllerBase::errorHook();
// }
// void AUVAlignedController::stopHook()
// {
//     AUVAlignedControllerBase::stopHook();
// }
// void AUVAlignedController::cleanupHook()
// {
//     AUVAlignedControllerBase::cleanupHook();
// }

