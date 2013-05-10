/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVOptimalOrientationController.hpp"

using namespace auv_control;

AUVOptimalOrientationController::AUVOptimalOrientationController(std::string const& name, TaskCore::TaskState initial_state)
    : AUVOptimalOrientationControllerBase(name, initial_state)
{
}

AUVOptimalOrientationController::AUVOptimalOrientationController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVOptimalOrientationControllerBase(name, engine, initial_state)
{
}

AUVOptimalOrientationController::~AUVOptimalOrientationController()
{
}


bool AUVOptimalOrientationController::configureHook()
{
    
    if (! auv_control::Controller_Base::configureHook())
        return false;
    

    

    
    return true;
    
}



bool AUVOptimalOrientationController::startHook()
{
    
    if (! auv_control::Controller_Base::startHook())
        return false;
    

    

    
    return true;
    
}



void AUVOptimalOrientationController::updateHook()
{
    
    auv_control::Controller_Base::updateHook();
    return;
    
}



void AUVOptimalOrientationController::errorHook()
{
    
    auv_control::Controller_Base::errorHook();
    

    

    
}



void AUVOptimalOrientationController::stopHook()
{
    
    auv_control::Controller_Base::stopHook();
    

    

    
}



void AUVOptimalOrientationController::cleanupHook()
{
    
    auv_control::Controller_Base::cleanupHook();
    

    

    
}


void AUVOptimalOrientationController::holdPosition(){
    
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

    _cmd_out.write(output_command);
}

bool AUVOptimalOrientationController::calcOutput(){
    
    base::Vector3d opt_orientation;
    double opt_orientation_distance;

    opt_orientation = _opt_orientation.get();
    opt_orientation_distance = _opt_orientation_distance.get(); 

    output_command = merged_command;
    //take optimal heading

    if(merged_command.linear.norm() > opt_orientation_distance){
        output_command.angular(2) = atan2(merged_command.linear(1), merged_command.linear(0))+base::getYaw(pose_sample.orientation) + opt_orientation(2);
    }
    return true;
}
