/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OptimalOrientationController.hpp"

using namespace auv_control;

OptimalOrientationController::OptimalOrientationController(std::string const& name, TaskCore::TaskState initial_state)
    : OptimalOrientationControllerBase(name, initial_state)
{
}

OptimalOrientationController::OptimalOrientationController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : OptimalOrientationControllerBase(name, engine, initial_state)
{
}

OptimalOrientationController::~OptimalOrientationController()
{
}


bool OptimalOrientationController::configureHook()
{
    
    if (! auv_control::Base::configureHook())
        return false;
    

    

    
    return true;
    
}



bool OptimalOrientationController::startHook()
{
    
    if (! auv_control::Base::startHook())
        return false;
    

    

    
    return true;
    
}



void OptimalOrientationController::updateHook()
{
    
    auv_control::Base::updateHook();
    return;
    
}



void OptimalOrientationController::errorHook()
{
    
    auv_control::Base::errorHook();
    

    

    
}



void OptimalOrientationController::stopHook()
{
    
    auv_control::Base::stopHook();
    

    

    
}



void OptimalOrientationController::cleanupHook()
{
    
    auv_control::Base::cleanupHook();
    

    

    
}


void OptimalOrientationController::keepPosition(){
    
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

bool OptimalOrientationController::calcOutput(){
    
    base::Vector3d opt_orientation;
    double opt_orientation_distance;

    opt_orientation = _opt_orientation.get();
    opt_orientation_distance = _opt_orientation_distance.get(); 

    output_command = merged_command;
    //take optimal heading

    if(merged_command.linear.norm() > opt_orientation_distance){
        output_command.linear(1) = 0;
        output_command.angular(2) = atan2(merged_command.linear(1), merged_command.linear(0))+base::getYaw(pose_sample.orientation) + opt_orientation(2);
        while (output_command.angular(2) > M_PI){
            output_command.angular(2) -= 2*M_PI;
        }

        while (output_command.angular(2) < -M_PI){
            output_command.angular(2) += 2*M_PI;
        }
     
    
    }
    return true;
}
