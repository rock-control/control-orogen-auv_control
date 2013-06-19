/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVRelativeController.hpp"
#include <float.h>
using namespace auv_control;

AUVRelativeController::AUVRelativeController(std::string const& name, TaskCore::TaskState initial_state)
    : AUVRelativeControllerBase(name, initial_state)
{
}

AUVRelativeController::AUVRelativeController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVRelativeControllerBase(name, engine, initial_state)
{
}

AUVRelativeController::~AUVRelativeController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AUVRelativeController.hpp for more detailed
// documentation about them.




bool AUVRelativeController::configureHook()
{
    
    if (! auv_control::Controller_Base::configureHook())
        return false;
    

    

    
    return true;
    
}



bool AUVRelativeController::startHook()
{
    
    if (! auv_control::Controller_Base::startHook())
        return false;
    on_start = true;    

    return true;
    
}



void AUVRelativeController::updateHook()
{
    auv_control::Controller_Base::updateHook();    
    /*base::samples::RigidBodyState pose_sample;

    if(!this->getPoseSample()){
        return;
    } 
    
    
    //Hold position at first update
    if(on_start){
        on_start = false;
        output_command.linear(0) = 0.0;
        output_command.linear(1) = 0.0;
        output_command.linear(2) = 0.0;
        output_command.angular(0) = base::getRoll(pose_sample.orientation);
        output_command.angular(1) = base::getPitch(pose_sample.orientation);
        output_command.angular(2) = base::getYaw(pose_sample.orientation);
        //Set unexpected input unset, else ther are errors in the next controlers
        for(int i = 0; i < 3; i++){
            if(!_expected_inputs.get().linear[i]){
                output_command.linear(i) = base::unset<double>();
            }
            if(!_expected_inputs.get().angular[i]){
                output_command.angular(i) = base::unset<double>();
            }
        }
        //write the command
        _cmd_out.write(output_command);
        return;
    }
    
    
    if(this->gatherInputCommand()){
       double orientation[] = {base::getRoll(pose_sample.orientation), base::getPitch(pose_sample.orientation), base::getYaw(pose_sample.orientation)}; 
	for(int i = 0; i < 3; i++){
            if(!base::isInfinity<double>(merged_command.linear(i))){
               
		 output_command.linear(i) = merged_command.linear(i);
	    }
            if(!base::isInfinity<double>(merged_command.angular(i))){
                
		 output_command.angular(i) = orientation[i] + merged_command.angular(i);
		 while (output_command.angular(i) < - M_PI){
		   output_command.angular(i) += 2*M_PI;
		 }
		 while (output_command.angular(i) > M_PI){
		   output_command.angular(i) -= 2*M_PI;
		 }
	    }
            
        }
    }
    
    _cmd_out.write(output_command);
    state(RUNNING);
    */
}



void AUVRelativeController::errorHook()
{
    
    auv_control::Controller_Base::errorHook();
    

    

    
}



void AUVRelativeController::stopHook()
{
    
    auv_control::Controller_Base::stopHook();
    

    

    
}



void AUVRelativeController::cleanupHook()
{
    
    auv_control::Controller_Base::cleanupHook();
    

    

    
}

void AUVRelativeController::holdPosition(){
    output_command.linear(0) = 0.0;
    output_command.linear(1) = 0.0;
    output_command.linear(2) = 0.0;
    output_command.angular(0) = base::getRoll(pose_sample.orientation);
    output_command.angular(1) = base::getPitch(pose_sample.orientation);
    output_command.angular(2) = base::getYaw(pose_sample.orientation);
    //Set unexpected input unset, else ther are errors in the next controlers
    for(int i = 0; i < 3; i++){
        if(!_expected_inputs.get().linear[i]){
            output_command.linear(i) = base::unset<double>();
        }
        if(!_expected_inputs.get().angular[i]){
            output_command.angular(i) = base::unset<double>();
        }
    }
    //write the command
    _cmd_out.write(output_command);
    return;
}

bool AUVRelativeController::calcOutput(){
    double orientation[] = {base::getRoll(pose_sample.orientation), base::getPitch(pose_sample.orientation), base::getYaw(pose_sample.orientation)}; 
  
    for(int i = 0; i < 3; i++){
        if(!base::isInfinity<double>(merged_command.linear(i))){

            output_command.linear(i) = merged_command.linear(i);
        }
        if(!base::isInfinity<double>(merged_command.angular(i))){

            output_command.angular(i) = orientation[i] + merged_command.angular(i);
            while (output_command.angular(i) < - M_PI){
                output_command.angular(i) += 2*M_PI;
            }
            while (output_command.angular(i) > M_PI){
                output_command.angular(i) -= 2*M_PI;
            }
        }

    }
   return true;
}
