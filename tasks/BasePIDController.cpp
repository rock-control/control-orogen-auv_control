/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "BasePIDController.hpp"
#include <auv_control/6dControl.hpp>

using namespace auv_control;

BasePIDController::BasePIDController(std::string const& name)
    : BasePIDControllerBase(name)
{
}

BasePIDController::BasePIDController(std::string const& name, RTT::ExecutionEngine* engine)
    : BasePIDControllerBase(name, engine)
{
}

BasePIDController::~BasePIDController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BasePIDController.hpp for more detailed
// documentation about them.

bool BasePIDController::configureHook()
{
    if (! BasePIDControllerBase::configureHook())
        return false;

    use_parallel_pid_settings = _use_parallel_pid_settings;
    parallel_pid_settings = _parallel_pid_settings.get();
    pid_settings = _pid_settings.get();

    for (int i = 0; i < 3; ++i)
    {
        if(_use_parallel_pid_settings){
            mLinearPIDs[i].setParallelPIDSettings(parallel_pid_settings.linear[i]);
            mAngularPIDs[i].setParallelPIDSettings(parallel_pid_settings.angular[i]);
        } else {
            mLinearPIDs[i].setPIDSettings(pid_settings.linear[i]);
            mAngularPIDs[i].setPIDSettings(pid_settings.angular[i]);
        }
    }

    return true;
}
bool BasePIDController::startHook()
{
    if (! BasePIDControllerBase::startHook())
        return false;
    return true;
}
void BasePIDController::updateHook()
{
    BasePIDControllerBase::updateHook();
}
void BasePIDController::errorHook()
{
    BasePIDControllerBase::errorHook();
}
void BasePIDController::stopHook()
{
    BasePIDControllerBase::stopHook();
}
void BasePIDController::cleanupHook()
{
    BasePIDControllerBase::cleanupHook();
}
bool BasePIDController::calcOutput()
{
    // Check for new PID-Settings
    if((use_parallel_pid_settings && (!(_use_parallel_pid_settings.get()))) ||
            (pid_settings != _pid_settings.get())){
        use_parallel_pid_settings = false;
        pid_settings = _pid_settings.get();
        for (int i = 0; i < 3; ++i)
        {
            mLinearPIDs[i].setPIDSettings(pid_settings.linear[i]);
            mAngularPIDs[i].setPIDSettings(pid_settings.angular[i]);
        }
    } else if((!use_parallel_pid_settings && ((_use_parallel_pid_settings.get()))) ||
            (parallel_pid_settings != _parallel_pid_settings.get())){
        use_parallel_pid_settings = true;
        parallel_pid_settings = _parallel_pid_settings.get();
        for (int i = 0; i < 3; ++i)
        {
            mLinearPIDs[i].setParallelPIDSettings(parallel_pid_settings.linear[i]);
            mAngularPIDs[i].setParallelPIDSettings(parallel_pid_settings.angular[i]);
        }
    } 

    
    // We start by copying merged_command so that we can simply ignore the unset
    // values
    base::LinearAngular6DCommand output_command = merged_command;
    LinearAngular6DPIDState pid_state;
    for (int i = 0; i < 3; ++i)
    {
        if (!base::isUnset(merged_command.linear(i)))
        {
            output_command.linear(i) =
                mLinearPIDs[i].update(currentLinear(i),
                                   merged_command.linear(i),
                                   merged_command.time.toSeconds());
            pid_state.linear[i] = auv_control::PIDState(mLinearPIDs[i].getState(), true);
        }
        else
        {
            pid_state.linear[i].active = false;
        }
        if (!base::isUnset(merged_command.angular(i)))
        {
            output_command.angular(i) =
                mAngularPIDs[i].update(currentAngular(i),
                                   merged_command.angular(i),
                                   merged_command.time.toSeconds());
            pid_state.angular[i] = auv_control::PIDState(mAngularPIDs[i].getState(), true);
        }
        else
        {
            pid_state.angular[i].active = false;
        }
    }

    // Stop controling for axis with bad variance
    bool unsure_pose = false; 
    for(unsigned i = 0; i < 3; i++){
        if(currentLinearCov(i,i) > _variance_threshold.get()){
            output_command.linear(i) = 0;
            unsure_pose = true;
            if(state() != UNSURE_POSE_SAMPLE){
                state(UNSURE_POSE_SAMPLE);
            }
        }
        if(currentAngularCov(i,i) > _variance_threshold.get()){
            output_command.angular(i) = 0;
            unsure_pose = true;
            if(state() != UNSURE_POSE_SAMPLE){
                state(UNSURE_POSE_SAMPLE);
            }
        }
    }




    _pid_state.write(pid_state);
    _cmd_out.write(output_command);
    
    if(unsure_pose){
        return false;
    }
    
    return true;
}

void BasePIDController::keepPosition(){
    base::LinearAngular6DCommand output_command;
    output_command.time = pose_sample.time;
    if(!_nan_on_keep_position.get()){
        for(unsigned int i = 0; i < 3; i++){
            if(_expected_inputs.get().linear[i]){
                output_command.linear(i) = 0;
            }
            if(_expected_inputs.get().angular[i]){
                output_command.angular(i) = 0;
            }
        }
    }
    _cmd_out.write(output_command);
}

