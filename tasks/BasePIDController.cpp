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



bool BasePIDController::setParallel_pid_settings(::base::LinearAngular6DParallelPIDSettings const & value)
{
    if(_use_parallel_pid_settings.get())
    {
        for (int i = 0; i < 3; ++i)
        {
            mPIDs.linear[i].setParallelPIDSettings(value.linear[i]);
            mPIDs.angular[i].setParallelPIDSettings(value.angular[i]);
        }
    }
    else return false;
    return(BasePIDControllerBase::setParallel_pid_settings(value));
}

bool BasePIDController::setPid_settings(::base::LinearAngular6DPIDSettings const & value)
{
    if(!_use_parallel_pid_settings.get())
    {
        for (int i = 0; i < 3; ++i)
        {
            mPIDs.linear[i].setPIDSettings(value.linear[i]);
            mPIDs.angular[i].setPIDSettings(value.angular[i]);
        }
    }
    else return false;
    return(BasePIDControllerBase::setPid_settings(value));
}

std::pair<base::LinearAngular6DCommand, LinearAngular6DPIDState> BasePIDController::calcPIDStateCommand(
    const base::LinearAngular6DCommand &reference, const currentState &current_state, LinearAngular6DPID &pid)
{
    // We start by copying the reference so that we can simply ignore the unset
    // values, but get the timestamp from the current_state
    base::LinearAngular6DCommand command_out = reference;
    command_out.time = current_state.time;
    LinearAngular6DPIDState pid_state;
    pid_state.time = current_state.time;

    for (int i = 0; i < 3; ++i)
    {   // Linear case
        if (!base::isUnset(reference.linear(i)))
        {
            command_out.linear[i] =
                pid.linear[i].update(current_state.linear(i),
                                   reference.linear(i),
                                   current_state.time.toSeconds());
            pid_state.linear[i] = auv_control::PIDState(pid.linear[i].getState(), true);
        }
        else
            pid_state.linear[i].active = false;
        // Angular case
        if (!base::isUnset(reference.angular(i)))
        {
            double current_state_angular = current_state.angular(i);
            if (_control_angle_diff)
            {
                double diff = base::Angle::normalizeRad(
                    current_state.angular(i) - reference.angular(i));
                current_state_angular = reference.angular(i) + diff;
            }
            command_out.angular[i] =
                pid.angular[i].update(current_state_angular,
                                      reference.angular(i),
                                      current_state.time.toSeconds());
            pid_state.angular[i] = auv_control::PIDState(pid.angular[i].getState(), true);
        }
        else
            pid_state.angular[i].active = false;
    }
    return std::make_pair(command_out, pid_state);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See BasePIDController.hpp for more detailed
// documentation about them.

bool BasePIDController::configureHook()
{
    if (! BasePIDControllerBase::configureHook())
        return false;

    // set derivative mode
    motor_controller::DerivativeMode derivative_mode = motor_controller::Output;
    if(_apply_derivative_to_error.get())
        derivative_mode = motor_controller::Error;
    for (int i = 0; i < 3; ++i)
    {
        mPIDs.linear[i].setDerivativeMode(derivative_mode);
        mPIDs.angular[i].setDerivativeMode(derivative_mode);
    }

    setPid_settings(_pid_settings.get());
    setParallel_pid_settings(_parallel_pid_settings.get());
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
    // reset the contollers since the control loop is interrupted
    for (unsigned i = 0; i < 3; ++i)
    {
        mPIDs.linear[i].reset();
        mPIDs.angular[i].reset();
    }

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
bool BasePIDController::calcOutput(const LinearAngular6DCommandStatus &merged_command)
{
    std::pair<base::LinearAngular6DCommand, LinearAngular6DPIDState> cmd_status;
    // Compute the pair of output_command and pid_state and update mPIDs
    cmd_status = calcPIDStateCommand(merged_command.command, mCurrentState, mPIDs);

    base::LinearAngular6DCommand output_command = cmd_status.first;

    // Stop controling for axis with bad variance
    bool unsure_pose = false;
    for(unsigned i = 0; i < 3; i++){
        if(mCurrentCov.linear(i,i) > _variance_threshold.get()){
            output_command.linear(i) = 0;
            unsure_pose = true;
        }
        if(mCurrentCov.angular(i,i) > _variance_threshold.get()){
            output_command.angular(i) = 0;
            unsure_pose = true;
        }
    }

    _pid_state.write(cmd_status.second);
    _cmd_out.write(output_command);

    if(unsure_pose){
        if(state() != UNSURE_POSE_SAMPLE)
            state(UNSURE_POSE_SAMPLE);
        return false;
    }
    return true;
}

void BasePIDController::keepPosition(){
    base::LinearAngular6DCommand output_command;
    output_command.time = mCurrentState.time;
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
