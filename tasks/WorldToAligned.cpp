/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WorldToAligned.hpp"

using namespace auv_control;

WorldToAligned::WorldToAligned(std::string const& name)
    : WorldToAlignedBase(name)
{
    _timeout_pose.set(base::Time::fromSeconds(1));
}

WorldToAligned::WorldToAligned(std::string const& name, RTT::ExecutionEngine* engine)
    : WorldToAlignedBase(name, engine)
{
    _timeout_pose.set(base::Time::fromSeconds(1));
}

WorldToAligned::~WorldToAligned()
{
}

bool WorldToAligned::configureHook()
{
    if (!WorldToAlignedBase::configureHook())
        return false;

    new_pose_samples_timeout = base::Timeout(_timeout_pose.get());

    return true;
}

bool WorldToAligned::startHook()
{
    on_init = true;
    if (!WorldToAlignedBase::startHook())
        return false;

    auv_control::ExpectedInputs expected_inputs = _expected_inputs.get();
    if (expected_inputs.linear[0] ^ expected_inputs.linear[1])
    {
        RTT::log(RTT::Error) << "the X and Y inputs must either be both given or not given at all" << RTT::endlog();
        return false;
    }

    new_pose_samples_timeout.restart();

    return true;
}
void WorldToAligned::updateHook()
{
    RTT::FlowStatus status = _pose_samples.readNewest(currentPose);
    if(status != RTT::NewData)
    {
        if(new_pose_samples_timeout.elapsed())
        {
            exception(POSE_TIMEOUT);
            return;
        }
        if(status == RTT::NoData)
        {
            state(WAIT_FOR_POSE_SAMPLE);
            return;
        }
    }
    else
        new_pose_samples_timeout.restart();


    if(!this->isPoseSampleValid(currentPose)){
        if(!on_init){
            exception(POSE_SAMPLE_INVALID);
        }
        return;
    }

    WorldToAlignedBase::updateHook();
    on_init = false;
}

void WorldToAligned::errorHook()
{
    WorldToAlignedBase::errorHook();
}

void WorldToAligned::keepPosition(){
    base::LinearAngular6DCommand output_command;
    output_command.time = currentPose.time;

    if(!_nan_on_keep_position.get()){
        output_command.linear(0) = 0;
        output_command.linear(1) = 0;
        output_command.linear(2) = 0;

        output_command.roll() = base::getRoll(currentPose.orientation);
        output_command.pitch() = base::getPitch(currentPose.orientation);
        output_command.yaw() = 0;
    }

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
}

bool WorldToAligned::calcOutput(const LinearAngular6DCommandStatus &merged_command){
    // In case there is an OldData, it should not output the same cmd again,
    // but no error occurred
    if (merged_command.status == RTT::OldData)
        return true;

    base::LinearAngular6DCommand output_command;

    base::Vector3d target_xyz = merged_command.command.linear;
    if (_position_control)
        target_xyz -= currentPose.position;

    double yaw = base::getYaw(currentPose.orientation);

    // Rotate the (x,y) part of the position target, copy the z part
    base::Vector3d target_xy  = target_xyz;
    target_xy.z() = 0;
    output_command.linear = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * target_xy;
    output_command.linear(2) = target_xyz(2);

    output_command.angular = merged_command.command.angular;
    if (_position_control)
    {
        // And shift the yaw target by the current yaw (leaving pitch and roll)
        output_command.angular(2) = output_command.angular(2) - yaw;
        while(output_command.angular(2) > M_PI)
            output_command.angular(2)-=(2*M_PI);
        while(output_command.angular(2) < -M_PI)
            output_command.angular(2)+=(2*M_PI);

    }
    else if (_ang_vel_euler_rate.get())
    {
        output_command.angular = base::eulerRate2AngularVelocity(
            output_command.angular.reverse(), currentPose.orientation
        );
    }

    // Finally, set the timestamp of the output
    output_command.time = merged_command.command.time;
    _cmd_out.write(output_command);
    return true;
}

bool WorldToAligned::isPoseSampleValid(base::samples::RigidBodyState pose){
    if((!_safe_mode.get()) &&
            (!base::samples::RigidBodyState::isValidValue(pose.position) ||
             (!base::samples::RigidBodyState::isValidValue(pose.orientation)))){
        return false;
    } else {
        auv_control::ExpectedInputs expected_inputs = _expected_inputs.get();
        if((expected_inputs.linear[0] || expected_inputs.linear[1]) &&
                (base::isUnset<double>(pose.position[0]) ||
                 base::isUnset<double>(pose.position[1]) ||
                 !base::samples::RigidBodyState::isValidValue(pose.orientation))){
            return false;
        }

        if(expected_inputs.linear[2] && base::isUnset<double>(pose.position[2])){
            return false;
        }

        if((expected_inputs.angular[0] || expected_inputs.angular[1] || expected_inputs.angular[2]) &&
                !base::samples::RigidBodyState::isValidValue(pose.orientation)){
            return false;
        }

    }
    return true;
}
