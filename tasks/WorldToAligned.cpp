/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WorldToAligned.hpp"

using namespace auv_control;

WorldToAligned::WorldToAligned(std::string const& name, TaskCore::TaskState initial_state)
    : WorldToAlignedBase(name, initial_state)
{
}

WorldToAligned::WorldToAligned(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : WorldToAlignedBase(name, engine, initial_state)
{
}

WorldToAligned::~WorldToAligned()
{
}

bool WorldToAligned::startHook()
{
    auv_control::Base::startHook();
    if (_expected_inputs.linear[0] ^ _expected_inputs.linear[1])
        RTT::log(Error) << "the X and Y inputs must either be both given or not given at all" << RTT::endlog();

    return true;
}
void WorldToAligned::updateHook()
{
    if (pose_samples.read(currentPose) == RTT::NoData)
        return;

    auv_control::Base::updateHook();
}

void WorldToAligned::keep(){
    LinearAngular6DCommand output_command;
    output_command.stamp = pose_sample.time;

    output_command.linear(0) = 0; 
    output_command.linear(1) = 0; 
    output_command.linear(2) = 0;

    output_command.angular(0) = base::getRoll(pose_sample.orientation);
    output_command.angular(1) = base::getPitch(pose_sample.orientation);
    output_command.angular(2) = base::getYaw(pose_sample.orientation);

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

bool WorldToAligned::calcOutput(){
    LinearAngular6DCommand output_command;

    base::Vector3d target_xyz = merged_command.linear;
    if (_position_control)
        target_xyz -= pose_sample.position;

    // Rotate the (x,y) part of the position target, copy the z part
    base::Vector3d target_xy  = target_xyz;
    target_xy.z() = 0;
    output_command.linear = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * target_xy;
    output_command.linear(2) = target_xyz.linear(2);

    output_command.angular = merged_command.angular;
    if (_position_control)
    {
        // And shift the yaw target by the current yaw (leaving pitch and roll)
        double yaw = base::getYaw(currentPose.orientation);
        output_command.angular(2) = output_command.angular(2) - yaw;
    }
        
    // Finally, set the timestamp of the output
    output_command.stamp = merged_command.stamp;
    _cmd_out.write(output_command);
    return true;
}

