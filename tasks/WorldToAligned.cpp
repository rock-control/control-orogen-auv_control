/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WorldToAligned.hpp"

using namespace auv_control;

WorldToAligned::WorldToAligned(std::string const& name)
    : WorldToAlignedBase(name)
{
}

WorldToAligned::WorldToAligned(std::string const& name, RTT::ExecutionEngine* engine)
    : WorldToAlignedBase(name, engine)
{
}

WorldToAligned::~WorldToAligned()
{
}

bool WorldToAligned::startHook()
{
    if (!Base::startHook())
        return false;

    auv_control::ExpectedInputs expected_inputs = _expected_inputs.get();
    if (expected_inputs.linear[0] ^ expected_inputs.linear[1])
    {
        RTT::log(RTT::Error) << "the X and Y inputs must either be both given or not given at all" << RTT::endlog();
        return false;
    }

    return true;
}
void WorldToAligned::updateHook()
{
    if (_pose_samples.read(currentPose) == RTT::NoData){
        if(state() != WAIT_FOR_POSE_SAMPLE){
            error(WAIT_FOR_POSE_SAMPLE);
        }
        return;
    }

    WorldToAlignedBase::updateHook();
}

void WorldToAligned::errorHook()
{
    if( state() == WAIT_FOR_POSE_SAMPLE){
        base::samples::RigidBodyState pose_sample;
        if (_pose_samples.read(pose_sample) != RTT::NoData){
            recover();
        }
    }

    WorldToAligned::errorHook();
}

void WorldToAligned::keep(){
    base::LinearAngular6DCommand output_command;
    output_command.time = currentPose.time;

    output_command.linear(0) = 0; 
    output_command.linear(1) = 0; 
    output_command.linear(2) = 0;

    output_command.roll() = base::getRoll(currentPose.orientation);
    output_command.pitch() = base::getPitch(currentPose.orientation);
    output_command.yaw() = base::getYaw(currentPose.orientation);

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
    base::LinearAngular6DCommand output_command;

    base::Vector3d target_xyz = merged_command.linear;
    if (_position_control)
        target_xyz -= currentPose.position;

    double yaw = base::getYaw(currentPose.orientation);

    // Rotate the (x,y) part of the position target, copy the z part
    base::Vector3d target_xy  = target_xyz;
    target_xy.z() = 0;
    output_command.linear = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) * target_xy;
    output_command.linear(2) = target_xyz(2);

    output_command.angular = merged_command.angular;
    if (_position_control)
    {
        // And shift the yaw target by the current yaw (leaving pitch and roll)
        output_command.angular(2) = output_command.angular(2) - yaw;
        while(output_command.angular(2) > M_PI)
            output_command.angular(2)-=(2*M_PI);
        while(output_command.angular(2) < -M_PI)
            output_command.angular(2)+=(2*M_PI);

    }
        
    // Finally, set the timestamp of the output
    output_command.time = merged_command.time;
    _cmd_out.write(output_command);
    return true;
}

