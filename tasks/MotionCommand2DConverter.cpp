/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MotionCommand2DConverter.hpp"

using namespace auv_control;

MotionCommand2DConverter::MotionCommand2DConverter(std::string const& name, TaskCore::TaskState initial_state)
    : MotionCommand2DConverterBase(name, initial_state)
{
}

MotionCommand2DConverter::MotionCommand2DConverter(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : MotionCommand2DConverterBase(name, engine, initial_state)
{
}

MotionCommand2DConverter::~MotionCommand2DConverter()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See MotionCommand2DConverter.hpp for more detailed
// documentation about them.

bool MotionCommand2DConverter::configureHook()
{
    if (! MotionCommand2DConverterBase::configureHook())
        return false;
    return true;
}
bool MotionCommand2DConverter::startHook()
{
    if (! MotionCommand2DConverterBase::startHook())
        return false;
    return true;
}
void MotionCommand2DConverter::updateHook()
{
    base::MotionCommand2D input_cmd;
    base::samples::RigidBodyState pose_sample;

    MotionCommand2DConverterBase::updateHook();

    if(_pose_sample.read(pose_sample) == RTT::NoData){
        return;
    }

    if(_cmd_in.read(input_cmd) == RTT::NewData){
        base::LinearAngular6DCommand velocity_cmd;
        base::LinearAngular6DCommand position_cmd;
        
        velocity_cmd.stamp = base::Time::now();
        velocity_cmd.linear(0) = input_cmd.translation;

        position_cmd.stamp = base::Time::now();
        position_cmd.angular(2) = base::getYaw(pose_sample.orientation) + input_cmd.rotation;
        while(position_cmd.angular(2) > M_PI){
            position_cmd.angular(2) -= 2*M_PI;
        }
        while(position_cmd.angular(2) < -M_PI){
            position_cmd.angular(2) += 2*M_PI;
        }
        
        _position_cmd_out.write(position_cmd);
        _velocity_cmd_out.write(velocity_cmd);

    }
}
void MotionCommand2DConverter::errorHook()
{
    MotionCommand2DConverterBase::errorHook();
}
void MotionCommand2DConverter::stopHook()
{
    MotionCommand2DConverterBase::stopHook();
}
void MotionCommand2DConverter::cleanupHook()
{
    MotionCommand2DConverterBase::cleanupHook();
}
