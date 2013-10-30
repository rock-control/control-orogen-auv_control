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

void MotionCommand2DConverter::updateHook()
{
    MotionCommand2DConverterBase::updateHook();

    base::MotionCommand2D input_cmd;

    if(_cmd_in.read(input_cmd) == RTT::NewData)
    {
        base::LinearAngular6DCommand cmd;
        cmd.time = base::Time::now();
        cmd.x() = input_cmd.translation;
        cmd.yaw() = input_cmd.rotation;
        
        _cmd_out.write(cmd);
    }
}
