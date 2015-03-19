/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "MotionCommand2DConverter.hpp"

using namespace auv_control;

MotionCommand2DConverter::MotionCommand2DConverter(std::string const& name)
    : MotionCommand2DConverterBase(name)
{
}

MotionCommand2DConverter::MotionCommand2DConverter(std::string const& name, RTT::ExecutionEngine* engine)
    : MotionCommand2DConverterBase(name, engine)
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

    base::commands::Motion2D input_cmd;

    if(_cmd_in.readNewest(input_cmd) == RTT::NewData)
    {
        base::LinearAngular6DCommand cmd;
        cmd.time = base::Time::now();
        cmd.x() = input_cmd.translation;
        cmd.yaw() = input_cmd.rotation;
        
        _cmd_out.write(cmd);
    }
}
