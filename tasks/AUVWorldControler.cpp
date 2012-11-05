/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVWorldControler.hpp"

using namespace auv_control;

AUVWorldControler::AUVWorldControler(std::string const& name, TaskCore::TaskState initial_state)
    : AUVWorldControlerBase(name, initial_state)
{
}

AUVWorldControler::AUVWorldControler(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVWorldControlerBase(name, engine, initial_state)
{
}

AUVWorldControler::~AUVWorldControler()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AUVWorldControler.hpp for more detailed
// documentation about them.

// bool AUVWorldControler::configureHook()
// {
//     if (! AUVWorldControlerBase::configureHook())
//         return false;
//     return true;
// }
// bool AUVWorldControler::startHook()
// {
//     if (! AUVWorldControlerBase::startHook())
//         return false;
//     return true;
// }
// void AUVWorldControler::updateHook()
// {
//     AUVWorldControlerBase::updateHook();
// }
// void AUVWorldControler::errorHook()
// {
//     AUVWorldControlerBase::errorHook();
// }
// void AUVWorldControler::stopHook()
// {
//     AUVWorldControlerBase::stopHook();
// }
// void AUVWorldControler::cleanupHook()
// {
//     AUVWorldControlerBase::cleanupHook();
// }

