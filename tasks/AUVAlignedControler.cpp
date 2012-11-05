/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVAlignedControler.hpp"

using namespace auv_control;

AUVAlignedControler::AUVAlignedControler(std::string const& name, TaskCore::TaskState initial_state)
    : AUVAlignedControlerBase(name, initial_state)
{
}

AUVAlignedControler::AUVAlignedControler(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVAlignedControlerBase(name, engine, initial_state)
{
}

AUVAlignedControler::~AUVAlignedControler()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AUVAlignedControler.hpp for more detailed
// documentation about them.

// bool AUVAlignedControler::configureHook()
// {
//     if (! AUVAlignedControlerBase::configureHook())
//         return false;
//     return true;
// }
// bool AUVAlignedControler::startHook()
// {
//     if (! AUVAlignedControlerBase::startHook())
//         return false;
//     return true;
// }
// void AUVAlignedControler::updateHook()
// {
//     AUVAlignedControlerBase::updateHook();
// }
// void AUVAlignedControler::errorHook()
// {
//     AUVAlignedControlerBase::errorHook();
// }
// void AUVAlignedControler::stopHook()
// {
//     AUVAlignedControlerBase::stopHook();
// }
// void AUVAlignedControler::cleanupHook()
// {
//     AUVAlignedControlerBase::cleanupHook();
// }

