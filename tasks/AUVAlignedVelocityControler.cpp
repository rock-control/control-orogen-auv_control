/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVAlignedVelocityControler.hpp"

using namespace auv_control;

AUVAlignedVelocityControler::AUVAlignedVelocityControler(std::string const& name, TaskCore::TaskState initial_state)
    : AUVAlignedVelocityControlerBase(name, initial_state)
{
}

AUVAlignedVelocityControler::AUVAlignedVelocityControler(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVAlignedVelocityControlerBase(name, engine, initial_state)
{
}

AUVAlignedVelocityControler::~AUVAlignedVelocityControler()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AUVAlignedVelocityControler.hpp for more detailed
// documentation about them.

// bool AUVAlignedVelocityControler::configureHook()
// {
//     if (! AUVAlignedVelocityControlerBase::configureHook())
//         return false;
//     return true;
// }
// bool AUVAlignedVelocityControler::startHook()
// {
//     if (! AUVAlignedVelocityControlerBase::startHook())
//         return false;
//     return true;
// }
// void AUVAlignedVelocityControler::updateHook()
// {
//     AUVAlignedVelocityControlerBase::updateHook();
// }
// void AUVAlignedVelocityControler::errorHook()
// {
//     AUVAlignedVelocityControlerBase::errorHook();
// }
// void AUVAlignedVelocityControler::stopHook()
// {
//     AUVAlignedVelocityControlerBase::stopHook();
// }
// void AUVAlignedVelocityControler::cleanupHook()
// {
//     AUVAlignedVelocityControlerBase::cleanupHook();
// }

