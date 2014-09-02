/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PIDController.hpp"

using namespace auv_control;

PIDController::PIDController(std::string const& name)
    : PIDControllerBase(name)
{
}

PIDController::PIDController(std::string const& name, RTT::ExecutionEngine* engine)
    : PIDControllerBase(name, engine)
{
}

PIDController::~PIDController()
{
    _variance_threshold.set(base::infinity<double>());
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PIDController.hpp for more detailed
// documentation about them.

bool PIDController::configureHook()
{
    if (! PIDControllerBase::configureHook())
        return false;
    return true;
}
bool PIDController::startHook()
{
    if (! PIDControllerBase::startHook())
        return false;
    return true;
}
void PIDController::updateHook()
{
    base::samples::RigidBodyState pose_sample;
    if (_pose_samples.read(pose_sample) == RTT::NoData){
        if(state() != WAIT_FOR_POSE_SAMPLE){
            error(WAIT_FOR_POSE_SAMPLE);
        }
        return;
    }

    if (_position_control)
    {
        currentLinearCov = pose_sample.cov_position;

        if(currentLinearCov(0,0) > currentLinearCov(1,1)){
            currentLinearCov(1,1) = currentLinearCov(0,0);
        }
        if(currentLinearCov(1,1) > currentLinearCov(0,0)){
            currentLinearCov(0,0) = currentLinearCov(1,1);
        }



        currentAngularCov = pose_sample.cov_orientation;
        
        // position control
        if (_world_frame)
        {
            currentLinear = pose_sample.position;
            currentAngular = base::Vector3d(
                base::getRoll(pose_sample.orientation),
                base::getPitch(pose_sample.orientation),
                base::getYaw(pose_sample.orientation));
        }
        else
        {
            currentLinear = base::Vector3d::Zero();
            currentAngular = base::Vector3d(
                base::getRoll(pose_sample.orientation),
                base::getPitch(pose_sample.orientation),
                0);
        }
    }
    else
    {
        currentLinearCov = pose_sample.cov_velocity;
        currentAngularCov = pose_sample.cov_angular_velocity;
        currentAngular = pose_sample.angular_velocity;

        // velocity control
        if (_world_frame){
            currentLinear = pose_sample.velocity;
        } 
        else
        {
            double yaw = base::getYaw(pose_sample.orientation);
            currentLinear = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) *
                pose_sample.velocity;
        }
    }

    PIDControllerBase::updateHook();
}
void PIDController::errorHook()
{
    if( state() == WAIT_FOR_POSE_SAMPLE){
        base::samples::RigidBodyState pose_sample;
        if (_pose_samples.read(pose_sample) != RTT::NoData){
            recover();
        }
    }

    PIDControllerBase::errorHook();
}
void PIDController::stopHook()
{
    PIDControllerBase::stopHook();
}
void PIDController::cleanupHook()
{
    PIDControllerBase::cleanupHook();
}
