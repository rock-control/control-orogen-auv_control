/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PIDController.hpp"

using namespace auv_control;

PIDController::PIDController(std::string const& name)
    : PIDControllerBase(name)
{
    _variance_threshold.set(base::infinity<double>());
}

PIDController::PIDController(std::string const& name, RTT::ExecutionEngine* engine)
    : PIDControllerBase(name, engine)
{
    _variance_threshold.set(base::infinity<double>());
}

PIDController::~PIDController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PIDController.hpp for more detailed
// documentation about them.

bool PIDController::configureHook()
{
    if (! PIDControllerBase::configureHook())
        return false;

    new_pose_samples_timeout = base::Timeout(base::Time::fromSeconds(_timeout_in.value()));

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
    RTT::FlowStatus status = _pose_samples.readNewest(pose_sample);
    if (status != RTT::NewData)
    {
        if(new_pose_samples_timeout.elapsed())
            exception(POSE_TIMEOUT);
        return;
    }
    new_pose_samples_timeout.restart();


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

    if(!this->isPoseSampleValid()){
        exception(POSE_SAMPLE_INVALID);
        return;
    }

    PIDControllerBase::updateHook();
}
void PIDController::errorHook()
{
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

bool PIDController::isPoseSampleValid(){
    for(int i = 0; i < 3; i++){
        if (_expected_inputs.get().linear[i] &&
                base::isUnset<double>(currentLinear[i])){
            return false;
        }
        
        if (_expected_inputs.get().angular[i] &&
                base::isUnset<double>(currentAngular[i])){
            return false;
        }
    }
    return true;
}




