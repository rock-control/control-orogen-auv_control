/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PIDController.hpp"

using namespace auv_control;

PIDController::PIDController(std::string const& name)
    : PIDControllerBase(name)
{
    _variance_threshold.set(base::infinity<double>());
    _timeout_pose.set(base::Time::fromSeconds(1));
}

PIDController::PIDController(std::string const& name, RTT::ExecutionEngine* engine)
    : PIDControllerBase(name, engine)
{
    _variance_threshold.set(base::infinity<double>());
    _timeout_pose.set(base::Time::fromSeconds(1));
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

    new_pose_samples_timeout = base::Timeout(_timeout_pose.get());

    return true;
}
bool PIDController::startHook()
{
    if (! PIDControllerBase::startHook())
        return false;

    new_pose_samples_timeout.restart();

    return true;
}
void PIDController::updateHook()
{
    base::samples::RigidBodyState pose_sample;
    RTT::FlowStatus status = _pose_samples.readNewest(pose_sample);
    if (status != RTT::NewData)
    {
        if(new_pose_samples_timeout.elapsed())
            exception(POSE_TIMEOUT);
        return;
    }
    new_pose_samples_timeout.restart();

    mCurrentCov.time = pose_sample.time;
    mCurrentState.time = pose_sample.time;
    if (_position_control)
    {
        mCurrentCov.angular = pose_sample.cov_orientation;
        mCurrentCov.linear = pose_sample.cov_position;

        if(mCurrentCov.linear(0,0) > mCurrentCov.linear(1,1)){
            mCurrentCov.linear(1,1) = mCurrentCov.linear(0,0);
        }
        if(mCurrentCov.linear(1,1) > mCurrentCov.linear(0,0)){
            mCurrentCov.linear(0,0) = mCurrentCov.linear(1,1);
        }

        // position control
        if (_world_frame)
        {
            _control_angle_diff = true; //member of the base class used to eliminate the pi=-pi singularity

            mCurrentState.linear = pose_sample.position;
            mCurrentState.angular = base::Vector3d(
                base::getRoll(pose_sample.orientation),
                base::getPitch(pose_sample.orientation),
                base::getYaw(pose_sample.orientation));
        }
        else
        {
            mCurrentState.linear = base::Vector3d::Zero();
            mCurrentState.angular = base::Vector3d(
                base::getRoll(pose_sample.orientation),
                base::getPitch(pose_sample.orientation),
                0);
        }
    }
    else
    {
        mCurrentCov.linear = pose_sample.cov_velocity;
        mCurrentCov.angular = pose_sample.cov_angular_velocity;
        mCurrentState.angular = pose_sample.angular_velocity;

        // velocity control
        if (_world_frame){
            mCurrentState.linear = pose_sample.velocity;
        }
        else
        {
            double yaw = base::getYaw(pose_sample.orientation);
            mCurrentState.linear = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()) *
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
                base::isUnset<double>(mCurrentState.linear[i])){
            return false;
        }

        if (_expected_inputs.get().angular[i] &&
                base::isUnset<double>(mCurrentState.angular[i])){
            return false;
        }
    }
    return true;
}
