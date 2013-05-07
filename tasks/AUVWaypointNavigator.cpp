/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "AUVWaypointNavigator.hpp"

using namespace auv_control;

AUVWaypointNavigator::AUVWaypointNavigator(std::string const& name, TaskCore::TaskState initial_state)
    : AUVWaypointNavigatorBase(name, initial_state)
{
}

AUVWaypointNavigator::AUVWaypointNavigator(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : AUVWaypointNavigatorBase(name, engine, initial_state)
{
}

AUVWaypointNavigator::~AUVWaypointNavigator()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See AUVWaypointNavigator.hpp for more detailed
// documentation about them.




/*bool AUVWaypointNavigator::configureHook()
{
    
    if (! RTT::TaskContext::configureHook())
        return false;
    

    

    
    return true;
    
}



bool AUVWaypointNavigator::startHook()
{
    
    if (! RTT::TaskContext::startHook())
        return false;
    

    

    
    return true;
    
}*/



void AUVWaypointNavigator::updateHook()
{
    
    RTT::TaskContext::updateHook();
 
    base::samples::RigidBodyState pose;
    std::vector<base::LinearAngular6DWaypoint> trajectory;

    if(_pose_sample.read(pose) == RTT::NoData) {
        //TODO: Fehler werfen
        std::cout << "POSE_SAMPLE MISSING" << std::endl;
        return;
    }
    last_pose = pose;

    if(_trajectory.read(trajectory) != RTT::NoData) {
        waypoints.clear();
        waypoints.resize(trajectory.size());
        std::copy(trajectory.begin(), trajectory.end(), waypoints.begin());

        //keeping_position = false;
         
        _trajectory.clear();
    }
    
    if(!waypoints.empty()){
        base::LinearAngular6DWaypoint wp;
        base::LinearAngular6DCommand delta;
        
        wp = waypoints.front();
        wp.cmd.stamp = base::Time::now();
        _cmd_out.write(wp.cmd);
        
        delta.stamp = base::Time::now();
        delta.linear = wp.cmd.linear - pose.position;
        delta.angular(0) = wp.cmd.angular(0) - base::getRoll(pose.orientation);
        delta.angular(1) = wp.cmd.angular(1) - base::getPitch(pose.orientation);
        delta.angular(2) = wp.cmd.angular(2) - base::getYaw(pose.orientation);
        _current_delta.write(delta);
        if (delta.linear.norm() <= wp.linear_tolerance && delta.angular.norm() <= wp.angular_tolerance){
            waypoints.pop_front();
        }
        _queue_size.write(waypoints.size());
        _current_waypoint.write(wp);

    }

    return;

    

    
}



void AUVWaypointNavigator::errorHook()
{
    
    RTT::TaskContext::errorHook();
    

    

    
}



void AUVWaypointNavigator::stopHook()
{
    
    RTT::TaskContext::stopHook();
    

    

    
}



void AUVWaypointNavigator::cleanupHook()
{
    
    RTT::TaskContext::cleanupHook();
    

    

    
}

