/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WaypointNavigator.hpp"

using namespace auv_control;

WaypointNavigator::WaypointNavigator(std::string const& name)
    : WaypointNavigatorBase(name)
{
}

WaypointNavigator::WaypointNavigator(std::string const& name, RTT::ExecutionEngine* engine)
    : WaypointNavigatorBase(name, engine)
{
}

WaypointNavigator::~WaypointNavigator()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See WaypointNavigator.hpp for more detailed
// documentation about them.

void WaypointNavigator::updateHook()
{
    
    RTT::TaskContext::updateHook();
 
    base::samples::RigidBodyState pose;
    std::vector<base::LinearAngular6DWaypoint> trajectory;

    if(_pose_sample.readNewest(pose) == RTT::NoData) {
        if(state() != POSE_SAMPLE_MISSING){
            state(POSE_SAMPLE_MISSING);
        }
        return;
    }
    last_pose = pose;

    if(_trajectory.read(trajectory) == RTT::NewData) {
        waypoints.clear();
        waypoints.resize(trajectory.size());
        std::copy(trajectory.begin(), trajectory.end(), waypoints.begin());
        keep_position = false;
        if(state() != FOLLOWING_WAYPOINTS){
            state(FOLLOWING_WAYPOINTS);
        }
    }
    
    base::LinearAngular6DWaypointInfo wpi;
    if(!waypoints.empty()){
        base::LinearAngular6DCommand delta;
        
        wp = waypoints.front();
        
        delta.time = base::Time::now();
        delta.linear = wp.cmd.linear - pose.position;
        delta.angular(0) = wp.cmd.angular(0) - base::getRoll(pose.orientation);
        delta.angular(1) = wp.cmd.angular(1) - base::getPitch(pose.orientation);
        delta.angular(2) = wp.cmd.angular(2) - base::getYaw(pose.orientation);
        wpi.current_delta = delta;
        if (delta.linear.norm() <= wp.linear_tolerance && delta.angular.norm() <= wp.angular_tolerance){
            if (state() != KEEP_WAYPOINT){
                state(KEEP_WAYPOINT);
                start_keeping = base::Time::now();
            } 
            if (last_pose.time.toSeconds()-start_keeping.toSeconds() >= wp.hold_time || wp.hold_time == 0){
                waypoints.pop_front();
                if (waypoints.size() == 0){
                    keep_position = true;
                    if(state() != FINISHED){
                        state(FINISHED);
                    }
                } else {
                    state(FOLLOWING_WAYPOINTS);
                }
            }
        }
    } else if(!keep_position && (state() != WAIT_FOR_WAYPOINTS)) {
        state(WAIT_FOR_WAYPOINTS);
    }
    
    
    wp.cmd.time = base::Time::now();
    //write the command
    _cmd_out.write(wp.cmd);

    
    wpi.queue_size = waypoints.size();
    wpi.current_waypoint = wp;
    //write infos about the Waypoints
    _waypoint_info.write(wpi);

    return;
}

void WaypointNavigator::errorHook()
{
    
    RTT::TaskContext::errorHook();
    

    

    
}



void WaypointNavigator::stopHook()
{
    
    RTT::TaskContext::stopHook();
    

    

    
}



void WaypointNavigator::cleanupHook()
{
    
    RTT::TaskContext::cleanupHook();
    

    

    
}
