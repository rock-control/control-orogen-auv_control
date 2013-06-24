/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_ALIGNEDCONTROLLER_TASK_HPP
#define AUV_CONTROL_ALIGNEDCONTROLLER_TASK_HPP

#include "auv_control/AlignedControllerBase.hpp"

namespace auv_control {

    class AlignedController : public AlignedControllerBase
    {
	friend class AlignedControllerBase;
    protected:
        bool on_start;
        motor_controller::PID linear_pid[3];
        motor_controller::PID angular_pid[3];
        base::Time last_pose_sample_time;
        base::LinearAngular6DPIDSettings last_pid_settings;

        void setPIDSettings(base::LinearAngular6DPIDSettings new_settings);
 
        void holdPosition();
        bool calcOutput();

        bool last[6];
        base::Time pos_start[6];
        double avg[6];
        int cnt[6];
    
    public:
        AlignedController(std::string const& name = "auv_control::AlignedController", TaskCore::TaskState initial_state = Stopped);

        AlignedController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	    ~AlignedController();

        bool startHook();

        void updateHook();

    };
}

#endif

