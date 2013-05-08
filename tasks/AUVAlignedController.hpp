/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_AUVALIGNEDCONTROLLER_TASK_HPP
#define AUV_CONTROL_AUVALIGNEDCONTROLLER_TASK_HPP

#include "auv_control/AUVAlignedControllerBase.hpp"

namespace auv_control {

    class AUVAlignedController : public AUVAlignedControllerBase
    {
	friend class AUVAlignedControllerBase;
    protected:
        bool on_start;
        motor_controller::PID linear_pid[3];
        motor_controller::PID angular_pid[3];
        base::Time last_pose_sample_time;
        base::LinearAngular6DPIDSettings last_pid_settings;

        void setPIDSettings(base::LinearAngular6DPIDSettings new_settings);
 
        void doNothing();
        bool calcOutput();

        bool last[6];
        base::Time pos_start[6];
        double avg[6];
        int cnt[6];
    
    public:
        AUVAlignedController(std::string const& name = "auv_control::AUVAlignedController", TaskCore::TaskState initial_state = Stopped);

        AUVAlignedController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	    ~AUVAlignedController();

        bool startHook();

        void updateHook();

    };
}

#endif

