/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_ALIGNEDPOSITIONCONTROLLER_TASK_HPP
#define AUV_CONTROL_ALIGNEDPOSITIONCONTROLLER_TASK_HPP

#include "auv_control/AlignedPositionControllerBase.hpp"

namespace auv_control {

    class AlignedPositionController : public AlignedPositionControllerBase
    {
	friend class AlignedPositionControllerBase;
    protected:
        bool on_start;
        motor_controller::PID linear_pid[3];
        motor_controller::PID angular_pid[3];
        base::Time last_pose_sample_time;
        base::LinearAngular6DPIDSettings last_pid_settings;

        void setPIDSettings(base::LinearAngular6DPIDSettings new_settings);
 
        void keepPosition();
        bool calcOutput();

        bool last[6];
        base::Time pos_start[6];
        double avg[6];
        int cnt[6];
    
    public:
        AlignedPositionController(std::string const& name = "auv_control::AlignedPositionController", TaskCore::TaskState initial_state = Stopped);

        AlignedPositionController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	    ~AlignedPositionController();

        bool startHook();

        void updateHook();

    };
}

#endif

