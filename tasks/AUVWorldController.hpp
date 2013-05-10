/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_AUVWORLDCONTROLLER_TASK_HPP
#define AUV_CONTROL_AUVWORLDCONTROLLER_TASK_HPP

#include "auv_control/AUVWorldControllerBase.hpp"
#include <math.h>
#include <float.h>

namespace auv_control {

    class AUVWorldController : public AUVWorldControllerBase
    {
	friend class AUVWorldControllerBase;
    protected:
        bool on_start;

        void holdPosition();
        bool calcOutput();

    public:
        AUVWorldController(std::string const& name = "auv_control::AUVWorldController", TaskCore::TaskState initial_state = Stopped);

        AUVWorldController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	    ~AUVWorldController();

        bool startHook();

        void updateHook();

    };
}

#endif

