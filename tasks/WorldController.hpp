/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_WORLDCONTROLLER_TASK_HPP
#define AUV_CONTROL_WORLDCONTROLLER_TASK_HPP

#include "auv_control/WorldControllerBase.hpp"
#include <math.h>
#include <float.h>

namespace auv_control {

    class WorldController : public WorldControllerBase
    {
	friend class WorldControllerBase;
    protected:
        bool on_start;

        void keepPosition();
        bool calcOutput();

    public:
        WorldController(std::string const& name = "auv_control::WorldController", TaskCore::TaskState initial_state = Stopped);

        WorldController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	    ~WorldController();

        bool startHook();

        void updateHook();

    };
}

#endif

