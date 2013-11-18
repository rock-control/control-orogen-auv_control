/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_WORLDTOALIGNED_TASK_HPP
#define AUV_CONTROL_WORLDTOALIGNED_TASK_HPP

#include "auv_control/WorldToAlignedBase.hpp"
#include <math.h>
#include <float.h>

namespace auv_control {

    class WorldToAligned : public WorldToAlignedBase
    {
	friend class WorldToAlignedBase;
    protected:
        base::samples::RigidBodyState currentPose;

        void keep();
        bool calcOutput();

    public:
        WorldToAligned(std::string const& name = "auv_control::WorldToAligned");

        WorldToAligned(std::string const& name, RTT::ExecutionEngine* engine);

	    ~WorldToAligned();

        bool startHook();

        void updateHook();

    };
}

#endif

