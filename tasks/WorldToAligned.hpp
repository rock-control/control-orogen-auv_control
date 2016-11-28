/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_WORLDTOALIGNED_TASK_HPP
#define AUV_CONTROL_WORLDTOALIGNED_TASK_HPP

#include "auv_control/WorldToAlignedBase.hpp"
#include <math.h>
#include <float.h>
#include <base/Timeout.hpp>

namespace auv_control {

    class WorldToAligned : public WorldToAlignedBase
    {
	friend class WorldToAlignedBase;
    protected:
        base::samples::RigidBodyState currentPose;
        bool on_init;
        base::Timeout new_pose_samples_timeout;

        void keepPosition();
        bool calcOutput(const LinearAngular6DCommandStatus &merged_command);
        bool isPoseSampleValid(base::samples::RigidBodyState pose);

    public:
        WorldToAligned(std::string const& name = "auv_control::WorldToAligned");

        WorldToAligned(std::string const& name, RTT::ExecutionEngine* engine);

	    ~WorldToAligned();

        bool configureHook();

        bool startHook();

        void updateHook();

        void errorHook();

    };
}

#endif

