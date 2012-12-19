/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_AUVFORCETORQUECONTROLLER_TASK_HPP
#define AUV_CONTROL_AUVFORCETORQUECONTROLLER_TASK_HPP

#include "auv_control/AUVForceTorqueControllerBase.hpp"
#include <Eigen/Dense>

namespace auv_control {

    class AUVForceTorqueController : public AUVForceTorqueControllerBase
    {
	friend class AUVForceTorqueControllerBase;
    protected:
    
    base::MatrixXd calibration;


    public:
        AUVForceTorqueController(std::string const& name = "auv_control::AUVForceTorqueController", TaskCore::TaskState initial_state = Stopped);

        AUVForceTorqueController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	~AUVForceTorqueController();

         bool startHook();

         void updateHook();

    };
}

#endif

