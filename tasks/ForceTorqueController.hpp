/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_FORCETORQUECONTROLLER_TASK_HPP
#define AUV_CONTROL_FORCETORQUECONTROLLER_TASK_HPP

#include "auv_control/ForceTorqueControllerBase.hpp"
#include <Eigen/Dense>

namespace auv_control {

    class ForceTorqueController : public ForceTorqueControllerBase
    {
	friend class ForceTorqueControllerBase;
    protected:
    
    base::MatrixXd calibration;


    public:
        ForceTorqueController(std::string const& name = "auv_control::ForceTorqueController", TaskCore::TaskState initial_state = Stopped);

        ForceTorqueController(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	~ForceTorqueController();

         bool startHook();

         void updateHook();

    };
}

#endif

