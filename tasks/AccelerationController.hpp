/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_ACCELERATIONCONTROLLER_TASK_HPP
#define AUV_CONTROL_ACCELERATIONCONTROLLER_TASK_HPP

#include "auv_control/AccelerationControllerBase.hpp"
#include <base/commands/Joints.hpp>

namespace auv_control {

    class AccelerationController : public AccelerationControllerBase
    {
	friend class AccelerationControllerBase;
    protected:
        base::MatrixXd thrusterMatrix;
        base::MatrixXd weightedPseudoInverse;
        base::VectorXd inputVector;
        base::VectorXd cmdVector;
        base::VectorXd expectedEffortVector;
        boost::shared_ptr<Eigen::JacobiSVD<Eigen::MatrixXd> > svd;
        base::commands::Joints jointCommand;
        std::vector<base::JointState::MODE> controlModes;
        std::vector<std::string> names;
        base::JointLimits limits;

        bool calcOutput(const LinearAngular6DCommandStatus &merged_command);

    public:
        /** TaskContext constructor for AccelerationController
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        AccelerationController(std::string const& name = "auv_control::AccelerationController");

        /** TaskContext constructor for AccelerationController 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        AccelerationController(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of AccelerationController
         */
	~AccelerationController();

        bool startHook();
        bool configureHook();
    };
}

#endif

