/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_BASEPIDCONTROLLER_TASK_HPP
#define AUV_CONTROL_BASEPIDCONTROLLER_TASK_HPP

#include "auv_control/BasePIDControllerBase.hpp"
#include <motor_controller/PID.hpp>

namespace auv_control {

    /*! \class BasePIDController 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Base implementation of all the tasks that use one PID controller per axis to
generate commands
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','auv_control::BasePIDController')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class BasePIDController : public BasePIDControllerBase
    {
	friend class BasePIDControllerBase;
    protected:

        struct currentState{
            base::Vector3d linear;
            base::Vector3d angular;
            base::Time time;
        };
        struct currentCov{
            base::Matrix3d linear;
            base::Matrix3d angular;
            base::Time time;
        };
        /** The current system's state, including the linear and angular part
         * It must be updated by subclasses before calling their base updateHook
         * class
         */
        currentState mCurrentState;
        /** The current system state's covariance, including the linear and angular part
         * It must be updated by subclasses before calling their base updateHook
         * class
         */
        currentCov mCurrentCov;
        /** The PIDs, including the linear and angular part
         * It need to have it states updated
         */
        LinearAngular6DPID mPIDs;

        bool calcOutput(const LinearAngular6DCommandStatus &merged_command);
        void keepPosition();


        virtual bool setParallel_pid_settings(::base::LinearAngular6DParallelPIDSettings const & value);
        virtual bool setPid_settings(::base::LinearAngular6DPIDSettings const & value);

        /** Compute PID control and it status for 6 DOF (linear and angular)
         *
         * @param reference, the disered state
         * @param current_state, measured state
         * @param pid, the controller used and be updated
         * @return pair of LinearAngular6D Command and PIDStates
         */
         std::pair<base::LinearAngular6DCommand, LinearAngular6DPIDState> calcPIDStateCommand(
            const base::LinearAngular6DCommand &reference, const currentState &current_state, LinearAngular6DPID &pid);

        /** This is used to control the orientation in the world frame by avoiding
         *  the problem of pi=-pi.
        */
        bool _control_angle_diff = false;

    public:
        /** TaskContext constructor for BasePIDController
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        BasePIDController(std::string const& name = "auv_control::BasePIDController");

        /** TaskContext constructor for BasePIDController 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        BasePIDController(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of BasePIDController
         */
	~BasePIDController();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();
    };
}

#endif
