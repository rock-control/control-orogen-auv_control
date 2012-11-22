/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_AUVALIGNEDVELOCITYCONTROLER_TASK_HPP
#define AUV_CONTROL_AUVALIGNEDVELOCITYCONTROLER_TASK_HPP

#include "auv_control/AUVAlignedVelocityControlerBase.hpp"
#include <Eigen/Dense>
#include <base/time.h>
#include <motor_controller/PID.hpp>
#include <base/actuators/commands.h>

namespace auv_control {

    /*! \class AUVAlignedVelocityControler 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','auv_control::AUVAlignedVelocityControler')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class AUVAlignedVelocityControler : public AUVAlignedVelocityControlerBase
    {
	friend class AUVAlignedVelocityControlerBase;
    protected:
    
    control::AlignedVelocityCommand6D mergeCommands(control::AlignedVelocityCommand6D, control::AlignedVelocityCommand6D);
    void lastTarget(control::AlignedVelocityCommand6D);
    void holdPosition(control::AlignedVelocityCommand6D);
    void genVector(control::AlignedVelocityCommand6D, base::Vector3d, base::Vector3d, double);
    void stopActuators();
    base::actuators::Command genMotionCommand();
    
    base::Vector6d vector_command;
    base::Matrix6d calibration;
    
    control::AlignedVelocityCommand6D current;
    double target_x;
    double target_y;
    double target_z;
    double target_yaw;
    double target_pitch;
    double target_roll;
   
    //motor_controller::PIDSettings x_settings;
    motor_controller::PID x_pid;
    motor_controller::PID y_pid;
    motor_controller::PID z_pid;
    motor_controller::PID yaw_pid;
    motor_controller::PID pitch_pid;
    motor_controller::PID roll_pid;
    
    base::Time last_body_state_time;
    public:
        /** TaskContext constructor for AUVAlignedVelocityControler
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        AUVAlignedVelocityControler(std::string const& name = "auv_control::AUVAlignedVelocityControler", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for AUVAlignedVelocityControler 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        AUVAlignedVelocityControler(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of AUVAlignedVelocityControler
         */
	~AUVAlignedVelocityControler();

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

