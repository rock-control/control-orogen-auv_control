/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_BASE_TASK_HPP
#define AUV_CONTROL_BASE_TASK_HPP

#include "auv_control/BaseBase.hpp"
#include <string.h>
#include <base/float.h>

namespace auv_control {

    class Base : public BaseBase
    {
	friend class BaseBase;
    protected:
        typedef RTT::InputPort<base::LinearAngular6DCommand> InputPortType;
        struct InputPortInfo{
            std::string name;
            double timeout;
            base::Time last_time;
            InputPortType *input_port;
            InputPortInfo()
                :input_port(0){}
        };    
    
        std::vector<InputPortInfo> input_ports;
        base::LinearAngular6DCommand merged_command;        
        base::LinearAngular6DCommand output_command;        
        base::samples::RigidBodyState pose_sample;

        bool gatherInputCommand();
        bool getPoseSample();        
        void genDefaultInput();
        void setDefaultTimeout();

        virtual void addCommandInput(::std::string const & name, double timeout);
        
        virtual void keepPosition();
        virtual bool calcOutput();
        
    public:
        /** TaskContext constructor for Base
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Base(std::string const& name = "auv_control::Base", TaskCore::TaskState initial_state = Stopped);

        /** TaskContext constructor for Base 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Base(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

        /** Default deconstructor of Base
         */
	~Base();

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
    private:
        bool merge(bool const expected[], base::Vector3d const& current, base::Vector3d& merged);
        
 
    };
}

#endif

