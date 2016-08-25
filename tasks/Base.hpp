/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_BASE_TASK_HPP
#define AUV_CONTROL_BASE_TASK_HPP

#include "auv_control/BaseBase.hpp"
#include <string.h>

namespace auv_control {

    class Base : public BaseBase
    {
	friend class BaseBase;
    protected:
        typedef RTT::InputPort<base::LinearAngular6DCommand> InputPortType;
        struct InputPortInfo{
            std::string name;
            double timeout;
            base::Time last_sample_time;
            base::Time last_system_time;
            InputPortType *input_port;
            InputPortInfo()
                :input_port(0){}
        };    
    
        std::vector<InputPortInfo> input_ports;
        base::LinearAngular6DCommand merged_command;        

        void registerInput(std::string const& name, int timeout, InputPortType* input_port);
        InputPortType* deregisterInput(std::string const& name);
        base::Time newestCommandTime;

        /** Check for timeout in input ports
         *
         * Check both in system time (time.now()) and sample time (newest_command)
         * In case of Timeout, switch to exception state TIMEOUT
         * @param newest_command. Time of last command in all ports
         * @return bool. True if there is no Timeout. False in Timeout case
         */
        bool verifyTimeout(const base::Time &newest_command);

        /** Check if all expected data field are present
         *
         * In case of missing any data field, switch to INPUT_MISSING state
         * @param expect, the expected fields that need to present
         * @param command to be verified
         * return bool. True if there is no missing data. False in that case
         */
        bool verifyMissingData(const auv_control::ExpectedInputs &expected, const base::LinearAngular6DCommand &command);

        /** Check for connected ports
         *
         * @param in_ports: vector of input ports to be verified
         * @return vector of address of connected ports
         */
        std::vector<Base::InputPortInfo*> checkConnectedPorts(std::vector<Base::InputPortInfo> &in_ports) const;

        /** Gather input from connected input ports
         *
         * @param merging_command, where the commands will be safe
         * @param cmd_status, Define the status of merging_command, NEW_COMMAND, OLD_COMMAND, NO_COMMAND or INVALID_COMMAND
         * @param connected_ports to be analyzed.
         * @return States of wich it should go
         */
        States gatherInputCommand(LinearAngular6DCommandStatus &merging_command, std::vector<Base::InputPortInfo*> &connected_ports);

        /** Creates a new input port called cmd_name of the type
         * LinearAngular6DCommand. Once defined, this input port will be merged
         * into the merged_command command before the controller calculates the
         * corresponding output
         */
        virtual bool addCommandInput(::std::string const & name, double timeout);

        /** Send a "do not move" command to the next level
         *
         * This is called if the keep_position_on_exception property is set and
         * the component goes into an exception state
         */
        virtual void keepPosition();

        /** Computes the output based on the value received in merged_command (command and status)
         *
         * As calcOutput is defined in the derived class, the class Base can not handle
         * the States of derived class. In that case, any state transition in DerivedClass::calcOutput
         * should return a false. Like that there would not a interference with Base's states.
         *
         * @param merging_command, The command itself and its status
         * @return bool. TRUE if there is NO state transition in derived class,
         *               FALSE otherwise.
         */
        virtual bool calcOutput(const LinearAngular6DCommandStatus &merging_command);

        /** Computes the output based on the value stored in merged_command. It
         * is called after merged_command has been updated
         */ 
        virtual bool calcOutput() = 0;
        
    public:
        /** TaskContext constructor for Base
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         */
        Base(std::string const& name = "auv_control::Base");

        /** TaskContext constructor for Base 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         */
        Base(std::string const& name, RTT::ExecutionEngine* engine);

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

        /** Do one loop of read inputs and computing output
         *
         * It will be called in updateHook() and errorHook(),
         * and it can transit in both states, according its return.
         *
         * @return States which it should go.
         */
        States OneLoop();
    private:
        States merge(auv_control::ExpectedInputs const& expected, base::LinearAngular6DCommand const& current, base::LinearAngular6DCommand &merged);


    };
}

#endif
