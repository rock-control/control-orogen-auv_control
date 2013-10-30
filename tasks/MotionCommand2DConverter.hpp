/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_MOTIONCOMMAND2DCONVERTER_TASK_HPP
#define AUV_CONTROL_MOTIONCOMMAND2DCONVERTER_TASK_HPP

#include "auv_control/MotionCommand2DConverterBase.hpp"

namespace auv_control {

    /*! \class MotionCommand2DConverter 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','auv_control::MotionCommand2DConverter')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class MotionCommand2DConverter : public MotionCommand2DConverterBase
    {
	friend class MotionCommand2DConverterBase;
    protected:



    public:
        /** TaskContext constructor for MotionCommand2DConverter
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        MotionCommand2DConverter(std::string const& name = "auv_control::MotionCommand2DConverter");

        /** TaskContext constructor for MotionCommand2DConverter 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        MotionCommand2DConverter(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of MotionCommand2DConverter
         */
	~MotionCommand2DConverter();

        void updateHook();
    };
}

#endif

