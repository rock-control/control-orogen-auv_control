/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef AUV_CONTROL_CONTROLLER_BASE_TASK_HPP
#define AUV_CONTROL_CONTROLLER_BASE_TASK_HPP

#include "auv_control/Controller_BaseBase.hpp"
#include <string.h>
#include <base/float.h>

namespace auv_control {

    class Controller_Base : public Controller_BaseBase
    {
	friend class Controller_BaseBase;
    protected:
        typedef RTT::InputPort<base::LinearAngular6DCommand> InputPortType;
        struct InputPortInfo{
            std::string name;
            InputPortType *input_port;
            InputPortInfo()
                :input_port(0){}
        };    
    
        std::vector<InputPortInfo> input_ports;
        base::LinearAngular6DCommand merged_command;        

        bool gatherInputCommand();
        
        void genDefaultInput();

        virtual void addCommandInput(::std::string const & name);
        
        
    public:
        Controller_Base(std::string const& name = "auv_control::Controller_Base", TaskCore::TaskState initial_state = Stopped);

        Controller_Base(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state = Stopped);

	~Controller_Base();

    private:
        bool merge(bool *expected, bool *is_set, base::Vector3d *current, base::Vector3d *merged);
        
 
    };
}

#endif

