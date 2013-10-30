/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Base.hpp"

using namespace auv_control;

Base::Base(std::string const& name, TaskCore::TaskState initial_state)
    : BaseBase(name, initial_state)
{
    this->genDefaultInput();
}

Base::Base(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : BaseBase(name, engine, initial_state)
{
    this->genDefaultInput();
}

Base::~Base()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Base.hpp for more detailed
// documentation about them.




bool Base::configureHook()
{
    if (! RTT::TaskContext::configureHook())
        return false;

    registerInput("in", _timeout_in.get(), &_cmd_in);
    registerInput("cascade", _timeout_cascade.get(), &_cmd_cascade);
    return true;
}



bool Base::startHook()
{
    if (! RTT::TaskContext::startHook())
        return false;
    
    newestCommandTime = base::Time();
    return true;
}

void Base::updateHook()
{
    RTT::TaskContext::updateHook();
    
    if(!this->gatherInputCommand()){
        if (_keep_position_on_exception.get()){    
            this->keepPosition();
        }
        return;
    }

    if(!this->calcOutput()){
        if (_keep_position_on_exception.get()){    
            this->keepPosition();
        }
        return;
    }
}



void Base::errorHook()
{
    RTT::TaskContext::errorHook();
   
    if (_keep_position_on_exception.get()){    
        this->keepPosition();
    }
}

void Base::stopHook()
{
    RTT::TaskContext::stopHook();
}

void Base::cleanupHook()
{
    RTT::TaskContext::cleanupHook();
    deregisterInput("cascade");
    deregisterInput("in");
    while (!input_ports.empty())
    {
        InputPortType* port = input_ports.back().input_port;
        provides()->removePort(port->getName());
        delete port;
        input_ports.pop_back();
    }
}

void Base::registerInput(std::string const& name, int timeout, InputPortType* input_port)
{
    InputPortInfo info;
    info.name = name;
    info.timeout = timeout;
    info.input_port = input_port;
    input_ports.push_back(info);
}

Base::InputPortType* Base::deregisterInput(std::string const& name)
{
    for (std::vector<InputPortInfo>::iterator it = input_ports.begin();
            it != input_ports.end(); ++it)
    {
        if (it->name == name)
        {
            InputPortType* port = it->input_port;
            input_ports.erase(it);
            return port;
        }
    }
    return 0;
}

bool Base::gatherInputCommand(){
    // The command that is being merged. It is written to this->merged_command
    // only if everything has been validated
    base::LinearAngular6DCommand merging_command;
    for(unsigned int i = 0; i < input_ports.size(); i++){
        base::LinearAngular6DCommand current_port;
        InputPortInfo& port_info = input_ports.at(i);
        InputPortType* port = port_info.input_port;
        if (!port->connected())
            continue;

        RTT::FlowStatus status = port->read(current_port);

        if(status == RTT::NoData){
            state(WAIT_FOR_INPUT);
            return false;
        } else if(status == RTT::NewData){
            port_info.last_time = current_port.time;
            if (newestCommandTime < current_port.time)
                newestCommandTime = current_port.time;
        }

        if(!(merge(_expected_inputs.get().linear, current_port.linear, merging_command.linear) &&
                    merge(_expected_inputs.get().angular, current_port.angular, merging_command.angular))){
            return false;
        }   
    }

    // Verify that no port is timing out
    if (!newestCommandTime.isNull())
    {
        if (!verifyTimeout())
            return false;
    }

    for(int j = 0; j < 3; j++)
    {
        if(base::isUnset(merging_command.linear(j)) && (_expected_inputs.get().linear[j])){
            exception(INPUT_MISSING);
            return false;
        }
        if(base::isUnset(merging_command.angular(j)) && (_expected_inputs.get().angular[j])){
            exception(INPUT_MISSING);
            return false;
        }
        
    }
    merging_command.time = newestCommandTime;
    merged_command = merging_command;
    return true;         
}

bool Base::verifyTimeout()
{
    for (unsigned int i = 0; i < input_ports.size(); ++i)
    {
        double timeout = input_ports[i].timeout;
        base::Time port_time = input_ports[i].last_time;
        if (timeout != 0 && (newestCommandTime - port_time).toSeconds() > timeout)
        {
            exception(TIMEOUT);
            return false;
        }
    }
    return true;
}

bool Base::addCommandInput(std::string const & name, double timeout){
    if (provides()->hasService("cmd_" + name))
        return false;

    InputPortType* input_port = new InputPortType("cmd_" + name);
    provides()->addPort(*input_port);
    registerInput(name, timeout, input_port);
    return true;
}

bool Base::merge(bool const expected[], base::Vector3d const& current, base::Vector3d& merged){
    for(int i = 0; i < 3; i++){
        if(base::isUnset(current(i))){
            continue;
        }else if(!expected[i]){
            exception(INPUT_UNEXPECTED);
            return false;
        }else if(!base::isUnset(merged(i))){
            //Ther is a value in the merged value and the value is set on this port.
            //This is an exception!
            exception(INPUT_COLLIDING);
            return false; 
        }else{
            //No value of this type in the merged value and the value is set on this
            //port. So write the Value from this Port in the merged value.
            merged(i) = current(i);
        }
    }
    return true;
}


void Base::keepPosition(){
    
}

