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
    return true;
}



bool Base::startHook()
{
    if (! RTT::TaskContext::startHook())
        return false;
    
    this->setDefaultTimeout();
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
    deregisterInput("cascade");
    deregisterInput("in");
}

void Base::registerInput(std::string const& name, int timeout, InputPortType* input_port)
{
    InputPortInfo info;
    info.name = name;
    info.timeout = timeout;
    info.input_port = input_port;
    input_ports.push_back(info);
}

InputPortType* Base::deregisterInput(std::string const& name)
{
    for (vector<InputPortInfo>::iterator it = input_ports.begin();
            it != input_ports.end(); ++it)
    {
        if (it->name == name)
        {
            RTT::InputPortBase* port = it->input_port;
            input_ports.erase(it);
            return port;
        }
    }
    return 0;
}

void Base::cleanupHook()
{
    RTT::TaskContext::cleanupHook();
}

void Base::setupDefaultInputs()
{
    if (_cmd_in.connected())
        registerInput("in", _timeout_in.get(), &_cmd_in);
    if (_cmd_cascade.connected())
        registerInput("cascade", _timeout_cascade.get(), &_cmd_cascade);
}

bool Base::gatherInputCommand(){
    // The command that is being merged. It is written to this->merged_command
    // only if everything has been validated
    base::LinearAngular6DCommand merging_command;
    // The latest input received. This is used to timestamp the merged command
    base::Time max_stamp;
    for(int i = 0; i < input_ports.size(); i++){
        base::LinearAngular6DCommand current_port;
        RTT::FlowStatus status = input_ports.at(i).input_port->read(current_port);

        if(status == RTT::NoData){
            state(WAIT_FOR_INPUT);
            return false;
        } else if(status == RTT::NewData){
            input_ports.at(i).last_time = pose_sample.time;
        } else if(input_ports.at(i).timeout > 0 && (pose_sample.time - input_ports.at(i).last_time).toSeconds() > input_ports.at(i).timeout){
            exception(TIMEOUT);
            return false;
        }

        if(current_port.stamp > max_stamp){
            max_stamp = current_port.stamp;
        }

        if(!(merge(_expected_inputs.get().linear, current_port.linear, merging_command.linear) &&
                    merge(_expected_inputs.get().angular, current_port.angular, merging_command.angular))){
            return false;
        }   
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
    merging_command.stamp = max_stamp;
    merged_command = merging_command;
    return true;         
}

bool Base::addCommandInput(std::string const & name, double timeout){
    if (provides()->hasService("cmd_" + name))
        return false;

    InputPortType* input_port = new InputPortType("cmd_" + name);
    provides()->addPort(*info.input_port);
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

