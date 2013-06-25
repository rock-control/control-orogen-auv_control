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
    
    if(!this->getPoseSample())
        return;

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

    _cmd_out.write(output_command);
    return;
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
    

    

    
}

void Base::genDefaultInput()
{
    InputPortInfo info;
    info.name = "cascade";
    info.timeout = _timeout_cascade.get();
    info.input_port = &_cascade;

    input_ports.push_back(info);
    
    info.name = "in";
    info.timeout = _timeout_cmd_in.get();
    info.input_port = &_cmd_in;
    
    input_ports.push_back(info);
}

void Base::setDefaultTimeout()
{
    input_ports.at(0).timeout = _timeout_cascade.get();
    input_ports.at(1).timeout = _timeout_cmd_in.get();
}

bool Base::getPoseSample(){      
    if(_pose_sample.read(pose_sample) == RTT::NoData){
        state(POSE_SAMPLE_MISSING);
        return false;
    }
    return true;
}

bool Base::gatherInputCommand(){
    base::LinearAngular6DCommand current_port;
    base::LinearAngular6DCommand merging_command;
    base::Time max_stamp;
    RTT::FlowStatus status;
    for(int i = 0; i < input_ports.size(); i++){
        status = input_ports.at(i).input_port->read(current_port);
        if(input_ports.at(i).input_port->connected()){
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

            if(!(merge(&_expected_inputs.get().linear[0], &current_port.linear, &merging_command.linear) &&
                    merge(&_expected_inputs.get().angular[0], &current_port.angular, &merging_command.angular))){
                return false;
            }   
        }
    }
    for(int j = 0; j < 3; j++){
       
        //If the Value is on evry port unset set the value in the merging_command unset too. 
        if(base::isUnset(merging_command.linear(j)) != _expected_inputs.get().linear[j]){
            state(INPUT_MISSING);
            return false;
        }
        if(base::isUnset(merging_command.angular(j)) != _expected_inputs.get().angular[j]){
            state(INPUT_MISSING);
            return false;
        }
        
    }
    merging_command.stamp = max_stamp;
    merged_command = merging_command;
    return true;           
}

void Base::addCommandInput(std::string const & name, double timeout){
    if(provides()->hasService("cmd_" + name)){
        //Fehler werfen! Port bereits vorhanden
    }
    InputPortInfo info;
    info.name = name;
    info.timeout = timeout;
    info.input_port = new InputPortType("cmd_" + name);
    provides()->addPort(*info.input_port);

    input_ports.push_back(info);

}

bool Base::merge(bool *expected, base::Vector3d *current, base::Vector3d *merged){
    for(int i = 0; i < 3; i++){
        if(expected[i] && base::isUnset((*merged)(i)) && !base::isUnset((*current)(i))){
            //No value of this type in the merged value and the value is set on this
            //port. So write the Value from this Port in the merged value.
            (*merged)(i) = (*current)(i);
        }else if(expected[i] && !base::isUnset((*merged)(i)) && !base::isUnset((*current)(i))){
            //Ther is a value in the merged value and the value is set on this port.
            //This is an exception!
            exception(INPUT_COLLIDING);
            return false; 
        }else if(!expected[i] && !base::isUnset((*current)(i))){
            exception(INPUT_UNEXPECTED);
            return false;
        }
    }
    return true;
}


void Base::keepPosition(){
    
}

bool Base::calcOutput(){
    return false;
}
