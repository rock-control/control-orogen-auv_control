/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Controller_Base.hpp"

using namespace auv_control;

Controller_Base::Controller_Base(std::string const& name, TaskCore::TaskState initial_state)
    : Controller_BaseBase(name, initial_state)
{
    this->genDefaultInput();
}

Controller_Base::Controller_Base(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : Controller_BaseBase(name, engine, initial_state)
{
    this->genDefaultInput();
}

Controller_Base::~Controller_Base()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Controller_Base.hpp for more detailed
// documentation about them.




bool Controller_Base::configureHook()
{
    
    if (! RTT::TaskContext::configureHook())
        return false;
    

    

    
    return true;
    
}



bool Controller_Base::startHook()
{
    
    if (! RTT::TaskContext::startHook())
        return false;
    
    this->setDefaultTimeout();
    

    
    return true;
    
}



void Controller_Base::updateHook()
{
    
    RTT::TaskContext::updateHook();
    
    if(!this->getPoseSample())
        this->doNothing();
        return;

    if(!this->gatherInputCommand()){
        this->doNothing();
        return;
    }

    if(!this->calcOutput()){
        this->doNothing();
        return;
    }

    _cmd_out.write(output_command);
    state(RUNNING);
    return;
}



void Controller_Base::errorHook()
{
    
    RTT::TaskContext::errorHook();
   
    this->doNothing();

    

    
}



void Controller_Base::stopHook()
{
    
    RTT::TaskContext::stopHook();
    

    

    
}



void Controller_Base::cleanupHook()
{
    
    RTT::TaskContext::cleanupHook();
    

    

    
}

void Controller_Base::genDefaultInput()
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

void Controller_Base::setDefaultTimeout()
{
    input_ports.at(0).timeout = _timeout_cascade.get();
    input_ports.at(1).timeout = _timeout_cmd_in.get();
}

bool Controller_Base::getPoseSample(){      
    if(_pose_sample.read(pose_sample) == RTT::NoData){
        state(POSE_SAMPLE_MISSING);
        return false;
    }
    return true;
}

bool Controller_Base::gatherInputCommand(){
    bool linear_is_set[] = {false, false, false};    
    bool angular_is_set[] = {false, false, false};    
    base::LinearAngular6DCommand current_port;    
    base::Time max_stamp;
    RTT::FlowStatus status;
    //std::cout << "Bin in gatherInputCommand()" << std::endl;
    for(int i = 0; i < input_ports.size(); i++){
        status = input_ports.at(i).input_port->read(current_port);
        if(input_ports.at(i).input_port->connected()){
            if(status == RTT::NoData){
                state(WAIT_FOR_INPUT);
                return false;
            } else if(status == RTT::NewData){
                input_ports.at(i).last_time = pose_sample.time;
                //std::cout << "New Input on port " << input_ports.at(i).name << std::endl;
            } else if(input_ports.at(i).timeout > 0 && (pose_sample.time - input_ports.at(i).last_time).toSeconds() > input_ports.at(i).timeout){
                std::cout << "[ERROR] Timeout" << std::endl;
                error(TIMEOUT);
                return false;
            }

            if(current_port.stamp > max_stamp){
                max_stamp = current_port.stamp;
             }
            //std::cout << "Input-Port" << current_port.linear << std::endl;

            if(!(merge(&_expected_inputs.get().linear[0], &linear_is_set[0], &current_port.linear, &merged_command.linear) &&
                    merge(&_expected_inputs.get().angular[0] ,&angular_is_set[0], &current_port.angular, &merged_command.angular))){
                return false;
            }   
        }
    }
    //std::cout << linear_is_set[0] << std::endl;    
    //std::cout <<  _expected_inputs.get().linear[0]<< std::endl;    
    //std::cout <<  merged_command.linear<< std::endl;    
    for(int j = 0; j < 3; j++){
       
        //If the Value is on evry port unset set the value in the merged_command unset too. 
        //std::cout << linear_is_set[j] << _expected_inputs.get().linear[j] << std::endl;
        if(!linear_is_set[j] && ! _expected_inputs.get().linear[j]){
            merged_command.linear(j) = base::unset<double>(); 
        } else if(linear_is_set[j] != _expected_inputs.get().linear[j]){
            //std::cout << "INPUT_MISSING (Linear " << j << ")" << std::endl;
            state(INPUT_MISSING);
            return false;
        }
        if(!angular_is_set[j] && ! _expected_inputs.get().angular[j]){
            merged_command.angular(j) = base::unset<double>(); 
        } else if(angular_is_set[j] != _expected_inputs.get().angular[j]){
            //std::cout << "INPUT_MISSING (Angular " << j << ")" << std::endl;
            state(INPUT_MISSING);
            return false;
        }
        
    }
    merged_command.stamp = max_stamp;
    return true;           
}

void Controller_Base::addCommandInput(std::string const & name, double timeout){
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

bool Controller_Base::merge(bool *expected, bool *is_set, base::Vector3d *current,
        base::Vector3d *merged){
    //std::cout << *current << std::endl;
    for(int i = 0; i < 3; i++){
        //merge linear
        if(expected[i] && !is_set[i] && !base::isUnset((*current)(i))){
            //No value of this type in the merged value and the value is set on this
            //port. So write the Value from this Port in the merged value.
            (*merged)(i) = (*current)(i);
            is_set[i] = true;
            //std::cout << "linear i, j :" << i << ", " << j << std::endl;
        }else if(expected[i] && is_set[i] && !base::isUnset((*current)(i))){
            //Ther is a value in the merged value and the value is set on this port.
            //This is an error!
            std::cout << "Their is a collison on the input ports. (" << i <<")!" << std::endl;
            error(INPUT_COLLIDING);
            return false; 
        }else if(!expected[i] && !base::isUnset((*current)(i))){
            error(INPUT_UNEXPECTED);
            return false;
        }
    }
    return true;
}


void Controller_Base::doNothing(){
    
}

bool Controller_Base::calcOutput(){
    return false;
}
