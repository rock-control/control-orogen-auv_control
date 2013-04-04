/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Controller_Base.hpp"

using namespace auv_control;

Controller_Base::Controller_Base(std::string const& name, TaskCore::TaskState initial_state)
    : Controller_BaseBase(name, initial_state)
{
    genDefaultInput();
}

Controller_Base::Controller_Base(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : Controller_BaseBase(name, engine, initial_state)
{
    genDefaultInput();
}

Controller_Base::~Controller_Base()
{
}

void Controller_Base::genDefaultInput()
{
    InputPortInfo info;
    info.name = "cascade";
    info.input_port = &_cascade;

    input_ports.push_back(info);
    
    info.name = "in";
    info.input_port = &_cmd_in;
    
    input_ports.push_back(info);
}

bool Controller_Base::gatherInputCommand(){
    bool linear_is_set[] = {false, false, false};    
    bool angular_is_set[] = {false, false, false};    
    base::LinearAngular6DCommand current_port;    
    base::Time max_stamp;
    //std::cout << "Bin in gatherInputCommand()" << std::endl;

    for(int i = 0; i < input_ports.size(); i++){
        if(input_ports.at(i).input_port->read(current_port) != RTT::NoData){
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
                std::cout << "INPUT_MISSING (Linear " << j << ")" << std::endl;
                state(INPUT_MISSING);
                return false;
            }
            if(!angular_is_set[j] && ! _expected_inputs.get().angular[j]){
                merged_command.angular(j) = base::unset<double>(); 
            } else if(angular_is_set[j] != _expected_inputs.get().angular[j]){
                std::cout << "INPUT_MISSING (Angular " << j << ")" << std::endl;
                state(INPUT_MISSING);
                return false;
            }
        
    }
    merged_command.stamp = max_stamp;
    return true;           
}

void Controller_Base::addCommandInput(std::string const & name){
    if(provides()->hasService("cmd_" + name)){
        //Fehler werfen! Port bereits vorhanden
    }
    InputPortInfo info;
    info.name = name;
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
        }else {
            std::cout << "I: " << i << std::endl;
        }

    }
    return true;
}
