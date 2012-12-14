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

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Controller_Base.hpp for more detailed
// documentation about them.

// bool Controller_Base::configureHook()
// {
//     if (! Controller_BaseBase::configureHook())
//         return false;
//     return true;
// }
// bool Controller_Base::startHook()
// {
//     if (! Controller_BaseBase::startHook())
//         return false;
//     return true;
// }
// void Controller_Base::updateHook()
// {
//     Controller_BaseBase::updateHook();
// }
// void Controller_Base::errorHook()
// {
//     Controller_BaseBase::errorHook();
// }
// void Controller_Base::stopHook()
// {
//     Controller_BaseBase::stopHook();
// }
// void Controller_Base::cleanupHook()
// {
//     Controller_BaseBase::cleanupHook();
// }


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

            for(int j = 0; j < 3; j++){
                //merge linear
                if(!linear_is_set[j] && !base::isUnset(current_port.linear(j))){
                    //No value of this type in the merged value and the value is set on this
                    //port. So write the Value from this Port in the merged value.
                    merged_command.linear(j) = current_port.linear(j);
                    linear_is_set[j] = true;
                    //std::cout << "linear i, j :" << i << ", " << j << std::endl;
                }else if(linear_is_set[j] && !base::isUnset(current_port.linear(j))){
                    //Ther is a value in the merged value and the value is set on this port.
                    //This is an error!
                    std::cout << "Their is a collison on the input ports. (linear(" << j <<"))!" << std::endl;
                    error(INPUT_COLLIDING);
                    return false; 
                }
                //merge angular
                if(!angular_is_set[j] && !base::isUnset(current_port.angular(j))){
                    //No value of this type in the merged value and the value is set on this
                    //port. So write the Value from this Port in the merged value.
                    merged_command.angular(j) = current_port.angular(j);
                    angular_is_set[j] = true;
                }else if(angular_is_set[j] && !base::isUnset(current_port.angular(j))){
                    //Ther is a value in the merged value and the value is set on this port.
                    //This is an error!
                    std::cout << "Their is a collison on the input ports. (angular(" << j <<"))!" << std::endl;
                    error(INPUT_COLLIDING);
                    return false;
                }
            }
        }
    }
    for(int j = 0; j < 3; j++){
       
            //If the Value is on evry port unset set the value in the merged_command unset too. 
            //std::cout << linear_is_set[j] << _expected_inputs.get().linear[j] << std::endl;
            if(!linear_is_set[j] && ! _expected_inputs.get().linear[j]){
                merged_command.linear(j) = base::unset<double>(); 
            } else if(linear_is_set[j] != _expected_inputs.get().linear[j]){
                error(INPUT_MISSING);
                return false;
            }
            if(!angular_is_set[j] && ! _expected_inputs.get().angular[j]){
                merged_command.angular(j) = base::unset<double>(); 
            } else if(angular_is_set[j] != _expected_inputs.get().angular[j]){
                error(INPUT_MISSING);
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
