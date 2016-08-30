/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Base.hpp"

using namespace auv_control;

Base::Base(std::string const& name)
    : BaseBase(name)
{
}

Base::Base(std::string const& name, RTT::ExecutionEngine* engine)
    : BaseBase(name, engine)
{
}

Base::~Base()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Base.hpp for more detailed
// documentation about them.




bool Base::configureHook()
{
    if (! BaseBase::configureHook())
        return false;

    registerInput("in", _timeout_in.get(), &_cmd_in);
    registerInput("cascade", _timeout_cascade.get(), &_cmd_cascade);
    // Assuming a max number of input ports of 6 (3 for each DOF, linear and angular)
    connected_input_ports.reserve(6);
    return true;
}



bool Base::startHook()
{
    if (! BaseBase::startHook())
        return false;
    
    newestCommandTime = base::Time();
    return true;
}

void Base::updateHook()
{
    BaseBase::updateHook();

    States stat = performOneLoop();
    // Error in calcOutput, state transitions should be handle in derived class
    if(stat == RUNTIME_ERROR)
        return;
    // Check for local error in Base
    if(stat != CONTROLLING && stat != CONTROLLING_UNSAFE)
    {
        error(stat);
        return;
    }
    // Switch to specific Running state defined in Base
    if(state() != stat);
        state(stat);
}



void Base::errorHook()
{
    BaseBase::errorHook();

    States stat = performOneLoop();
    // Error in calcOutput, state transitions should be handle in derived class
    if(stat == RUNTIME_ERROR)
        return;
    // Return to Running state
    if(stat == CONTROLLING || stat == CONTROLLING_UNSAFE)
    {
        state(stat);
        recover();
        return;
    }
    // Switch beetwen error states if it's the case
    if(state() != stat)
        error(stat);

    if (_keep_position_on_exception.get())
        this->keepPosition();
}

void Base::stopHook()
{
    BaseBase::stopHook();
}

void Base::cleanupHook()
{
    BaseBase::cleanupHook();
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

Base::States Base::performOneLoop()
{
    if(!checkConnectedPorts(input_ports, connected_input_ports))
        return WAIT_FOR_CONNECTED_INPUT_PORT;

    LinearAngular6DCommandStatus merged_command;
    States state = this->gatherInputCommand(merged_command, connected_input_ports);

    if (state != CONTROLLING)
        return state;

    if (!this->calcOutput(merged_command))
        return RUNTIME_ERROR;

    if (_safe_mode.get())
        return CONTROLLING;

    return CONTROLLING_UNSAFE;
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

Base::States Base::gatherInputCommand(LinearAngular6DCommandStatus &merged_command, std::vector<Base::InputPortInfo*> &connected_ports)
{
    // The command that is being merged. It is written to this->merged_command
    // only if everything has been validated
    merged_command.status = RTT::OldData;
    for(unsigned int i = 0; i < connected_ports.size(); i++){
        base::LinearAngular6DCommand current_port;
        InputPortInfo& port_info = *connected_ports.at(i);
        InputPortType* port = port_info.input_port;

        RTT::FlowStatus status = port->readNewest(current_port);

        if(status == RTT::NoData)
            return WAIT_FOR_INPUT;
        else if(status == RTT::NewData)
        {   // Has at least one new data
            merged_command.status = RTT::NewData;
            port_info.last_sample_time = current_port.time;
            port_info.last_system_time = base::Time::now();
            if (newestCommandTime < current_port.time)
                newestCommandTime = current_port.time;
        }
        States merge_state = merge(_expected_inputs.get(), current_port, merged_command.command);
        if(merge_state != CONTROLLING)
            return merge_state;
    }

    if (!verifyTimeout(newestCommandTime))
        return TIMEOUT;
    if(_safe_mode.get() && !verifyMissingData(_expected_inputs.get(), merged_command.command))
        return INPUT_MISSING;

    merged_command.command.time = newestCommandTime;
    return CONTROLLING;
}

bool Base::verifyTimeout(const base::Time &newest_command)
{
    // In case it's not initialzed
    if(newest_command.isNull())
        return true;
    for (unsigned int i = 0; i < input_ports.size(); ++i)
    {
        if(input_ports[i].input_port->connected())
        {
            double timeout = input_ports[i].timeout;
            base::Time port_time = input_ports[i].last_sample_time;
            if (timeout != 0 && ((newest_command - port_time).toSeconds() > timeout))
                return false;
            if (timeout != 0 && ((base::Time::now() - input_ports[i].last_system_time).toSeconds() > timeout))
                return false;
        }
    }
    return true;
}

bool Base::verifyMissingData(const auv_control::ExpectedInputs &expected, const base::LinearAngular6DCommand &command)
{
    for(int j = 0; j < 3; j++)
    {
        if(base::isUnset(command.linear[j]) && expected.linear[j])
            return false;
        if(base::isUnset(command.angular[j]) && expected.angular[j])
            return false;
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

Base::States Base::merge(auv_control::ExpectedInputs const& expected, base::LinearAngular6DCommand const& current, base::LinearAngular6DCommand &merged)
{
    for(int i = 0; i < 3; i++)
    {
        if(_safe_mode.get() && !expected.linear[i] && !base::isUnset(current.linear[i]))
            return INPUT_UNEXPECTED;
        if(_safe_mode.get() && !expected.angular[i] && !base::isUnset(current.angular[i]))
            return INPUT_UNEXPECTED;

        //There is a value in the merged value and the value is set on this port.
        //This is an error!
        if(!base::isUnset(merged.linear[i]) && !base::isUnset(current.linear[i]))
            return INPUT_COLLIDING;
        if(!base::isUnset(merged.angular[i]) && !base::isUnset(current.angular[i]))
            return INPUT_COLLIDING;

        //No value of this type in the merged value and the value is set on this
        //port. So write the Value from this Port in the merged value.
        if(base::isUnset(merged.linear[i]))
            merged.linear[i] = current.linear[i];
        if(base::isUnset(merged.angular[i]))
            merged.angular[i] = current.angular[i];
    }
    return CONTROLLING;
}

bool Base::checkConnectedPorts(std::vector<Base::InputPortInfo> &input_ports, std::vector<Base::InputPortInfo*> &connected_input_ports)
{   // Clear vector before assign connected ports
    if(!connected_input_ports.empty())
        connected_input_ports.erase(connected_input_ports.begin(), connected_input_ports.end());
    for(unsigned int i = 0; i < input_ports.size(); i++)
    {
        if(input_ports.at(i).input_port->connected())
            connected_input_ports.push_back(&input_ports.at(i));
    }
    return !connected_input_ports.empty();
}

void Base::keepPosition(){

}
