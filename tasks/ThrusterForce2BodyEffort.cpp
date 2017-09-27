/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ThrusterForce2BodyEffort.hpp"
#include "base-logging/Logging.hpp"

using namespace auv_control;

ThrusterForce2BodyEffort::ThrusterForce2BodyEffort(std::string const& name)
    : ThrusterForce2BodyEffortBase(name)
{
}

ThrusterForce2BodyEffort::ThrusterForce2BodyEffort(std::string const& name, RTT::ExecutionEngine* engine)
    : ThrusterForce2BodyEffortBase(name, engine)
{
}

ThrusterForce2BodyEffort::~ThrusterForce2BodyEffort()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ThrusterForce2BodyEffort.hpp for more detailed
// documentation about them.

bool ThrusterForce2BodyEffort::configureHook()
{
    if (! ThrusterForce2BodyEffortBase::configureHook())
        return false;

    thrusterMatrix = _thruster_configuration_matrix.get();

    return true;
}
bool ThrusterForce2BodyEffort::startHook()
{
    if (! ThrusterForce2BodyEffortBase::startHook())
        return false;
    return true;
}
void ThrusterForce2BodyEffort::updateHook()
{
    ThrusterForce2BodyEffortBase::updateHook();

    base::samples::Joints thrusterForces;

    while(_thruster_forces.read(thrusterForces) == RTT::NewData)
    {
        base::LinearAngular6DCommand bodyEffort;
        base::VectorXd thrusterForcesVector;
        base::Vector6d bodyEffortVector;
        unsigned int numberOfThrusters = thrusterMatrix.cols();
        thrusterForcesVector = base::VectorXd::Zero(numberOfThrusters);

        if(thrusterForces.elements.size() != numberOfThrusters)
        {
            LOG_ERROR("The input vector should have a size equal to %i, but actually it "
                    "has size equal to %i. Check configuration. ",
                    numberOfThrusters, thrusterForces.elements.size());
            exception(UNEXPECTED_THRUSTER_INPUT);
            return;
        }

        for(uint i = 0; i < numberOfThrusters; i++)
        {
            if(!thrusterForces.elements[i].hasEffort())
            {
                std::string textThruster;

                // Check whether names were specified for the thrusters
                if(thrusterForces.names.size() == numberOfThrusters)
                    textThruster = thrusterForces.names[i];
                else
                {
                    std::stringstream number;
                    number << i;
                    textThruster = number.str();
                }
                LOG_ERROR("The field effort of the thruster %s was not set.",
                        textThruster.c_str());
                exception(UNSET_THRUSTER_INPUT);
                return;
            }

            thrusterForcesVector[i] = thrusterForces.elements[i].effort;
        }

        bodyEffortVector = thrusterMatrix * thrusterForcesVector;

        bodyEffort.linear = bodyEffortVector.head(3);
        bodyEffort.angular = bodyEffortVector.tail(3);
        bodyEffort.time = thrusterForces.time;

        _body_efforts.write(bodyEffort);
    }
}
void ThrusterForce2BodyEffort::errorHook()
{
    ThrusterForce2BodyEffortBase::errorHook();
}
void ThrusterForce2BodyEffort::stopHook()
{
    ThrusterForce2BodyEffortBase::stopHook();
}
void ThrusterForce2BodyEffort::cleanupHook()
{
    ThrusterForce2BodyEffortBase::cleanupHook();
}
