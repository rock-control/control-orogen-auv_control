#ifndef D_CONTROL_HPP
#define D_CONTROL_HPP

#include <motor_controller/PID.hpp> 
#include <math.h>
#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base/Float.hpp>
#include <base/commands/LinearAngular6DCommand.hpp>

namespace auv_control{
    struct ExpectedInputs{
        bool linear[3];
        bool angular[3];
    };

    struct PIDState : public motor_controller::PIDState
    {
        bool active;

        PIDState()
            : motor_controller::PIDState(), active(false) {}
        PIDState(motor_controller::PIDState const& state, bool active)
            : motor_controller::PIDState(state), active(active) {}
    };

    struct LinearAngular6DPIDState{
        PIDState linear[3];
        PIDState angular[3];
    };

    enum CommandStatus
    {
        NEW_COMMAND,
        OLD_COMMAND,
        NO_COMMAND,
        INVALID_COMMAND,
    };

    struct LinearAngular6DCommandStatus
    {
        base::LinearAngular6DCommand command;
        CommandStatus status;

        LinearAngular6DCommandStatus(){
            command = base::LinearAngular6DCommand();
            status = INVALID_COMMAND;
        }
    };
}

namespace base{
    struct LinearAngular6DWaypoint{
        LinearAngular6DCommand cmd;
        double opt_orientation;
        double opt_orientation_distance;
        double linear_tolerance;
        double angular_tolerance;
        double hold_time;

        LinearAngular6DWaypoint(){
            opt_orientation_distance = base::infinity<double>();
            linear_tolerance = 0.2;
            angular_tolerance = 0.2;
            hold_time = 0;
        }
    };

    struct LinearAngular6DWaypointInfo{
        LinearAngular6DCommand current_delta;
        LinearAngular6DWaypoint current_waypoint;
        int queue_size;
    };
    
        
    struct LinearAngular6DPIDSettings{
        motor_controller::PIDSettings linear[3];
        motor_controller::PIDSettings angular[3];

        bool operator==(const LinearAngular6DPIDSettings& rhs) const{
            for(int i = 0; i < 3; i++){
                if(this->linear[i] != rhs.linear[i] || this->angular[i] != rhs.angular[i])
                    return false;
            }
            return true;
        }
        
        bool operator!=(const LinearAngular6DPIDSettings& rhs) const{
            return !(*this == rhs);
        }
    };
    
    struct LinearAngular6DParallelPIDSettings{
        motor_controller::ParallelPIDSettings linear[3];
        motor_controller::ParallelPIDSettings angular[3];

        bool operator==(const LinearAngular6DParallelPIDSettings& rhs) const{
            for(int i = 0; i < 3; i++){
                if(this->linear[i] != rhs.linear[i] || this->angular[i] != rhs.angular[i])
                    return false;
            }
            return true;
        }
        
        bool operator!=(const LinearAngular6DParallelPIDSettings& rhs) const{
            return !(*this == rhs);
        }
    };
}
#endif
