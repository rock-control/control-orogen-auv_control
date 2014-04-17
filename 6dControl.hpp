#ifndef D_CONTROL_HPP
#define D_CONTROL_HPP

#include <motor_controller/PID.hpp> 
#include <math.h>
#include <base/time.h>
#include <base/eigen.h>
#include <base/float.h>

namespace auv_control{
    struct ExpectedInputs{
        bool linear[3];
        bool angular[3];
    };
}    

namespace base{
    /** Common command structure for all controller types, in all control frames */
    struct LinearAngular6DCommand{
        /** The command timestamp */
        base::Time time;
        /** The linear part of the command, as (x,y,z) */
        base::Vector3d linear;
        /** The angular part of the command, as (r,p,y) */
        base::Vector3d angular;

        LinearAngular6DCommand(){
            for(int i = 0; i < 3; i++){
                linear(i) = base::unset<double>();
                angular(i) = base::unset<double>();
            }
        }

        double& x() { return linear(0); }
        double& y() { return linear(1); }
        double& z() { return linear(2); }
        double& roll() { return angular(0); }
        double& pitch() { return angular(1); }
        double& yaw() { return angular(2); }

        double x() const { return linear(0); }
        double y() const { return linear(1); }
        double z() const { return linear(2); }
        double roll() const { return angular(0); }
        double pitch() const { return angular(1); }
        double yaw() const { return angular(2); }
    };

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
}
#endif
