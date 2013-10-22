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
        base::Time stamp;
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
    };

    struct LinearAngular6DWaypoint{
        LinearAngular6DCommand cmd;
        base::Vector3d opt_orientation;
        double opt_orientation_distance;
        double linear_tolerance;
        double angular_tolerance;

        LinearAngular6DWaypoint(){
            opt_orientation_distance = base::infinity<double>();
            linear_tolerance = 0.2;
            angular_tolerance = 0.2;
        }
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
