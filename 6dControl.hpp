#ifndef D_CONTROL_HPP
#define D_CONTROL_HPP

#include <motor_controller/PID.hpp> 
#include <math.h>
#include <base/time.h>
#include <base/eigen.h>

namespace auv_control{
    struct ExpectedInputs{
        bool linear[3];
        bool angular[3];
    };
}    

namespace base{
    struct LinearAngular6DCommand{
        base::Time stamp;
        base::Vector3d linear;
        base::Vector3d angular;
    };

    struct LinearAngular6DPIDSettings{
        motor_controller::PIDSettings linear[3];
        motor_controller::PIDSettings angular[3];

        bool operator==(const LinearAngular6DPIDSettings& rhs) const{
            bool linear;
            bool angular;
            for(int i = 0; i < 3; i++){
                linear = this->linear[i].Ts    == rhs.linear[i].Ts   &&
                         this->linear[i].K     == rhs.linear[i].K    &&
                         this->linear[i].Ti    == rhs.linear[i].Ti   &&
                         this->linear[i].Td    == rhs.linear[i].Td   &&
                         this->linear[i].N     == rhs.linear[i].N    &&
                         this->linear[i].B     == rhs.linear[i].B    &&
                         this->linear[i].Tt    == rhs.linear[i].Tt   &&
                         this->linear[i].YMin  == rhs.linear[i].YMin &&
                         this->linear[i].YMax  == rhs.linear[i].YMax;
                
                angular = this->angular[i].Ts    == rhs.angular[i].Ts   &&
                          this->angular[i].K     == rhs.angular[i].K    &&
                          this->angular[i].Ti    == rhs.angular[i].Ti   &&
                          this->angular[i].Td    == rhs.angular[i].Td   &&
                          this->angular[i].N     == rhs.angular[i].N    &&
                          this->angular[i].B     == rhs.angular[i].B    &&
                          this->angular[i].Tt    == rhs.angular[i].Tt   &&
                          this->angular[i].YMin  == rhs.angular[i].YMin &&
                          this->angular[i].YMax  == rhs.angular[i].YMax;

                if(!linear || !angular){
                    return false;
                }
            }
            return true;
        }
        
        bool operator!=(const LinearAngular6DPIDSettings& rhs) const{
            bool linear;
            bool angular;
            for(int i = 0; i < 3; i++){
                linear = this->linear[i].Ts    != rhs.linear[i].Ts   ||
                         this->linear[i].K     != rhs.linear[i].K    ||
                         this->linear[i].Ti    != rhs.linear[i].Ti   ||
                         this->linear[i].Td    != rhs.linear[i].Td   ||
                         this->linear[i].N     != rhs.linear[i].N    ||
                         this->linear[i].B     != rhs.linear[i].B    ||
                         this->linear[i].Tt    != rhs.linear[i].Tt   ||
                         this->linear[i].YMin  != rhs.linear[i].YMin ||
                         this->linear[i].YMax  != rhs.linear[i].YMax;
                
                angular = this->angular[i].Ts    != rhs.angular[i].Ts   ||
                          this->angular[i].K     != rhs.angular[i].K    ||
                          this->angular[i].Ti    != rhs.angular[i].Ti   ||
                          this->angular[i].Td    != rhs.angular[i].Td   ||
                          this->angular[i].N     != rhs.angular[i].N    ||
                          this->angular[i].B     != rhs.angular[i].B    ||
                          this->angular[i].Tt    != rhs.angular[i].Tt   ||
                          this->angular[i].YMin  != rhs.angular[i].YMin ||
                          this->angular[i].YMax  != rhs.angular[i].YMax;

                if(linear || angular){
                    return true;
                }
            }
            return false;
        }
    };
}


namespace control{
    enum controlMode{
        NOT_SET = 0,
        CONTROL = 1,
        LAST_TARGET = 2,
        HOLD_SETTING = 3, 
    };
    
    struct WorldCommand6D{
        double x;
        double y;
        double z;
        double yaw;
        double pitch;
        double roll;
        
        controlMode x_mode;
        controlMode y_mode;
        controlMode z_mode;
        controlMode yaw_mode;
        controlMode pitch_mode;
        controlMode roll_mode;
    };
    
    struct AlignedCommand6D{
        double x;
        double y;
        double z;
        double yaw;
        double pitch;
        double roll;
        
        controlMode x_mode;
        controlMode y_mode;
        controlMode z_mode;
        controlMode yaw_mode;
        controlMode pitch_mode;
        controlMode roll_mode;
    };
    
    struct AlignedVelocityCommand6D{
        double x;
        double y;
        double z;
        double yaw;
        double pitch;
        double roll;
        
        controlMode x_mode;
        controlMode y_mode;
        controlMode z_mode;
        controlMode yaw_mode;
        controlMode pitch_mode;
        controlMode roll_mode;
        
        bool AllSet(){
            return ((x_mode != NOT_SET &&
                     y_mode != NOT_SET &&
                     z_mode != NOT_SET &&
                     yaw_mode != NOT_SET &&
                     pitch_mode != NOT_SET &&
                     roll_mode != NOT_SET) && 
                    (!isnan(x) &&
                     !isnan(y) &&
                     !isnan(z) &&
                     !isnan(yaw) &&
                     !isnan(pitch) &&
                     !isnan(roll)));
        }
    };
    

    
    
    
    struct AlignedCommand3D{
        double x;
        double y;
        double z;
        
        controlMode x_mode;
        controlMode y_mode;
        controlMode z_mode;
    };
    
    struct WorldController6D{
        motor_controller::PIDSettings x_settings;
        motor_controller::PIDSettings y_settings;
        motor_controller::PIDSettings z_settings;
        motor_controller::PIDSettings yaw_settings;
        motor_controller::PIDSettings pitch_settings;
        motor_controller::PIDSettings roll_settings;
    };
    
    struct AlignedController6D{
        motor_controller::PIDSettings x_settings;
        motor_controller::PIDSettings y_settings;
        motor_controller::PIDSettings z_settings;
        motor_controller::PIDSettings yaw_settings;
        motor_controller::PIDSettings pitch_settings;
        motor_controller::PIDSettings roll_settings;
    };
    
    struct AlignedVelocityController6D{
        motor_controller::PIDSettings x_settings;
        motor_controller::PIDSettings y_settings;
        motor_controller::PIDSettings z_settings;
        motor_controller::PIDSettings yaw_settings;
        motor_controller::PIDSettings pitch_settings;
        motor_controller::PIDSettings roll_settings;
    };

}

#endif
