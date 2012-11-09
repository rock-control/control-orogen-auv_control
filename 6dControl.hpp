#ifndef D_CONTROL_HPP
#define D_CONTROL_HPP

#include <motor_controller/PID.hpp> 
#include <math.h>

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
                    (isnan(x) &&
                     isnan(y) &&
                     isnan(z) &&
                     isnan(yaw) &&
                     isnan(pitch) &&
                     isnan(roll)));
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
