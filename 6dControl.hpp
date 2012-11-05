#ifndef D_CONTROL_HPP
#define D_CONTROL_HPP

namespace control{
    enum controlMode{
        NOT_SET = 0,
        CONTROL = 1,
        LAST_TARGET = 2,
        HOLD_SETTING = 3, 
    };
    
    struct Command6D{
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
    
    struct Command3D{
        double x;
        double y;
        double z;
        
        controlMode x_mode;
        controlMode y_mode;
        controlMode z_mode;
    };
    
    struct Controler6D{
        double i;
    };

}

#endif
