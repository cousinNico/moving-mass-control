#include "motor_mapping.h"


/**
 * Convert the total pulses of the motor (which trackes angular position of the shaft) to radians then to linear position of the mass
 */
Vector3d ConvertMotorPositionToMassPosition(int32_t x, int32_t y, int32_t z)
{
    // radians to meters
    double x_pos =   x * 0.000003048;  
    double y_pos =   y * 0.000003048;  
    double z_pos =   z * 0.000003048;  // z needs shifted
    
    Vector3d output; output << x_pos, y_pos, z_pos;
    return output;
}

/**
 * Convert the linear position of the mass (meters) back to total pulses of the motor (pulses)
 */
std::vector<int32_t> ConvertMassPositionToMotorPosition(double x_pos, double y_pos, double z_pos)
{
    // Convert the mass position relative to the center of rotation back to the motor's position in radians
    int32_t x = (x_pos) / 0.000003048; //
    int32_t y = (y_pos) / 0.000003048; //
    int32_t z = (z_pos) / 0.000003048; //

    return std::vector<int32_t>{x,y,z};
}

/**
 * Convert pulses per second velocity to meters per second linear mass velocity 
 */
Vector3d ConvertMotorSpeedToMassVelocity(int32_t xdot, int32_t ydot, int32_t zdot )
{
    // rotational, maps units of the stepper motor to radians per second
    double x_linear = double(xdot) * 0.000003048;
    double y_linear = double(ydot) * 0.000003048;
    double z_linear = double(zdot) * 0.000003048;
    
    Vector3d output; 
    output << x_linear, y_linear, z_linear;
    return output;
}