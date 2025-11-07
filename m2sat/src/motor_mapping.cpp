#include "motor_mapping.h"


/**
 * Convert the total pulses of the motor (which trackes angular position of the shaft) to radians then to linear position of the mass
 */
Vector3d ConvertMotorPositionToMassPosition(int32_t x, int32_t y, int32_t z)
{
    // // rotational, maps units of the stepper motor to radians
    // double x_rad = (double(x)-X_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES*STEPPER_STEP_MODE_NUMERIC) * TAU * (1/STEPPER_STEPS_PER_REV) ; // (pos - initial) * 2pi radians / 200*step_mode  
    // double y_rad = (double(y)-Y_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES*STEPPER_STEP_MODE_NUMERIC) * TAU * (1/STEPPER_STEPS_PER_REV) ; 
    // double z_rad = (double(z)-Z_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES*STEPPER_STEP_MODE_NUMERIC) * TAU * (1/STEPPER_STEPS_PER_REV) ; 

    // radians to meters
    // double x_pos =   x_rad * X_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000);  
    // double y_pos =   y_rad * Y_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000);  
    // double z_pos =   z_rad * Z_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000);  // z needs shifted
    
    double x_pos = double(x) * 0.003048;
    double y_pos = double(y) * 0.003048;
    double z_pos = double(z) * 0.003048;

    Vector3d output; output << x_pos, y_pos, z_pos;
    return output;
}

/**
 * Convert the linear position of the mass (meters) back to total pulses of the motor (pulses)
 */
std::vector<int32_t> ConvertMassPositionToMotorPosition(double x_pos, double y_pos, double z_pos)
{
    // // Convert the mass position relative to the center of rotation back to the motor's position in radians
    // double x_rad = (x_pos) / (X_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000)); // (x_pos) / (X_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM / 1000));
    // double y_rad = (y_pos) / (Y_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000)); // (y_pos) / (Y_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM / 1000));
    // double z_rad = (z_pos) / (Z_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM/1000)); // (z_pos) / (Z_GEAR_RATIO * (PULLEY_PITCH_RADIUS_MM / 1000));

    // // Convert radians back to pulses
    // int32_t x = int32_t(x_rad * STEPPER_STEPS_PER_REV / TAU) + int32_t(X_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES*STEPPER_STEP_MODE_NUMERIC);
    // int32_t y = int32_t(y_rad * STEPPER_STEPS_PER_REV / TAU) + int32_t(Y_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES*STEPPER_STEP_MODE_NUMERIC);
    // int32_t z = int32_t(z_rad * STEPPER_STEPS_PER_REV / TAU) + int32_t(Z_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES*STEPPER_STEP_MODE_NUMERIC);

    int32_t x = int32_t(x_pos) / 0.003048;
    int32_t y = int32_t(y_pos) / 0.003048;
    int32_t z = int32_t(z_pos) / 0.003048;

    return std::vector<int32_t>{x,y,z};
}

/**
 * Convert pulses per second velocity to meters per second linear mass velocity 
 */
Vector3d ConvertMotorSpeedToMassVelocity(int32_t xdot, int32_t ydot, int32_t zdot )
{
    // // rotational, maps units of the stepper motor to radians per second
    // double x_radians_per_sec = double(xdot) * TAU * (1/STEPPER_STEPS_PER_REV) * (1/PPS_UNIT_CONVERSION); // 2pi radians per 200 counts
    // double y_radians_per_sec = double(ydot) * TAU * (1/STEPPER_STEPS_PER_REV) * (1/PPS_UNIT_CONVERSION); // 2pi radians per 200 counts
    // double z_radians_per_sec = double(zdot) * TAU * (1/STEPPER_STEPS_PER_REV) * (1/PPS_UNIT_CONVERSION); // 2pi radians per 200 counts
    
    // // linear = r * omega where r is pitch radius in meters and omega is angular velocity of the pulley
    // double x_linear = (X_GEAR_RATIO * x_radians_per_sec) * (PULLEY_PITCH_RADIUS_MM/1000.0f); // account for gear ratio on angular velocity, pitch radius maps rot to linear
    // double y_linear = (Y_GEAR_RATIO * y_radians_per_sec) * (PULLEY_PITCH_RADIUS_MM/1000.0f); // account for gear ratio on angular velocity, pitch radius maps rot to linear
    // double z_linear = (Z_GEAR_RATIO * z_radians_per_sec) * (PULLEY_PITCH_RADIUS_MM/1000.0f); // account for gear ratio on angular velocity, pitch radius maps rot to linear

    double x_linear = double(xdot) * 0.003048;
    double y_linear = double(ydot) * 0.003048;
    double z_linear = double(zdot) * 0.003048;

    Vector3d output; 
    output << x_linear, y_linear, z_linear;
    return output;
}