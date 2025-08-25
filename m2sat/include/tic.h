#pragma once

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define RAD_TO_REV (1/(2*M_PI))

#define STEPPER_MAX_ACCEL_PPS2  (100000.0f)  //80000 on 4th steps  // pulses per second squared, if we go higher, we need more current but we are already current limiting
#define STEPPER_MAX_DECELL_PPS2 (100000.0f) //60000 on 4th steps
#define TIC_CURRENT_LIMIT_MILLIAMPS (1800)
#define STEPPER_STEP_MODE_NUMERIC (8.0f) // 2 half step, 4 quater etc
#define STEPPER_STEP_MODE (3) 
/* per the stepper_step_mode documentation from polulu
0: Full step
1: 1/2 step
2: 1/4 step
3: 1/8 step
4: 1/16 step (Tic T834, Tic T825, and Tic 36v4 only)
*/

#define STEPPER_STEPS_PER_REV (200.0f * STEPPER_STEP_MODE_NUMERIC) // (1.8 degree step angle full step)
#define PPS2_UNIT_CONVERSION (100.0f) // polulu uses pulses per 10,000 seconds instead of per second (pulses per second squared)
#define PPS_UNIT_CONVERSION (10000.0f) // polulu uses pulses per 10,000 seconds instead of per second (pulses per second)


/* Stepper max rates */
#define STEPPER_MAX_RAD_PER_SEC (50.0f) 
#define STEPPER_MAX_PULSES_PER_SEC (STEPPER_MAX_RAD_PER_SEC * RAD_TO_REV * STEPPER_STEPS_PER_REV * STEPPER_STEP_MODE_NUMERIC)
#define STEPPER_START_SPEED_PPS (0.0f) // speed the motor tries to start at, if too high, it stalls


#define TIC_I2C_ADDRESS_DEVICE "/dev/i2c-1"

/* Physical parameters */
#define X_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES (318.0f) // measured in full pulses
#define Y_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES (325.0f)
#define Z_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES (-225.0f)

/* Read addresses */
#define ADDR_GET_VARIABLE (0xA1)
#define ADDR_CURRENT_POSITION (0x22)
#define ADDR_CURRENT_VELOCITY (0x26)
#define ADDR_GET_CURRENT_TARGET_POSITION (0x0A)
#define ADDR_MISC_FLAGS_1 (0x01)
/* Command addresses */
#define ADDR_DEENERGIZE (0x86)
#define ADDR_ENERGIZE (0x85)
#define ADDR_GO_HOME (0x97)

/* Configuration addresses*/
#define ADDR_SET_CURRENT_LIMIT (0x91)
#define ADDR_SET_TARGET_POSITION (0xE0)
#define ADDR_SET_MAX_SPEED (0xE6)
#define ADDR_SET_STARTING_SPEED (0xE5)
#define ADDR_SET_MAX_ACCEL (0xEA)
#define ADDR_SET_MAX_DECELL (0xE9)
#define ADDR_SET_STEP_MODE (0x94)
#define ADDR_SET_REVERSE_MODE (0x1B)

// Use #pragma pack to ensure the compiler doesn't add extra padding:
#pragma pack(push, 1)
union misc_flags_1_t
{
    uint8_t all; // The raw 8-bit value
    struct
    {
        // Each field is 1 bit wide.
        // Bits are declared in the order from least significant to most.
        uint8_t energized           : 1; // Bit 0
        uint8_t positionUncertain   : 1; // Bit 1
        uint8_t forwardLimitActive  : 1; // Bit 2
        uint8_t reverseLimitActive  : 1; // Bit 3
        uint8_t homingActive        : 1; // Bit 4
        uint8_t reserved            : 3; // Bits 5-7
    } bits;
};
#pragma pack(pop)


extern "C" 
{
    int open_i2c_device(const char * device);
    int tic_exit_safe_start(int fd, uint8_t address);
    int tic_deenergize(int fd, uint8_t address);
    int tic_go_home(int fd, uint8_t address, uint8_t direction);

    int tic_set_target_position(int fd, uint8_t address, int32_t target);
    
    int tic_set_max_speed(int fd, uint8_t address, int32_t max_speed);
    int tic_set_starting_speed(int fd, uint8_t address, int32_t starting_speed);
    int tic_set_max_accel(int fd, uint8_t address, int32_t max_accel);
    int tic_set_max_deccel(int fd, uint8_t address, int32_t max_deccel);
    int tic_set_current_limit(int fd, uint8_t address, uint8_t current_limit);
    int tic_set_step_mode(int fd, uint8_t address, uint8_t step_mode);
    int tic_set_reverse_mode(int fd, uint8_t address);


    int tic_get_variable(int fd, uint8_t address, uint8_t offset, uint8_t * buffer, uint8_t length);
    int tic_get_current_position(int fd, uint8_t address, int32_t * output);
    int tic_get_current_velocity(int fd, uint8_t address, int32_t * output);
    int tic_get_target_position(int fd, uint8_t address, int32_t * result);

}

uint8_t mapCurrentLimit(double current_mA);
int SetTicSettings(int fd, uint8_t address);
int SendAllTicsHome(int fd, const uint8_t * addresses);