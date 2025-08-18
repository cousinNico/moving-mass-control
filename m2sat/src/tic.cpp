#include "tic.h"
#include <cstdint>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
// Opens the specified I2C device.  Returns a non-negative file descriptor
// on success, or -1 on failure.
int open_i2c_device(const char * device)
{
    int fd = open(device, O_RDWR);
    if (fd == -1) {
        perror(device);
        return -1;
    }
    return fd;
}

// Sends the "Exit safe start" command.
// Returns 0 on success and -1 on failure.
int tic_exit_safe_start(int fd, uint8_t address)
{
    uint8_t command[] = { 0x83 };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to exit safe start");
        return -1;
    }
    return 0;
}

// Sets the target position, returning 0 on success and -1 on failure.
//
// For more information about what this command does, see the
// "Set Target Position" command in the "Command reference" section of the
// Tic user's guide.
int tic_set_target_position(int fd, uint8_t address, int32_t target)
{
    uint8_t command[] = {
        ADDR_SET_TARGET_POSITION,
        (uint8_t)(target >> 0 & 0xFF),
        (uint8_t)(target >> 8 & 0xFF),
        (uint8_t)(target >> 16 & 0xFF),
        (uint8_t)(target >> 24 & 0xFF),
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set target position");
        return -1;
    }
    return 0;
}

// Gets one or more variables from the Tic (without clearing them).
// Returns 0 for success, -1 for failure.
int tic_get_variable(int fd, uint8_t address, uint8_t offset,
uint8_t * buffer, uint8_t length)
{
    uint8_t command[] = { ADDR_GET_VARIABLE, offset };
    struct i2c_msg messages[] = {
        { address, 0, sizeof(command), command },
        { address, I2C_M_RD, length, buffer },
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 2) {
    perror("failed to get variables");
    return -1;
    }

    return 0;
}

int tic_get_target_position(int fd, uint8_t address, int32_t * result)
{
    tic_get_variable(fd, address, ADDR_GET_CURRENT_TARGET_POSITION, (uint8_t*)result, sizeof(int32_t));
    return 0;
}

misc_flags_1_t tic_get_misc_flags(int fd, uint8_t address)
{
    uint8_t result;
    tic_get_variable(fd, address, ADDR_MISC_FLAGS_1, &result, sizeof(uint8_t));

    misc_flags_1_t flag_1;
    flag_1.all = result;

    return flag_1;
}


// Gets the "Current position" variable from the Tic.
// Returns 0 for success, -1 for failure.
int tic_get_current_position(int fd, uint8_t address, int32_t * output)
{
    *output = 0;
    uint8_t buffer[4];
    int result = tic_get_variable(fd, address, ADDR_CURRENT_POSITION, buffer, sizeof(buffer));
    if (result) { return -1; }
    *output = buffer[0] + ((uint32_t)buffer[1] << 8) +
    ((uint32_t)buffer[2] << 16) + ((uint32_t)buffer[3] << 24);
    return 0;
}

int tic_get_current_velocity(int fd, uint8_t address, int32_t * output)
{
    *output = 0;
    uint8_t buffer[4];
    
    int result = tic_get_variable(fd, address, ADDR_CURRENT_VELOCITY, buffer, sizeof(buffer));
    
    if (result) { return -1; }
    *output = buffer[0] + ((uint32_t)buffer[1] << 8) +
    ((uint32_t)buffer[2] << 16) + ((uint32_t)buffer[3] << 24);
    return 0;
}


int tic_set_max_speed(int fd, uint8_t address, int32_t max_speed)
{
    uint8_t command[] = {
        ADDR_SET_MAX_SPEED,
        (uint8_t)(max_speed >> 0 & 0xFF),
        (uint8_t)(max_speed >> 8 & 0xFF),
        (uint8_t)(max_speed >> 16 & 0xFF),
        (uint8_t)(max_speed >> 24 & 0xFF),
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set max speed");
        return -1;
    }
    return 0;
}

int tic_set_starting_speed(int fd, uint8_t address, int32_t starting_speed)
{
    uint8_t command[] = {
        ADDR_SET_STARTING_SPEED,
        (uint8_t)(starting_speed >> 0 & 0xFF),
        (uint8_t)(starting_speed >> 8 & 0xFF),
        (uint8_t)(starting_speed >> 16 & 0xFF),
        (uint8_t)(starting_speed >> 24 & 0xFF),
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set starting speed");
        return -1;
    }
    return 0;
}

int tic_set_max_accel(int fd, uint8_t address, int32_t max_accel)
{
    uint8_t command[] = {
        ADDR_SET_MAX_ACCEL,
        (uint8_t)(max_accel >> 0 & 0xFF),
        (uint8_t)(max_accel >> 8 & 0xFF),
        (uint8_t)(max_accel >> 16 & 0xFF),
        (uint8_t)(max_accel >> 24 & 0xFF),
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set max accel");
        return -1;
    }
    return 0;
}

int tic_set_max_deccel(int fd, uint8_t address, int32_t max_deccel)
{
    uint8_t command[] = {
        ADDR_SET_MAX_DECELL,
        (uint8_t)(max_deccel >> 0 & 0xFF),
        (uint8_t)(max_deccel >> 8 & 0xFF),
        (uint8_t)(max_deccel >> 16 & 0xFF),
        (uint8_t)(max_deccel >> 24 & 0xFF),
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set max decell");
        return -1;
    }
    return 0;
}

int tic_set_current_limit(int fd, uint8_t address, uint8_t current_limit)
{
    uint8_t command[] = {
        ADDR_SET_CURRENT_LIMIT,
        current_limit
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set current limit");
        return -1;
    }
    return 0;
}

int tic_set_step_mode(int fd, uint8_t address, uint8_t step_mode)
{
    uint8_t command[] = {
        ADDR_SET_STEP_MODE,
        step_mode
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set step mode");
        return -1;
    }
    return 0;
}

int tic_set_reverse_mode(int fd, uint8_t address)
{
    uint8_t command[] = {
        ADDR_SET_REVERSE_MODE,
        128
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set reverse mode");
        return -1;
    }
    return 0;
}

int tic_deenergize(int fd, uint8_t address)
{
     uint8_t command[] = {
        ADDR_ENERGIZE
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to deenergize");
        return -1;
    }
    return 0;
}


int tic_go_home(int fd, uint8_t address, uint8_t direction) // 0 = reverse 1 = forward
{
    uint8_t command[] = {
        ADDR_GO_HOME,
        direction
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to go home");
        return -1;
    }
    return 0;
}

uint8_t mapCurrentLimit(double current_mA)
{
    constexpr double MAX_CURRENT_MA = 3968.0;
    constexpr uint8_t MAX_SETTING = 124;

    // Ensure the input is within the valid range
    if (current_mA <= 0) return 0;
    if (current_mA >= MAX_CURRENT_MA) return MAX_SETTING;

    // Compute the setting and round to the nearest integer
    return static_cast<uint8_t>(std::round((current_mA / MAX_CURRENT_MA) * MAX_SETTING));
}


int SetTicSettings(int fd, uint8_t address)
{
    tic_deenergize(fd, address);
    tic_set_current_limit(fd, address, mapCurrentLimit(TIC_CURRENT_LIMIT_MILLIAMPS));
    tic_set_max_accel(fd, address, STEPPER_MAX_ACCEL_PPS2*PPS2_UNIT_CONVERSION);
    tic_set_max_deccel(fd, address, STEPPER_MAX_DECELL_PPS2*PPS2_UNIT_CONVERSION);
    tic_set_max_speed(fd, address, STEPPER_MAX_PULSES_PER_SEC*PPS_UNIT_CONVERSION);
    tic_set_starting_speed(fd,address, STEPPER_START_SPEED_PPS*PPS_UNIT_CONVERSION);
    tic_set_step_mode(fd, address, 2); // so we get home faster
    
    if (address == 102) {
        tic_go_home(fd,address, 1);
    } else {
        tic_go_home(fd, address, 0);
    }


    misc_flags_1_t flags;
    flags = tic_get_misc_flags(fd, address);
    while (flags.bits.positionUncertain == true) // wait till we get home
    {
        flags = tic_get_misc_flags(fd, address);
        

        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    tic_set_step_mode(fd, address, STEPPER_STEP_MODE);

    std::cout << "Configured Tic: " << int(address) << std::endl;
    return 0;
}

int SendAllTicsHome(int fd, const uint8_t *addresses)
{
    tic_set_target_position(fd, 101, int32_t(Y_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES * STEPPER_STEP_MODE_NUMERIC));
    tic_set_target_position(fd, 100, int32_t(X_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES * STEPPER_STEP_MODE_NUMERIC));
    tic_set_target_position(fd, 102, int32_t(Z_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES * STEPPER_STEP_MODE_NUMERIC));
    return 0;
}
