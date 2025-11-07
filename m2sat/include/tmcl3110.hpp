// tmcl3110.hpp

/*
Header file for TMCL3110 stepper motor controller communication
Ideally this would be in a library but until validated, keep it here

- Author: Nicolo Woodward
- Date: November 2025

*/

#pragma once
#include <cstdint>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>


/* Imported from tic.h--- need to be adjusted */
    #define RAD_TO_REV (1/(2*M_PI))

    #define STEPPER_MAX_ACCEL_PPS2  (100000.0f)  //80000 on 4th steps  // pulses per second squared, if we go higher, we need more current but we are already current limiting
    #define STEPPER_MAX_DECELL_PPS2 (100000.0f) //60000 on 4th steps
    #define TIC_CURRENT_LIMIT_MILLIAMPS (1800)
    #define STEPPER_STEP_MODE_NUMERIC (8.0f) // 2 half step, 4 quater etc
    #define STEPPER_STEP_MODE (3) 
    /* per the stepper_step_mode documentation from polulu
    0: Full step    1: 1/2 step    2: 1/4 step    3: 1/8 step    4: 1/16 step (Tic T834, Tic T825, and Tic 36v4 only)
    */

    #define STEPPER_STEPS_PER_REV (200.0f * STEPPER_STEP_MODE_NUMERIC) // (1.8 degree step angle full step)
    #define PPS2_UNIT_CONVERSION (100.0f) // polulu uses pulses per 10,000 seconds instead of per second (pulses per second squared)
    #define PPS_UNIT_CONVERSION (10000.0f) // polulu uses pulses per 10,000 seconds instead of per second (pulses per second)

    /* Stepper max rates */
    #define STEPPER_MAX_RAD_PER_SEC (50.0f) 
    #define STEPPER_MAX_PULSES_PER_SEC (STEPPER_MAX_RAD_PER_SEC * RAD_TO_REV * STEPPER_STEPS_PER_REV * STEPPER_STEP_MODE_NUMERIC)
    #define STEPPER_START_SPEED_PPS (0.0f) // speed the motor tries to start at, if too high, it stalls


    /* Physical parameters */
    #define X_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES (318.0f) // measured in full pulses
    #define Y_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES (325.0f)
    #define Z_OFFSET_FROM_LIMIT_SWITCH_WHOLE_PULSES (-225.0f)

class TMCL3110 {
public:
    struct Reply {
        uint8_t reply_addr;   // host address
        uint8_t module_addr;  // module address
        uint8_t status;       // 100 = OK
        uint8_t cmd;          // echoes command
        int32_t value;        // big-endian signed
    };

    TMCL3110(const std::string& serialPath, int baud=115200, uint8_t moduleAddr=1)
    : fd_(-1), addr_(moduleAddr) {
        fd_ = openSerial_(serialPath.c_str(), baud);
        if (fd_ < 0) throw std::runtime_error("Failed to open serial port");
    }

    ~TMCL3110() { if (fd_>=0) ::close(fd_); }

    // --- High-level helpers ---
    Reply SAP(uint8_t param, uint8_t motor, int32_t value)   { return send_(5, param, motor, value); } // Set Axis Param
    Reply GAP(uint8_t param, uint8_t motor)                  { return send_(6, param, motor, 0); }     // Get Axis Param
    Reply MVP_ABS(uint8_t motor, int32_t pos)                { return send_(4, 0, motor, pos); }      // type=0 => ABS
    Reply MVP_REL(uint8_t motor, int32_t delta)              { return send_(4, 1, motor, delta); }    // type=1 => REL
    Reply ROR(uint8_t motor, int32_t vel)                    { return send_(1, 0, motor, vel); }
    Reply ROL(uint8_t motor, int32_t vel)                    { return send_(2, 0, motor, vel); }
    Reply MST(uint8_t motor)                                 { return send_(3, 0, motor, 0); }
    Reply RFS(uint8_t mode, uint8_t motor)                   { return send_(13, mode, motor, 0); }    // reference search
    Reply SIO(uint8_t port, uint8_t bank, int32_t value)     { return send_(14, port, bank, value); } // set output
    Reply GIO(uint8_t port, uint8_t bank)                    { return send_(15, port, bank, 0); }     // read input

    int32_t getActualPosition(uint8_t motor, int32_t * output) {         // µsteps
        auto r = GAP(/*param=*/1, motor);
        if (r.status != 100) throw std::runtime_error("GAP(1) failed");
        *output = r.value;
        return r.value;
    }

    int32_t getActualSpeedInt(uint8_t motor, int32_t * output) {         // internal units [-2047..2047]
        auto r = GAP(/*param=*/3, motor);
        if (r.status != 100) throw std::runtime_error("GAP(3) failed");
        *output = r.value;
        return r.value;
    }
    
    void setMicrostepResolution(uint8_t motor, int resolutionPow2)
    {
        if (resolutionPow2 < 0 || resolutionPow2 > 8)
            throw std::invalid_argument("Microstep index must be 0–8");
        auto r = SAP(140, motor, resolutionPow2);
        if (r.status != 100)
            throw std::runtime_error("Failed to set microstep resolution");
    }

private:
    int fd_;
    uint8_t addr_;

    static uint8_t checksum_(const std::array<uint8_t,8>& b) {
        uint8_t sum = 0;
        for (auto v : b) sum += v;
        return sum;
    }

    static void i32_to_be(int32_t v, uint8_t out[4]) {
        out[0] = uint8_t((v >> 24) & 0xFF);
        out[1] = uint8_t((v >> 16) & 0xFF);
        out[2] = uint8_t((v >>  8) & 0xFF);
        out[3] = uint8_t((v >>  0) & 0xFF);
    }
    static int32_t be_to_i32(const uint8_t in[4]) {
        return (int32_t(in[0])<<24) | (int32_t(in[1])<<16) | (int32_t(in[2])<<8) | int32_t(in[3]);
    }

    Reply send_(uint8_t cmd, uint8_t type, uint8_t motorOrBank, int32_t value) {
        std::array<uint8_t,9> frame{};
        frame[0] = addr_;
        frame[1] = cmd;
        frame[2] = type;
        frame[3] = motorOrBank;
        i32_to_be(value, &frame[4]);

        std::array<uint8_t,8> tmp{};
        for (int i=0;i<8;++i) tmp[i] = frame[i];
        frame[8] = checksum_(tmp);

        writeAll_(frame.data(), frame.size());

        uint8_t rbuf[9];
        readAll_(rbuf, 9);

        Reply r;
        r.reply_addr = rbuf[0];
        r.module_addr= rbuf[1];
        r.status     = rbuf[2];
        r.cmd        = rbuf[3];
        r.value      = be_to_i32(&rbuf[4]);
        return r;
    }

    void writeAll_(const uint8_t* data, size_t len) {
        size_t off=0;
        while (off<len) {
            ssize_t n = ::write(fd_, data+off, len-off);
            if (n<=0) throw std::runtime_error("serial write failed");
            off += size_t(n);
        }
        ::tcdrain(fd_);
    }
    void readAll_(uint8_t* out, size_t len) {
        size_t off=0;
        while (off<len) {
            ssize_t n = ::read(fd_, out+off, len-off);
            if (n<=0) throw std::runtime_error("serial read failed");
            off += size_t(n);
        }
    }
    static int openSerial_(const char* path, int baud) {
        int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd<0) return fd;
        termios tio{};
        ::tcgetattr(fd, &tio);
        cfmakeraw(&tio);
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~CSTOPB;      // 1 stop bit
        tio.c_cflag &= ~PARENB;      // no parity
        tio.c_cflag &= ~CRTSCTS;     // no HW flow
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 10;        // read timeout (1.0s)
        speed_t sp = B115200; // default; adjust if needed
        switch (baud) {
            case 9600: sp=B9600; break;
            case 19200: sp=B19200; break;
            case 38400: sp=B38400; break;
            case 57600: sp=B57600; break;
            case 115200: default: sp=B115200; break;
        }
        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);
        ::tcsetattr(fd, TCSANOW, &tio);
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
        return fd;
    }

};
