#pragma once

#include <iostream>
#include <mutex>
#include <Eigen/Geometry> 
#include <Eigen/Dense>

#include "vn/sensors.h" // Include this header file to get access to VectorNav sensors.
#include "vn/thread.h" // We need this file for our sleep function.

#include "telemetry.h"
// #include <types.h>

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

#define VN_IMU_SERIAL_PORT "/dev/ttyUSB0"

/* IF THE  VN_IMU_OUTPUT_FORMAT IS CHANGED, THEN THE PARSE FUNCTION IN imu.c MUST ALSO BE CHANGED*/
static const  vn::protocol::uart::AsciiAsync imu_connection_mode = vn::protocol::uart::VNQTR; //= vn::protocol::uart::VNQMR;

static const Quaterniond q_b2imu(std::sqrt(0.5), 0, std::sqrt(0.5), 0); // imu is rotated around body frame y axis by positive 90 degrees which is this quaternion

struct imu_data_vn_format_t
{
    vec4f quaternion; // pose
    vec3f magnetic; // ignored for now
    vec3f acceleration; //m/s^2
    vec3f angularRate; // rad/s

    bool valid_data = false; // set to true at the first instance of data received, prevents controller from running without data

    void PullMeasurement(telemetry_t * tele) // fill our telemetry packet 
    {   
        // convert to Eigen quaternion
        tele->q_i2b.w() = (quaternion.w); // shortest principle rotation correspondes to always positive first scalar component
        tele->q_i2b.x() = quaternion.x;
        tele->q_i2b.y() = quaternion.y;
        tele->q_i2b.z() = quaternion.z;

        // body angular velocity
        tele->omega_b2i_B.x() = angularRate.x; 
        tele->omega_b2i_B.y() = angularRate.y; 
        tele->omega_b2i_B.z() = angularRate.z;    
    }    
};

void ConnectAndConfigureIMU(imu_data_vn_format_t * imu_data, std::mutex * imu_mutex);
void tare_heading();


class LowPassFilter {
public:
    LowPassFilter(float alpha) : alpha_(alpha), initialized_(false), y_prev_(0.0f) {}

    float update(float x) {
        if (!initialized_) {
            y_prev_ = x;  // Initialize with first input
            initialized_ = true;
        }
        float y = alpha_ * x + (1.0f - alpha_) * y_prev_;
        y_prev_ = y;
        return y;
    }

private:
    float alpha_;
    float y_prev_;
    bool initialized_;
};
