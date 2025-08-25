#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <iostream> 

#define TELEMETRY_SEND_RATE_HZ (60.0f) 
using namespace Eigen;
typedef Vector<double, 6> Vector6d;

struct telemetry_t 
{
    uint64_t time = 0;
    Quaterniond q_i2b = Quaterniond::Identity(); // q from body to inertial (from IMU)
    Quaterniond q_i2d = Quaterniond::Identity();
    Vector3d omega_b2i_B = Vector3d::Zero(); // angular velocity of b2i in body frame (from IMU) rad/sec
    Vector3d r_mass = Vector3d::Zero(); //mass postion meters
    Vector3d rdot_mass = Vector3d::Zero(); // mass velocity meters / sec
    Vector3d r_mass_commanded = Vector3d::Zero(); // commanded mass position meters
    Vector3d u_com = Vector3d::Zero(); // commanded torque Nm
    Vector3d u_actual = Vector3d::Zero(); // actual control torque Nm
    Vector6d nu; // kalman filter estimate
    Vector3d theta_hat = Vector3d::Zero();
    Vector3d omega_d2i_d = Vector3d::Zero();

//     Quaterniond get_q_i2b()
//     {
//         return q_b2i.conjugate();
//     }

    void Disp() {
    std::cout << "Time: " << time << " ms" << std::endl;
    // std::cout << "q_b2i (Quaternion from body to inertial): "
            //   << q_b2i.coeffs().transpose() << std::endl;
    // std::cout << "q_i2d (Quaternion from inertial to desired): "
            //   << q_i2d.coeffs().transpose() << std::endl;
    // std::cout << "omega_b2i_B (Angular velocity of b2i in body frame): "
            //   << omega_b2i_B.transpose() << std::endl;
    std::cout << "r_mass (Mass position): " << r_mass.transpose() << std::endl;
    // std::cout << "rdot_mass (Mass velocity): " << rdot_mass.transpose() << std::endl;
    std::cout << "r_mass_commanded (Commanded mass position): "
              << r_mass_commanded.transpose() << std::endl;
    std::cout << "u_com (Commanded torque): " << u_com.transpose() << std::endl;
    std::cout << "u_actual (Actual control torque): " << u_actual.transpose() << std::endl;
    // std::cout << "theta_hat (Estimated offset): " << theta_hat.transpose() << std::endl;
    std::cout << "+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=\n";
}
};
