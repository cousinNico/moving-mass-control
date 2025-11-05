//test_tmcl.cpp

/*
- Test program for TMCL communication with TMC3110 motor driver module
- Sets up the TMCL3110, reads motor positions and velocities, and commands target positions
- Uses the TMCL3110 class defined in tmcl3110.hpp

- Author: Nicolo Woodward
- Date: November 2025


compile with: g++ -o test_tmcl test_tmcl.cpp -I../include

*/

#include "tmcl3110.hpp"
#include <iostream>
#include <cmath>

#define TMCL3110_ADDRESS ("/dev/ttyUSB0")

/* Reference for Axis Parameters 
https://www.trinamic.com/fileadmin/assets/Products/Modules/TMC1310_Datasheet.pdf  8.3.1 Axis Parameters
SAP/GAP 0: Target Position  - SAP internally handled by MVP and ROR/ROL commands
SAP/GAP 1: Actual Position
SAP/GAP 2: Target Velocity  - SAP internally handled by MVP and ROR/ROL commands
SAP/GAP 3: Actual Velocity
SAP/GAP 4: Max Velocity
SAP/GAP 5: Max Acceleration
*/

int main() {
    try {
        // Initialize serial connection with the microcontroller module
        TMCL3110 tmcl(TMCL3110_ADDRESS, 115200, /*module addr*/ 1);

        // Setup max velocity and max acceleration for the three motors
        for (uint8_t m=0; m<3; ++m) {
            auto r1 = tmcl.SAP(/*param=*/4, m, /*max vel*/ STEPPER_MAX_PULSES_PER_SEC*PPS_UNIT_CONVERSION); // SAP 4 (Max Velocity)
            auto r2 = tmcl.SAP(/*param=*/5, m, /*max acc*/ STEPPER_MAX_ACCEL_PPS2*PPS2_UNIT_CONVERSION); // SAP 5 (Max Acceleration)
            if (r1.status!=100 || r2.status!=100) throw std::runtime_error("SAP failed");
        }
        
        // Example: read actual positions and velocities
        int32_t motor_positions[3]; int32_t motor_velocities[3]; // arrays to hold motor positions and velocities
        try
        {
            for (int i = 0; i<3; i++)
            {
                tmcl.getActualPosition(i, &motor_positions[i]);
                tmcl.getActualSpeedInt(i, &motor_velocities[i]);
            }
            std::cout << "Motor Positions: " << motor_positions[0] << ", " << motor_positions[1] << ", " << motor_positions[2] << "\n";
            std::cout << "Motor Velocities: " << motor_velocities[0] << ", " << motor_velocities[1] << ", " << motor_velocities[2] << "\n";
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error reading motor positions/velocities: " << e.what() << "\n";
        }
        
        // Example: set absolute target position
        std::vector<int32_t> target_positions = {10000, -10000, 0}; // target positions for motors 0, 1, and 2 in microsteps
        for (int i = 0; i<3; i++){
            auto r = tmcl.MVP_ABS(i, target_positions[i]);
            if (r.status != 100) throw std::runtime_error("MVP_ABS failed for motor " + std::to_string(i));
        }
        while (tmcl.getActualPosition(0, &motor_positions[0]), motor_positions[0] != target_positions[0] ||
               tmcl.getActualPosition(1, &motor_positions[1]), motor_positions[1] != target_positions[1] ||
               tmcl.getActualPosition(2, &motor_positions[2]), motor_positions[2] != target_positions[2])
        {
            std::cout << "Motor Positions: " << motor_positions[0] << ", " << motor_positions[1] << ", " << motor_positions[2] << "\n";
        }

        // Stop all motors
        for (int i = 0; i<3; i++){
            tmcl.MST(i);
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
