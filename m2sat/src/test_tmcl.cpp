//test_tmcl.cpp

/*
- Test program for TMCL communication with TMC3110 motor driver module
- Sets up the TMCL3110, reads motor positions and velocities, and commands target positions
- Uses the TMCL3110 class defined in tmcl3110.hpp

- Author: Nicolo Woodward
- Date: November 2025


compile with:
g++ -o test_tmcl test_tmcl.cpp -I../include

*/

#include "tmcl3110.hpp"
#include <iostream>
#include <cmath>

#define TMCL3110_ADDRESS ("/dev/ttyACM0")

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

        tmcl.setMicrostepResolution(0, 1);  // 1- fullstep
        tmcl.setMicrostepResolution(1, 1);  // 1- fullstep
        tmcl.setMicrostepResolution(2, 1);  // 1- fullstep

        // Setup max velocity and max acceleration for the three motors
        for (uint8_t m=0; m<3; ++m) {
            auto r1 = tmcl.SAP(/*param=*/4, m, /*max vel*/ 500); // SAP 4 (Max Velocity)
            auto r2 = tmcl.SAP(/*param=*/5, m, /*max acc*/ 50); // SAP 5 (Max Acceleration)
            if (r1.status!=100 || r2.status!=100) throw std::runtime_error("SAP failed");
        }
        std::vector<int32_t> target_positions = {20000, 20000, 30000}; // target positions for motors 0, 1, and 2 in microsteps
        // Setup max current for each motor
        for (uint8_t m=0; m<3; ++m) {
            auto r3 = tmcl.SAP(/*param=*/6, m, /*max current*/ 62); // SAP 6 (Max Current) 0-255
            if (r3.status!=100) throw std::runtime_error("SAP failed");
        }

        // Setup standby current for each motor
        for (uint8_t m=0; m<3; ++m) {
            auto r3 = tmcl.SAP(/*param=*/7, m, /*standby current*/ 5); // SAP 7 (Standby Current) 0-255
            if (r3.status!=100) throw std::runtime_error("SAP failed");
        }

        std::cout<<"Motors setup complete\n";

        // Example: read actual positions and velocities
        int32_t motor_positions[3]; int32_t motor_velocities[3]; // arrays to hold motor positions and velocities
        
        try
        {
            for (int i = 0; i<3; i++)
            {
                tmcl.getActualPosition(i, &motor_positions[i]);
                tmcl.getActualSpeedInt(i, &motor_velocities[i]);
            }
            std::cout << "Motor Positions: " << motor_positions[0] << "\n";
            std::cout << "Motor Velocities: " << motor_velocities[0] << "\n";
        }
        catch(const std::exception& e)
        {
            std::cerr << "Error reading motor positions/velocities: " << e.what() << "\n";
        }
        std::cout<<"Motor positions and velocities read successfully\n";

        // Example: set absolute target position

        for (int i = 0; i<3; i++){
            auto r = tmcl.MVP_ABS(i, target_positions[i]);
            if (r.status != 100) throw std::runtime_error("MVP_ABS failed for motor " + std::to_string(i));
        }
        while (tmcl.getActualPosition(0, &motor_positions[0]), motor_positions[0] != target_positions[0], tmcl.getActualPosition(1, &motor_positions[1]), motor_positions[1] != target_positions[1])
        {
            tmcl.getActualSpeedInt(0, &motor_velocities[0]);
            tmcl.getActualSpeedInt(1, &motor_velocities[1]);
            std::cout << "Motor Positions: " << motor_positions[0] << " " << motor_positions[1] << "\n";
            std::cout << "Motor Velocities: " << motor_velocities[0] << " " << motor_velocities[1] << "\n";
        }
        

        std::cout<<"HERE"<<std::endl;
        
        // Stop all motors
        for (int i = 0; i<3; i++){
            tmcl.MST(i);
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        TMCL3110 tmcl(TMCL3110_ADDRESS, 115200, /*module addr*/ 1);
        tmcl.MST(0);
        tmcl.MST(1);
        tmcl.MST(2);
        // Setup max current to 0 for each motor
        for (uint8_t m=0; m<3; ++m) {
            auto r3 = tmcl.SAP(/*param=*/6, m, /*max current*/ 0); // SAP 6 (Max Current) 0-255
            if (r3.status!=100) throw std::runtime_error("SAP failed");
        }

        // Setup standby current to 0 for each motor
        for (uint8_t m=0; m<3; ++m) {
            auto r3 = tmcl.SAP(/*param=*/7, m, /*standby current*/ 0); // SAP 7 (Standby Current) 0-255
            if (r3.status!=100) throw std::runtime_error("SAP failed");
        }
        return 1;
    }
    TMCL3110 tmcl(TMCL3110_ADDRESS, 115200, /*module addr*/ 1);
        tmcl.MST(0);
        tmcl.MST(1);
        tmcl.MST(2);
        // Setup max current to 0 for each motor
        for (uint8_t m=0; m<3; ++m) {
            auto r3 = tmcl.SAP(/*param=*/6, m, /*max current*/ 0); // SAP 6 (Max Current) 0-255
            if (r3.status!=100) throw std::runtime_error("SAP failed");
        }

        // Setup standby current to 0 for each motor
        for (uint8_t m=0; m<3; ++m) {
            auto r3 = tmcl.SAP(/*param=*/7, m, /*standby current*/ 0); // SAP 7 (Standby Current) 0-255
            if (r3.status!=100) throw std::runtime_error("SAP failed");
        }
}
