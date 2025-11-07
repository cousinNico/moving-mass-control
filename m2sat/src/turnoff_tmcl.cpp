//turnoff_tmcl.cpp

/*
- turnoff program for TMCL communication with TMC3110 motor driver module
- Sets up the TMCL3110, reads motor positions and velocities, and commands target positions
- Uses the TMCL3110 class defined in tmcl3110.hpp

- Author: Nicolo Woodward
- Date: November 2025


compile with:
g++ -o turnoff_tmcl turnoff_tmcl.cpp -I../include

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
        // Initialize serial connection with the microcontroller module
        TMCL3110 tmcl(TMCL3110_ADDRESS, 115200, /*module addr*/ 1);

        tmcl.setMicrostepResolution(0, 1);  // 1- fullstep
        tmcl.setMicrostepResolution(1, 1);  // 1- fullstep
        tmcl.setMicrostepResolution(2, 1);  // 1- fullstep

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
