#include <iostream>
#include <zmq.hpp>
#include <thread>
#include <random>
#include <csignal>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <fstream>

#include "controller.h"
#include "telemetry_conversion.h"
#include "motor_mapping.h"
#include "ClockManager.h"
#include "kalman.h"
#include "imu.h"
#include "tic.h"
#include "log.h"
#include "tmcl3110.hpp"

#define MAIN_LOOP_RATE_HZ (500.0f)
#define ACTUATION_RATE_HZ (100.0f)
#define LOGGING_RATE_HZ (100.0f)

#define MOTOR_FEEDBACK_RATE_HZ (200.0f)

#define TMCL3110_ADDRESS ("/dev/ttyACM0")

void signalHandler(int signum);
void MotorFeedbackLoop(telemetry_t * tele);
void LoadGainsFromJSON();
uint64_t getTimestamp();
void waitForKeyPress();

/* IMU access variables */
std::mutex imu_mutex;
imu_data_vn_format_t imu_data;

/* ZMQ Communications */
zmq::context_t context(1);
zmq::socket_t telemetry_socket(context, zmq::socket_type::pub);

/* Stepper motors */
static const std::vector<uint8_t> tic_id = {100,101,102}; // x y z motor id in the firmware of the tic. must match!
int stepper_i2c_fd;  // only 1 file descriptor for the i2c bus required 
std::mutex stepper_mutex;

bool kalman_filter_initialized = false;

static uint64_t exp_start_time;

int main()
{
    /* Signal catching stuff for clean exits */
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    /* Setup Logging */
    std::ofstream log_file;
    std::string filename = generateTimestampedFilename();
    log_file.open(filename);
    writeHeader(log_file);
    

    /* Add clocks for loop timing*/
    ClockManager clockManager;
    const auto loopDuration = duration<float>(1.0f / MAIN_LOOP_RATE_HZ);
    clockManager.AddClock("controller", 1.0f / CONTROLLER_RATE_HZ);
    // clockManager.AddClock("kalman", 1.0f / KALMAN_RATE_HZ);
    clockManager.AddClock("telemetry", 1.0f / TELEMETRY_SEND_RATE_HZ);
    clockManager.AddClock("logging", 1.0f / LOGGING_RATE_HZ);
    clockManager.AddClock("actuation", 1.0f / ACTUATION_RATE_HZ);
    clockManager.AddClock("motor_feedback", 1.0f / MOTOR_FEEDBACK_RATE_HZ);

    /* Start IMU communication*/
    ConnectAndConfigureIMU(&imu_data, &imu_mutex);
    
    /* Groundstation communication */
    telemetry_socket.bind("tcp://*:5555");  // Bind to TCP port 5555

    /* Data structure for telemetry */
    telemetry_t tele; // default initilziation to zeros already no need for memset zeros
    tele.r_mass << 0,0,0; // masses start at origin

    /* Load gains for controller from JSON*/
    LoadGainsFromJSON();

    /* Start TMCL communication*/
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
        std::vector<int32_t> target_positions = {30000, 30000, 10000}; // target positions for motors 0, 1, and 2 in microsteps
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

    // /* Start TIC communication */
    // stepper_i2c_fd = open_i2c_device(TIC_I2C_ADDRESS_DEVICE);
    // if (stepper_i2c_fd < 0) 
    // {
    //     std::cout << "Could not connect to i2c bus!!!" << std::endl;
    // } else // configure the tics
    // {
    //     // tic_set_reverse_mode(stepper_i2c_fd, 102);
    //     for (auto iter = tic_id.begin(); iter != tic_id.end(); iter++)
    //     {
    //         SetTicSettings(stepper_i2c_fd, *iter); 
    //     } 
    // }
    
    waitForKeyPress(); // manual starting of the experiment. steppers are sent to the origin and held there
    tare_heading(); // zero heading angle

    exp_start_time = getTimestamp(); 

    /* Set inertia and initial estimate values for controller, can do other things in here for controller if desired */
    InitController();

    while (true) 
    {
        auto start = high_resolution_clock::now(); // timestamp

        /* update measurements by pulling most recent data from IMU */
        if (imu_mutex.try_lock()) {
            imu_data.PullMeasurement(&tele); // pulls measurement from IMU into our current telemetry
            imu_mutex.unlock();

            if (!kalman_filter_initialized)
            {
                InitKalmanFilter(tele.omega_b2i_B, tele.q_i2b);
                kalman_filter_initialized = true;
            }
        } 
        
        /* Get Mass Positions and Velocities from motor controllers */
        if (clockManager.Elapsed("motor_feedback").first)
        {
            int32_t motor_positions[3]; int32_t motor_velocities[3];
            for (int i = 0; i<3; i++)
            {
                tmcl.getActualPosition(i, &motor_positions[i]);
                tmcl.getActualSpeedInt(i, &motor_velocities[i]);
            }
    
            // Map shaft position and velocity to moving mass position and velocity
            tele.r_mass = ConvertMotorPositionToMassPosition(motor_positions[0], motor_positions[1], motor_positions[2]);
            tele.rdot_mass = ConvertMotorSpeedToMassVelocity(motor_velocities[0], motor_velocities[1], motor_velocities[2]);
        }

        /* Run the controller */
        auto check_controller_clock = clockManager.Elapsed("controller");        
        if (check_controller_clock.first && (imu_data.valid_data == true)) // if sufficient time elapsed and we have imu data
        {
            tele = Controller(tele, check_controller_clock.second.count());
            // tele = PD_Controller(tele,  check_controller_clock.second.count());
            tele.time = getTimestamp() - exp_start_time; 
        }

        /* Actuate motors */
        if (clockManager.Elapsed("actuation").first)
        {
            std::vector<int32_t> target_positions = ConvertMassPositionToMotorPosition(
                tele.r_mass_commanded.x(), tele.r_mass_commanded.y(),  tele.r_mass_commanded.z());
            // Send motor position commands to motors
            for (uint8_t i = 0; i<3; i++) 
            {
                auto r = tmcl.MVP_ABS(i, target_positions[i]);
            if (r.status != 100) throw std::runtime_error("MVP_ABS failed for motor " + std::to_string(i));
            }
        }

        /* Send telemetry to groundstation */
        auto check_tele_clock = clockManager.Elapsed("telemetry");        
        if (check_tele_clock.first)
        {
            // Create and populate a telemetry message
            TelemetryMessage msg = toProto(tele); // Convert to Protobuf        
            std::string serialized_msg; msg.SerializeToString(&serialized_msg); 
            zmq::message_t zmq_msg(serialized_msg.data(), serialized_msg.size());
            telemetry_socket.send(zmq_msg, zmq::send_flags::none);
            // tele.Disp();
        }

        if (clockManager.Elapsed("logging").first)
        {
            LogTele(log_file, tele);
        }
        
        /* Loop timing business */
        auto end = high_resolution_clock::now();
        duration<float> workTime = end - start; // Calculate the elapsed time for the work
        duration<float> sleepTime = loopDuration - workTime;  // Calculate remaining time to wait to achieve the desired frequency
        // If the work took longer than the loop duration, we skip sleeping, else sleep
        if (sleepTime > duration<float>::zero()) {
            std::this_thread::sleep_for(sleepTime);
        }    
    }
}

void signalHandler(int signum) {
    std::cout << "\nCaught signal " << signum << ", performing cleanup...\n";
    // deenergize steppers on exit
    for (auto iter = tic_id.begin(); iter != tic_id.end(); iter++)
    {
        tic_deenergize(stepper_i2c_fd, *iter);
    } 
    std::exit(signum);
}

Matrix3d loadMatrix(const nlohmann::json& j) {
    Matrix3d mat;
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) {  // Use `k` instead of `j` to avoid confusion
            mat(i, k) = j[i][k].get<double>();  // Correct indexing
        }
    }
    return mat;
}

void LoadGainsFromJSON()
{
    ifstream file("/home/pi/moving-mass-control/m2sat/gains.json");
    nlohmann::json j;
    file >> j;
    SetGains(loadMatrix(j["K_1"]), loadMatrix(j["K_2"]), loadMatrix(j["K_3_a"]), loadMatrix(j["K_4"]), 
        loadMatrix(j["alpha_1_a"]), loadMatrix(j["alpha_2_a"]),
        loadMatrix(j["gamma_gain_a"]), loadMatrix(j["CL_gain"]), loadMatrix(j["adaptive_gain"]),
        loadMatrix(j["K_3_b"]), loadMatrix(j["alpha_1_b"]), loadMatrix(j["alpha_2_b"]), loadMatrix(j["gamma_gain_b"]),
        loadMatrix(j["gamma_gain_c"]));
}

uint64_t getTimestamp()
{
    // Get the current time point from the system clock
    auto now = std::chrono::system_clock::now();
    // Convert the time point to a duration since epoch
    auto duration = now.time_since_epoch();
    // Convert the duration to milliseconds
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    return static_cast<uint64_t>(milliseconds);
}

void setNonBlockingInput() {
    struct termios newt;
    tcgetattr(STDIN_FILENO, &newt);  // Get current terminal settings
    newt.c_lflag &= ~(ICANON | ECHO); // Disable line buffering and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Set stdin to non-blocking mode
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

void restoreTerminalSettings(struct termios& oldt) {
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // Restore terminal settings
}

void waitForKeyPress() {
    std::cout << "Press any key to start the experiment\n";

    struct termios oldt;
    tcgetattr(STDIN_FILENO, &oldt);  // Save current terminal settings
    setNonBlockingInput();

    while (true) {
        /* Send to origin to not timeout */
        SendAllTicsHome(stepper_i2c_fd, tic_id.data());

        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            break; // Break on key press
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    restoreTerminalSettings(oldt);
}