#include <thread>
#include <chrono>

#include "imu.h"
#include "vn/thread.h" // We need this file for our sleep function.
#include "vn/sensors.h" // Include this header file to get access to VectorNav sensors.

// I dont normally like namespaces but we dont use the VN lib too much 
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

static VnSensor vs; // this needs to be defined outside of a temporary function stack like the ConnectAndConfigure, since it needs to stay alive
static std::mutex * imu_mutex_;

#define VN_IMU_BAUD_RATE 115200
#define VN_IMU_UPDATE_HZ 40

void asciiAsyncMessageReceived(void* imu_struct, Packet& p, size_t index);

void ConnectAndConfigureIMU(imu_data_vn_format_t * imu_data, std::mutex * imu_mutex)
{
    imu_mutex_ = imu_mutex;
	
    vs.connect(VN_IMU_SERIAL_PORT,   VN_IMU_BAUD_RATE); // connect to sensor
	
    /* Heading 2D */
    FilterBasicControlRegister reg = vs.readFilterBasicControl();
    reg.magMode = MAGNETICMODE_2D; // magnetometer only effects heading
    vs.writeFilterBasicControl(reg, 1);

    /* Tell IMU it is rotated 90 degrees about the y axis of the body */
    // The IMU needs UNPLUGGED and REPLUGGED if this matrix changes to apply the change
    vn::math::mat3f R_imu2body_y(
        0.0,  0.0f,  -1.0f, 
		0.0f, 1.0f,  0.0f, 
		1.0f,  0.0f, 0.0f);
    vn::math::mat3f R_imu2body_z(
        0.0f,  -1.0f,   0.0f,
        1.0f,   0.0f,   0.0f,
        0.0f,   0.0f,   1.0f);
    
    vn::math::mat3f R_body2principal(
        0.964273155673016,  -0.264591257791476,   0.012990286706965,
        0.264876543734282,   0.963770388740287,  -0.031417421389680,
       -0.004206878627359,   0.033735798311594,   0.999421931960918);

    // const vn::math::mat3f R_imu2principal = R_body2principal * (R_imu2body_z * R_imu2body_y);

	vs.writeReferenceFrameRotation(R_imu2body_y, true);

    /* Indoor mode for heading */
    VpeBasicControlRegister vpeReg = vs.readVpeBasicControl();
    vpeReg.headingMode = HEADINGMODE_INDOOR;
    vpeReg.filteringMode = VPEMODE_MODE1;
    vs.writeVpeBasicControl(vpeReg);
    
    VpeAccelerometerBasicTuningRegister vpeAccReg = vs.readVpeAccelerometerBasicTuning();
    vpeAccReg.adaptiveFiltering.x = 10; // 10 is max, 5 is default. higher seems better to kill vibrations from masses
    vpeAccReg.adaptiveFiltering.y = 10; // 10 is max, 5 is default. higher seems better to kill vibrations from masses
    vpeAccReg.adaptiveFiltering.z = 10; // 10 is max, 5 is default. higher seems better to kill vibrations from masses
    vs.writeVpeAccelerometerBasicTuning(vpeAccReg);

    vs.writeAsyncDataOutputFrequency(VN_IMU_UPDATE_HZ); // set output frequency
	uint32_t newHz = vs.readAsyncDataOutputFrequency();

	vs.writeSettings(true); 


    vs.writeAsyncDataOutputType(imu_connection_mode); // set data output format, false = dont wait for reply
    vs.registerAsyncPacketReceivedHandler((void *) imu_data, asciiAsyncMessageReceived); // register callback function

}

void tare_heading()
{
    vs.tare(1);
}

void asciiAsyncMessageReceived(void* imu_struct, Packet& p, size_t index)
{
    if (!imu_mutex_) // make sure our mutex exists
        return;

    if(p.determineAsciiAsyncType() == imu_connection_mode) { // just double check we received the right packet type
        
        imu_data_vn_format_t * imu_output_ptr = static_cast<imu_data_vn_format_t*>(imu_struct); // casting into usable struct
        
        // must lock 
        if (imu_mutex_->try_lock()) { 
            /* VERY IMPORTANT: IF WE CHANGE THE OUTPUT FORMAT, THIS PARSE MUST ALSO CHANGE */
            // p.parseVNICM(&imu_ptr->yawPitchRoll, &imu_ptr->magnetic, &imu_ptr->acceleration, &imu_ptr->angularRate); // copy data into our structs
            imu_data_vn_format_t imu_data;
            imu_data_vn_format_t * imu_ptr = &imu_data;

            // p.parseVNQMR(&imu_ptr->quaternion, &imu_ptr->magnetic, &imu_ptr->acceleration, &imu_ptr->angularRate);
            p.parseVNQTR(&imu_ptr->quaternion, &imu_ptr->angularRate);

            LowPassFilter omega[3] = {0.1, 0.1, 0.1};

            imu_output_ptr->quaternion = imu_ptr->quaternion;
            // imu_output_ptr->magnetic = imu_ptr->magnetic;
            // imu_output_ptr->acceleration = imu_ptr->acceleration;
            vec3f output;

            output.x = omega[0].update(imu_ptr->angularRate.x);
            output.y = omega[1].update(imu_ptr->angularRate.y);
            output.z = omega[2].update(imu_ptr->angularRate.z);
            
            imu_output_ptr->angularRate = output; //imu_ptr->angularRate;


            imu_output_ptr->valid_data = true;
            imu_mutex_->unlock(); 
        }

		// std::cout << "got imu" << std::endl;
    }
}