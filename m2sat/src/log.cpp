#include "log.h"

std::string generateTimestampedFilename() 
{
    // Get the current time
    std::time_t now = std::time(nullptr);
    std::tm* now_tm = std::localtime(&now);

    // Create a stringstream to format the date and time
    std::stringstream ss;
    ss << "data_"
       << std::put_time(now_tm, "%m%d___%H_%M_%S_mmc")
       << ".csv";

    // Return the formatted string
    return ss.str();
}


void writeHeader(std::ofstream& file) {
    // Vectors sizes
    const int omega_size = 3;
    const int q_size = 4;
    const int euler_size = 3;

    const int Omega_size = 4;
    const int Gamma_size = 4;
    const int Lr_size = 3;
    std::stringstream ss;

    // Write the header to the CSV file
   
ss << "time" << ","
<< "omega_b2i_0" << ","
<< "omega_b2i_1" << ","
<< "omega_b2i_2" << ","
<< "q_b2i_0" << ","
<< "q_b2i_1" << ","
<< "q_b2i_2" << ","
<< "q_b2i_3" << ","
<< "q_i2d_0" << ","
<< "q_i2d_1" << ","
<< "q_i2d_2" << ","
<< "q_i2d_3" << ","
<< "r_mass_0" << ","
<< "r_mass_1" << ","
<< "r_mass_2" << ","
<< "rdot_mass_0" << ","
<< "rdot_mass_1" << ","
<< "rdot_mass_2" << ","
<< "r_mass_commanded_0" << ","
<< "r_mass_commanded_1" << ","
<< "r_mass_commanded_2" << ","
<< "u_com_0" << ","
<< "u_com_1" << ","
<< "u_com_2" << ","
<< "u_actual_0" << ","
<< "u_actual_1" << ","
<< "u_actual_2" << ","
<< "theta_hat_0" << ","
<< "theta_hat_1" << ","
<< "theta_hat_2" << ","
<< "omega_d2i_0" << ","
<< "omega_d2i_1" << ","
<< "omega_d2i_2" << ",""\n";

    // Write the stringstream content to the file
    file << ss.str();
    file.flush();
}

void LogTele(std::ofstream& file, const telemetry_t & tele)
{
    Matrix<double,1,1>  time_mat; time_mat << double(tele.time);
    saveData(file, time_mat);
    saveData(file, tele.omega_b2i_B);
    saveData(file, tele.q_i2b.coeffs());
    saveData(file, tele.q_i2d.coeffs());
    saveData(file, tele.r_mass);
    saveData(file, tele.rdot_mass);
    saveData(file, tele.r_mass_commanded);
    saveData(file, tele.u_com);
    saveData(file, tele.u_actual);
    saveData(file, tele.theta_hat);
    saveData(file, tele.omega_d2i_d);
    file << "\n";
}


void saveData(std::ofstream& file, MatrixXd  matrix)
{
	if (file.is_open())
	{
        for (int i = 0; i < matrix.rows(); ++i) {
            for (int j = 0; j < matrix.cols(); ++j) {
                file << matrix(i, j);
                file << ",";
            }
        }

    }
}

