#include "controller.h"
#include "kalman.h"
#include <vector>
#include <iostream>

using namespace Eigen;

static int total_pts = 0;

static std::vector<Vector<double, 9>> Phi_stack;
static std::vector<Vector<double, 3>> Tau_stack; // CL stacks
// static Vector3d omega_b2i_I_0;

static Matrix3d J; // time varying moment of inertia

// Helper functions
Vector3d quat_rotate(const Quaterniond& q, const Vector3d& v);
Matrix3d skew(const Vector3d& v);
Quaterniond quat_mult(const Quaterniond& q, const Quaterniond& p);
int rank(MatrixXd mat);
Vector3d projected_update(const Vector3d& theta_hat, const Matrix3d& Phi, Vector3d r,
    const Vector3d& theta_hat_min_, const Vector3d& theta_hat_max_, const Matrix3d& gamma_gain, const Vector3d& CL_contribution=Vector3d::Zero()); // PROJ
Matrix<dcomplex, -1, 1> eig(const MatrixXd& mat);

int SetGains(Matrix3d K_1_, Matrix3d K_2_, Matrix3d K_3_a_, Matrix3d K_4_, Matrix3d alpha_1_a_,
     Matrix3d alpha_2_a_, Matrix3d gamma_gain_a_, Matrix3d CL_gain_, Matrix3d adaptive_gain_,
       Matrix3d K_3_b_, Matrix3d alpha_1_b_, Matrix3d alpha_2_b_, Matrix3d gamma_gain_b_, Matrix3d gamma_gain_c_)
{
    K_1 = K_1_;    
    K_2 = K_2_;
    K_3_a = K_3_a_;
    K_3_b = K_3_b_;
    K_4 = K_4_;    

    alpha_1_a = alpha_1_a_;
    alpha_2_a = alpha_2_a_;
    gamma_gain_a = gamma_gain_a_;

    alpha_1_b = alpha_1_b_;
    alpha_2_b = alpha_2_b_;
    gamma_gain_b = gamma_gain_b_;

    gamma_gain_c = gamma_gain_c_;
    
    CL_gain = CL_gain_;
    adaptive_gain = adaptive_gain_;

    return 0;
}

int InitKalmanFilter(std::vector<Vector6d>& nu, Vector3d omega_b2i_measurement, Quaterniond q_i2b_0)
{
    nu.emplace_back( (Vector<double,6>() << omega_b2i_measurement,0,0,0).finished() ); // nu_0
    InitCovariance();   // P_0
    return 0;
}

int InitController()
{
    J = J_B;
    return 0;
}

// inputs: body rate in body frame (omega)
// pose quaternion i2b 
// nu vector from kalman filter (omega and omega_dot)
// position of sliding masses relative to CoR
// velocity of sliding masses
// desired quaternion q_i2d
telemetry_t Controller(telemetry_t t, double dt_seconds)
{
    telemetry_t controller_output = t; // maybe dont do this 
    

    
    // Extract states
    Quaterniond q_i2b = t.q_i2b; //t.get_q_i2b();
    // Vector3d omega_b2i_B_hat = nu.back().head<3>();  // first 3 elements
    // Vector3d omega_b2i_B_dot_hat = nu.back().segment(3,3); // last 3 elements 
    /*  Desired trajectories */
    // Predefined Omega and Quaternion Trajectory w.r. to Inertial Frame
    static const double b = 0.01*M_PI; 
    static const double f = 0.01;
    Vector3d omega_b2i_I = quat_rotate(q_i2b, t.omega_b2i_B); // map omega_b2i_B from body frame to inertial frame
    Vector3d omega_d2i_I = ( Vector3d() << 0,0,omega_b2i_I.z() ).finished(); // Angular rates of DF w.r. Inertial in Inertial Frame

    // Transformation Rotation of Angular Rates of the DF w.r. to Inertial from Inertial to DF
    controller_output.omega_d2i_d = quat_rotate(t.q_i2d.conjugate(), omega_d2i_I); //  quat_mult(quat_mult(quat_conj(q_i2d),[0;omega_d2i_I]),q_i2d);

    /* Trajectory Design */
    // used to be 200
    static const double T_start = 100.0f; // when it starts
    static const double t_mult = 0.5f; // change when gains get changed
    static const double T_trans = 15.0f; // half as long as it takes manuever takes to get to full pitch 
    static const double T_offset =  T_trans + T_start; 

    static const double T_begin_second_dip = T_start + 100.0f;
    static const double T_offset_2 = T_begin_second_dip + T_trans; 

    static const double rot_trans = M_PI/12.0f;
    
    Matrix3d gamma_gain;

    if (t.time*1e-3 < T_start)
    {
        gamma_gain = gamma_gain_a;
        std::cout << "Maneuver 1" << std::endl;
    }
    else if (t.time*1e-3 >= T_start && (t.time*1e-3 <  T_offset)) // in the time for the first dip
    {
        Vector3d added_desired_term; added_desired_term <<  
            0, 
            rot_trans*M_PI/(2*T_trans)*sin(M_PI/T_trans*(t.time*1e-3 - T_start)),
            0;
        controller_output.omega_d2i_d += added_desired_term;
        gamma_gain = gamma_gain_b;
        std::cout << "Begin Maneuver 2" << std::endl;
    }
    else if (t.time*1e-3 >= T_offset && t.time*1e-3 < T_begin_second_dip)  // stabilize to origin for this long
    {
        // do nothing to desired trajectory
        std::cout << "Maneuver 2" << std::endl;
        gamma_gain = gamma_gain_b;
    }
    else if (t.time*1e-3 >= T_begin_second_dip && t.time*1e-3 < T_begin_second_dip+T_trans) // in the time 
    {
        Vector3d added_desired_term;  
        added_desired_term << 0,
            -rot_trans*M_PI/(2*T_trans)*sin(M_PI/T_trans*(t.time*1e-3 - T_begin_second_dip)), 
            0;
        controller_output.omega_d2i_d += added_desired_term;
        gamma_gain = gamma_gain_c;
        std::cout << "Begin Maneuver 3" << std::endl;
    } 
    else if (t.time*1e-3 >= T_begin_second_dip+T_trans && t.time*1e-3 < T_begin_second_dip+T_start)
    {
        gamma_gain = gamma_gain_c;
        std::cout << "Maneuver 3" << std::endl;
    }
    else if (t.time*1e-3 >= T_begin_second_dip+T_start )
    {
        gamma_gain = gamma_gain_c;
        std::cout << "Test Completed - Balanced" << std::endl;
    }
    
    Vector3d omega_dot_d2i_D = (Vector3d() << 0,0,0).finished(); //0; 0*A*exp(-f*t)*(f*sin(b*t) - b*cos(b*t));0];
    Vector3d omega_dot_d2i_I = quat_rotate(q_i2b, omega_dot_d2i_D); // quat_mult(quat_mult(q_i2d,[0;omega_dot_d2i_D]),quat_conj(q_i2d));
    Vector3d omega_dot_d2i_B = quat_rotate(q_i2b.conjugate(), omega_dot_d2i_I); // quat_mult(quat_mult(quat_conj(q_i2b),[0;omega_dot_d2i_I]),q_i2b);

    /* Error signal */
    Quaterniond q_d2b = t.q_i2d.conjugate() * q_i2b; //  quat_mult(quat_conj(q_i2d),q_i2b);
    // Omega Desired w.r. Inertial in Body Fixed Frame (BFF)
    Vector3d omega_d2i_B =  quat_rotate(q_d2b.conjugate(), controller_output.omega_d2i_d); //  quat_mult(quat_mult(quat_conj(q_d2b),[0;omega_d2i_d]),q_d2b);
    Vector3d omega_b2d_B = t.omega_b2i_B - omega_d2i_B; // Omega Body w.r. Desired in BFF
    // Vector3d omega_b2d_B=  omega_b2i_B_hat - omega_d2i_B; // Omega Body w.r. Desired in BFF that has noise influece, used in control signal

    Vector3d r;
    if (t.time*1e-3 < t_mult*T_start)
    {
        r = omega_b2d_B + alpha_2_a*q_d2b.vec(); // Definition of r error signal 
    }
    else if (t.time*1e-3 >= t_mult*T_start) // in the time for the first dip
    {
        r = omega_b2d_B + alpha_2_b*q_d2b.vec(); // Definition of r error signal 
    }

    /* Update of J(t) as a function of new mass position */
    Matrix3d Jm_B_dot;
    Jm_B_dot << mm_mass_matrix(1,1)*t.r_mass.y()*t.rdot_mass.y() + mm_mass_matrix(2,2)*t.r_mass.z()*t.rdot_mass.z() , 0 , 0,
            0, mm_mass_matrix(0,0)*t.r_mass.x()*t.rdot_mass.x() + mm_mass_matrix(2,2)*t.r_mass.z()*t.rdot_mass.z(), 0, 
            0, 0, mm_mass_matrix(0,0)*t.r_mass.x()*t.rdot_mass.x() + mm_mass_matrix(1,1)*t.r_mass.y()*t.rdot_mass.y(); 
    Jm_B_dot *= 2; 

    /* Compute basis function Phi */
    Vector3d g_B = quat_rotate(q_i2b.conjugate(), g_I);  //quat_mult(quat_mult(quat_conj(q_i2b),[0;g_I]),q_i2b);
    Matrix3d Phi = -M*skew(g_B); // Phi definition as in ref[DOI: 10.2514/1.60380]

    /* Control Torque as Designed by Lyapunov Analysis */
    Matrix3d Proj_operator = Matrix3d::Identity() - ((g_B*g_B.transpose())) / g_B.squaredNorm();

    if (t.time*1e-3 < t_mult*T_start)
    {
    controller_output.u_com = Proj_operator * (
            -K_1*Jm_B_dot*(0.5*r - t.omega_b2i_B) + 
            K_2*skew(t.omega_b2i_B)*J*t.omega_b2i_B 
            - adaptive_gain*Phi*t.theta_hat 
            - K_3_a*r )// - diag((theta_hat)*(theta_hat))*r
            + Proj_operator*J*(omega_dot_d2i_B 
            + K_4*skew(omega_d2i_B)*omega_b2d_B 
            - 0.5*alpha_1_a*(skew(q_d2b.vec()) + q_d2b.w()*Matrix3d::Identity())*omega_b2d_B); // desired torque 
    }
    else if (t.time*1e-3 >= t_mult*T_start && t.time*1e-3 < T_start) // half way into first first maneuver
    {
        controller_output.u_com = Proj_operator * (
            -K_1*Jm_B_dot*(0.5*r - t.omega_b2i_B) + 
            K_2*skew(t.omega_b2i_B)*J*t.omega_b2i_B 
            - adaptive_gain*Phi*t.theta_hat 
            - K_3_b*r )// - diag((theta_hat)*(theta_hat))*r
            + Proj_operator*J*(omega_dot_d2i_B 
           + K_4*skew(omega_d2i_B)*omega_b2d_B 
            - 0.5*alpha_1_b*(skew(q_d2b.vec()) + q_d2b.w()*Matrix3d::Identity())*omega_b2d_B); // desired torque 
    }
    else if (t.time*1e-3 >= T_start && t.time*1e-3 < T_begin_second_dip - t_mult*T_start) // second maneuver begins
    {
        controller_output.u_com = Proj_operator * (
            -K_1*Jm_B_dot*(0.5*r - t.omega_b2i_B) + 
            K_2*skew(t.omega_b2i_B)*J*t.omega_b2i_B 
            - adaptive_gain*Phi*t.theta_hat 
            - K_3_b*r )// - diag((theta_hat)*(theta_hat))*r
            + Proj_operator*J*(omega_dot_d2i_B 
           + K_4*skew(omega_d2i_B)*omega_b2d_B 
            - 0.5*alpha_1_b*(skew(q_d2b.vec()) + q_d2b.w()*Matrix3d::Identity())*omega_b2d_B); // desired torque 
    }
    else if (t.time*1e-3 >= T_begin_second_dip - t_mult*T_start) // half way into second maneuver
    {
        controller_output.u_com = Proj_operator * (
            -K_1*Jm_B_dot*(0.5*r - t.omega_b2i_B) + 
            K_2*skew(t.omega_b2i_B)*J*t.omega_b2i_B 
            - adaptive_gain*Phi*t.theta_hat 
            - K_3_b*r )// - diag((theta_hat)*(theta_hat))*r
            + Proj_operator*J*(omega_dot_d2i_B 
           + K_4*skew(omega_d2i_B)*omega_b2d_B 
            - 0.5*alpha_1_b*(skew(q_d2b.vec()) + q_d2b.w()*Matrix3d::Identity())*omega_b2d_B); // desired torque 
    }
    //else if (t.time*1e-3 >= T_begin_second_dip+T_trans)
    //{
        //controller_output.u_com = Phi*t.theta_hat;
    //}

    /* Concurrent learning data selection algorithm */
    // initialization done once
    static int p_CL_idx = 0;
    static bool cyclic_started = false;
    static Vector<double, 9> Phi_previous = Phi.reshaped(9, 1);
    static int current_index_for_cyclic_stack = 0;

    // extract states
    Vector3d omega_hat =  t.omega_b2i_B; // t.nu.segment(0,3);
    Vector3d omega_dot_hat = t.nu.segment(3,3);

    // append Phi to a new stack to check if rank changes
    // MatrixXd Phi_new_stack = Phi_stack; // [Phi_stack, Phi(:)];
    // Phi_new_stack.conservativeResize(Eigen::NoChange, Phi_new_stack.cols() + 1);
    // Phi_new_stack.col(Phi_new_stack.cols() - 1) = Phi.reshaped(9,1); // added new col, set new col equal to Phi(:)
    
    // Point selection criteria
    Vector3d Phi_col = Phi.col(2);
    Vector3d Phi_col_prev = Phi_previous.reshaped(3,3).col(2);
    std::cout << "delta cl: " << (Phi_col - Phi_col_prev).squaredNorm() / Phi_col.norm() << std::endl;
    // if ( (Phi.reshaped(9,1)- Phi_previous).squaredNorm()  >= CL_point_accept_epsilon)  //|| (rank(Phi_new_stack) > rank(Phi_stack)) )
    if ( (Phi_col - Phi_col_prev).squaredNorm() / Phi_col.norm() > CL_point_accept_epsilon )
    {
        Vector3d u_actual = -g_B.cross(mm_mass_matrix*(t.r_mass)); // current actual control torques according to motor feedback
        Vector3d Tau_j = J*omega_dot_hat + Jm_B_dot*omega_hat + omega_hat.cross(J*omega_hat) - controller_output.u_actual ;

        if (p_CL_idx < p_bar && !cyclic_started) // record more data until p_bar points
        {
            Phi_stack.push_back(Phi.reshaped(9,1)); 
            Tau_stack.push_back(Tau_j);
            Phi_previous = Phi.reshaped(9,1);
            current_index_for_cyclic_stack = p_CL_idx;
        }
        else
        {
            cyclic_started = true;
            // cyclic history stack
            Phi_stack.at(current_index_for_cyclic_stack) = Phi.reshaped(9,1); // Insert the new element at the current index
            Tau_stack.at(current_index_for_cyclic_stack) = Tau_j; // Insert the new element at the current index
            Phi_previous = Phi.reshaped(9,1);

            current_index_for_cyclic_stack++; 
            if (current_index_for_cyclic_stack == p_bar) 
                current_index_for_cyclic_stack  = 0;
        }
        p_CL_idx++;
        total_pts++;
        std::cout << "Point added! " << total_pts << std::endl;
        // point_added = [point_added t]; %  record that we stored a point
    }

    /* Concurrent learning error signal */
    Vector3d concurrent_learning_Tau_ext = Vector3d::Zero();
    Matrix3d P = Matrix3d::Zero();
    for (int j=0; j < Phi_stack.size(); j++)
    {
        Vector3d Tau_j = Tau_stack.at(j); // jth delta from storage
        Matrix3d Phi_j = Phi_stack.at(j).reshaped(3,3); // jth phi matrix from storage
        Vector3d epsilon_Tau_ext = Tau_j - Phi_j*t.theta_hat ;

        concurrent_learning_Tau_ext += Phi_j.transpose()*epsilon_Tau_ext;  
        P += Phi_j.transpose()*Phi_j;
    }
    
    std::cout << "Test Time: " << t.time*1e-3 << std::endl;
    // std::cout << "cl_tau_ext: " << concurrent_learning_Tau_ext.transpose() << std::endl;
    std::cout << "CL Points Total: " << total_pts << std::endl;

    // std::cout << "eig P: " << eig(P) << std::endl;
    // std::cout << "P rank: " << rank(P) << std::endl;

    // std::cout << "CL Update law contribution: " << (CL_on*CL_gain*gamma_gain*concurrent_learning_Tau_ext).transpose()  << std::endl; 
    // std::cout << "Standard update law contribution: " << (gamma_gain * (Phi.transpose()*r)).transpose() << std::endl;

    //std::cout << "mat_m_inv: " << mm_mass_matrix.inverse() << std::endl;
    //std::cout << "alpha_2 Error term: " << (Proj_operator *(alpha_2*q_d2b.vec())).transpose() << std::endl;
    //std::cout << "K_1 term: " << (Proj_operator *(-K_1*Jm_B_dot*(0.5*r - t.omega_b2i_B))).transpose() << std::endl;
    //std::cout << "K_2 Derivative term: " << (Proj_operator *(K_2*skew(t.omega_b2i_B)*J*t.omega_b2i_B)).transpose() << std::endl;
    // std::cout << "K_3 Error term: " << (Proj_operator *(-K_3_b*r)).transpose() << std::endl;
    //std::cout << "K_4 Derivative term: " << (Proj_operator *( K_4*skew(omega_d2i_B)*omega_b2d_B)).transpose() << std::endl;
    //std::cout << "Alpha_1 term: " << (Proj_operator *(alpha_1*(skew(q_d2b.vec()) + q_d2b.w()*Matrix3d::Identity())*omega_b2d_B)).transpose() << std::endl;
    // std::cout << "Adaptive term: " << (Proj_operator *(adaptive_gain*Phi*t.theta_hat)).transpose() << std::endl;
   
    /* Map control Torque to mass positions */ //Transformation of u_com to Commanded Positions as in ref[DOI: 10.2514/1.60380]
    if (t.time*1e-3 < T_begin_second_dip+T_start)
    {
        controller_output.r_mass_commanded = mm_mass_matrix.inverse() * (g_B.cross(controller_output.u_com) / g_B.squaredNorm() ); // desired commanded mass positions
    }
    else if (t.time*1e-3 >= T_begin_second_dip+T_start)
    {
        controller_output.r_mass_commanded = -M * mm_mass_matrix.inverse() * t.theta_hat;
    } 
    /* Make sure r_mass_commanded is within saturation limits (makes sense to apply here before stepper mapping) */
    controller_output.r_mass_commanded = SaturationLimit(controller_output.r_mass_commanded);

    // at this point, r_mass_commanded is relative to the middle zero position of the sliding masses (not the zero limit switch position)

    
    /* Dynamics for desired frame */
    Quaterniond omega_d2i_D_quaternion; 
    omega_d2i_D_quaternion.w() = 0; 
    omega_d2i_D_quaternion.vec() = controller_output.omega_d2i_d; // (Vector3d() << 0, 0, 5).finished();
    

    Quaterniond q_i2d_dot = t.q_i2d * omega_d2i_D_quaternion;  // quat_mult(q_i2d,[0;omega_d2i_D]); % q_dot of DF w.r. to Inertial Frame
    q_i2d_dot.coeffs() *= 0.5; // dont forget to scale by 0.5 since we cant do that above due to * operator override
    controller_output.q_i2d.coeffs() += q_i2d_dot.coeffs()*dt_seconds; 
    controller_output.q_i2d.normalize();


    /* Update law */
    Vector3d theta_hat_dot;
    
    theta_hat_dot = gamma_gain * (Phi.transpose()*r); //  Adaptive update law
    // if (t.time*1e-3 > T_start) {
        Vector3d CL_contribution = CL_on*gamma_gain*CL_gain*concurrent_learning_Tau_ext;
        theta_hat_dot += CL_contribution;
    // }

    controller_output.theta_hat += theta_hat_dot*dt_seconds; 

    /* Compute our actual control torque at the moment for logging */
    controller_output.u_actual = -g_B.cross(mm_mass_matrix*(t.r_mass));
    return controller_output;
}

// telemetry_t PD_Controller(telemetry_t t, double dt_seconds)
// {
    // telemetry_t controller_output = t; // maybe dont do this 

    // Eigen::Quaterniond q_error = Quaterniond(1,0,0,0) * t.q_b2i.conjugate();
    // q_error.normalize();
    // if (q_error.w() < 0.0)
        // q_error.coeffs() *= -1.0;
// 
        // Quaterniond q_i2b = t.q_i2b; //t.get_q_i2b();
        // Vector3d g_B = quat_rotate(q_i2b.conjugate(), g_I);  //quat_mult(quat_mult(quat_conj(q_i2b),[0;g_I]),q_i2b);
    // 
    // Eigen::AngleAxisd aa(q_error);
    // Eigen::Vector3d rot_axis = aa.axis();   // unit vector
    // double rot_angle = aa.angle();          // in radians
// 
    // Matrix3d Proj_operator =  Matrix3d::Identity() - ((g_B*g_B.transpose()) ) / g_B.squaredNorm();


    // controller_output.u_com = K_1 * rot_angle * rot_axis  - K_2 *Proj_operator* t.omega_b2i_B; // proportional , derivative body frame

 
    // controller_output.r_mass_commanded = mm_mass_matrix.inverse() * (g_B.cross(controller_output.u_com) / g_B.squaredNorm() ); // desired commanded mass positions
    // // at this point, r_mass_commanded is relative to the middle zero position of the sliding masses (not the zero limit switch position)
    // /* Make sure r_mass_commanded is within saturation limits (makes sense to apply here before stepper mapping) */
    // controller_output.r_mass_commanded = SaturationLimit(controller_output.r_mass_commanded);
    // controller_output.u_actual = -mm_mass_matrix * g_B.cross(t.r_mass);

    // return controller_output;
//}
/* Apply a saturation limit to the mass position based on mechanical limitations of the vehicle */
Vector3d SaturationLimit(Vector3d r_com)
{
    // there is probably a shorter easier way of doing this but it works fine and is easy to read
    Vector3d saturated_r_com = r_com;

    // x (symmetric)
    if (saturated_r_com.x() > m_x_max_pos_meters) {
        saturated_r_com.x() = m_x_max_pos_meters;
    } else if (saturated_r_com.x() < -m_x_max_pos_meters) {
        saturated_r_com.x() = -m_x_max_pos_meters;
    }
    
    // y (symmetric)
    if (saturated_r_com.y() > m_y_max_pos_meters) {
        saturated_r_com.y() = m_y_max_pos_meters;
    } else if (saturated_r_com.y() < -m_y_max_pos_meters) {
        saturated_r_com.y() = -m_y_max_pos_meters;
    }

    // z
    if (saturated_r_com.z() > m_z_max_pos_meters) {
        saturated_r_com.z() = m_z_max_pos_meters;
    } else if (saturated_r_com.z() < -m_z_max_pos_meters) {
        saturated_r_com.z() = -m_z_max_pos_meters;
    }

    return saturated_r_com;
}

Vector3d quat_rotate(const Quaterniond& q, const Vector3d& v)
{
    Quaterniond v_quat(0, v.x(), v.y(), v.z());  // Pure quaternion [0; v]
    Quaterniond rotated = q * v_quat * q.conjugate();  // q * [0; v] * q^*
    return rotated.vec();  // Extract the vector part (x, y, z)
}

Matrix3d skew(const Vector3d& v)
{
    Matrix3d skew_sym_matrix;
    skew_sym_matrix <<  0,   -v.z(),  v.y(),
             v.z(),   0,  -v.x(),
            -v.y(),  v.x(),   0;
    return skew_sym_matrix;
}



Quaterniond quat_mult(const Quaterniond& q, const Quaterniond& p)
{
    return q * p;  
}

int rank(MatrixXd mat)
{
    JacobiSVD<MatrixXd> svd(mat, ComputeThinU | ComputeThinV);
    double tol = 1e-6 * svd.singularValues().array().abs()(0);  // Tolerance based on largest singular value
    int rank_ = (svd.singularValues().array() > tol).count();
    return rank_;
}

Matrix<dcomplex, -1, 1> eig(const MatrixXd& mat)
{
    EigenSolver<MatrixXd> es(mat);

    // std::cout << "Eigenvalues:\n" << es.eigenvalues() << "\n";
    // std::cout << "Eigenvectors:\n" << es.eigenvectors() << "\n";
    auto ev = es.eigenvalues();
    return ev;
    // return es.eigenvalues();
}

// Projection operator 
Vector3d projected_update(const Vector3d& theta_hat, const Matrix3d& Phi, Vector3d r,
    const Vector3d& theta_hat_min_, const Vector3d& theta_hat_max_, const Matrix3d& gamma_gain, const Vector3d& CL_contribution)
{   
    Vector3d theta_hat_dot = gamma_gain * Phi.transpose() * r + CL_contribution;

    for (int i = 0; i < theta_hat.size(); ++i)
    {
        // If at lower bound and trying to decrease, block update
        if (theta_hat(i) <= theta_hat_min(i) && theta_hat_dot(i) < 0)
        {
            theta_hat_dot(i) = 0;
        }
        // If at upper bound and trying to increase, block update
        else if (theta_hat(i) >= theta_hat_max(i) && theta_hat_dot(i) > 0)
        {
            theta_hat_dot(i) = 0;
        }
    }

    return theta_hat_dot;
}

