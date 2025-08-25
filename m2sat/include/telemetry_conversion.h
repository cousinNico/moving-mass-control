#pragma once

#include "telemetry.h"
#include "telemetry.pb.h"


// Helper function to convert repeated field to Eigen::Vector3d
inline Eigen::Vector3d fromProtoVector3d(const ::google::protobuf::RepeatedField<double>& proto_vec) {
    return (proto_vec.size() == 3) ? Eigen::Vector3d(proto_vec[0], proto_vec[1], proto_vec[2]) 
                                   : Eigen::Vector3d::Zero();
}

// Helper function to convert Eigen::Vector3d to repeated field
inline void toProtoVector3d(const Eigen::Vector3d& vec, ::google::protobuf::RepeatedField<double>* proto_vec) {
    proto_vec->Add(vec.x());
    proto_vec->Add(vec.y());
    proto_vec->Add(vec.z());
}

// Helper function to convert repeated field to Eigen::Quaterniond
inline Eigen::Quaterniond fromProtoQuaterniond(const ::google::protobuf::RepeatedField<double>& proto_quat) {
    return (proto_quat.size() == 4) ? Eigen::Quaterniond(proto_quat[0], proto_quat[1], proto_quat[2], proto_quat[3])
                                    : Eigen::Quaterniond::Identity();
}

// Helper function to convert Eigen::Quaterniond to repeated field
inline void toProtoQuaterniond(const Eigen::Quaterniond& quat, ::google::protobuf::RepeatedField<double>* proto_quat) {
    proto_quat->Add(quat.w());
    proto_quat->Add(quat.x());
    proto_quat->Add(quat.y());
    proto_quat->Add(quat.z());
}


// Conversion from Protobuf to custom struct
inline telemetry_t fromProto(const TelemetryMessage& proto) {
    Vector3d nu_top = fromProtoVector3d(proto.nu_top());
    Vector3d nu_bot = fromProtoVector3d(proto.nu_bottom());
    Vector6d nu; nu << nu_top, nu_bot;

    return 
    {

        proto.time(),
        fromProtoQuaterniond(proto.q_b2i()),
        fromProtoQuaterniond(proto.q_i2d()),
        fromProtoVector3d(proto.omega_b2i()),
        fromProtoVector3d(proto.r_mass()),
        fromProtoVector3d(proto.rdot_mass()),
        fromProtoVector3d(proto.r_mass_commanded()),
        fromProtoVector3d(proto.u_com()),
        fromProtoVector3d(proto.u_actual()),
        nu,
        fromProtoVector3d(proto.theta_hat()),
        fromProtoVector3d(proto.omega_d2i_d())
        
    };
}

// Conversion from custom struct to Protobuf
inline TelemetryMessage toProto(const telemetry_t& t) {
    TelemetryMessage proto;
    proto.set_time(t.time);
    toProtoQuaterniond(t.q_i2b, proto.mutable_q_b2i());
    toProtoQuaterniond(t.q_i2d, proto.mutable_q_i2d());
    toProtoVector3d(t.omega_b2i_B, proto.mutable_omega_b2i());
    toProtoVector3d(t.r_mass, proto.mutable_r_mass());
    toProtoVector3d(t.rdot_mass, proto.mutable_rdot_mass());
    toProtoVector3d(t.r_mass_commanded, proto.mutable_r_mass_commanded());
    toProtoVector3d(t.u_com, proto.mutable_u_com());
    toProtoVector3d(t.u_actual, proto.mutable_u_actual());
    toProtoVector3d(t.nu.segment(0,3), proto.mutable_nu_top());
    toProtoVector3d(t.nu.segment(3,3), proto.mutable_nu_bottom());
    toProtoVector3d(t.theta_hat, proto.mutable_theta_hat());
    toProtoVector3d(t.omega_d2i_d, proto.mutable_omega_d2i_d());

    return proto;
}