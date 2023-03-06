// FT DRIVER BASED ON STEWART PLATFORM

#include "ft_sensor_stewart.hpp"
// #include <iostream>
// #include <sstream>


FTSensor::FTSensor():

m_atol{0.000001},
m_joints_P_in_fixed_platform(6,KDL::Vector()),
m_joints_P_in_moving_platform(6,KDL::Vector()),
m_lengths_between_joints(6,0.0),
m_actuation_unit_vectors(6,KDL::Vector()),
m_F_p{KDL::Vector{0.0,0.0,0.05}},
m_sensed_forces(6),
m_offset(6,0.0)

{
std::cout << "Kinematics constructor!" << std::endl;
    
    // Stewart platform Base close mp small
    double r1{0.100/2};
    double r2{0.300/2};
    std::vector<double> n{ 0.0, 0.0, -120.0, -120.0, 120.0, 120.0};
    std::vector<double> o{-60.0, -60.0, -180.0, 180.0, 60.0, 60.0};

    KDL::Vector r1_vec{0, -r1, 0};
    KDL::Vector r2_vec{0, -r2, 0};

    for (size_t i{0}; i<6; i++){
        m_joints_P_in_fixed_platform[i] = KDL::Rotation::RotZ(n[i]*M_PI/180)*r1_vec;
        m_joints_P_in_moving_platform[i] = KDL::Rotation::RotZ(o[i]*M_PI/180)*r2_vec;
    }

    stewartIK(m_lengths_between_joints, m_actuation_unit_vectors, m_F_p);

}


void FTSensor::mvToN(std::vector<double> &_sensed_mv){
    // m_sensed_forces = _sensed_mv;
    // for (size_t i{0}; i<6; i++){
    //     m_sensed_forces(i) = _sensed_mv[i];
    // }
    m_sensed_forces(0) = -(_sensed_mv[0] - m_offset[0]);
    m_sensed_forces(1) = -(_sensed_mv[1] - m_offset[1]);
    m_sensed_forces(2) = -(_sensed_mv[2] - m_offset[2]);
    m_sensed_forces(3) = -(_sensed_mv[3] - m_offset[3]);
    m_sensed_forces(4) = -(_sensed_mv[4] - m_offset[4]);
    m_sensed_forces(5) = -(_sensed_mv[5] - m_offset[5]);
}


void FTSensor::jacobian(Eigen::Matrix<double,6,6> &_J, const std::vector<KDL::Vector> &_actuation_unit_vectors){

    KDL::Vector lastcols;
    for(size_t i{0}; i<6; i++){
        lastcols = m_F_p.M*m_joints_P_in_moving_platform[i]* _actuation_unit_vectors[i];
        _J(i,0) =  _actuation_unit_vectors[i][0],
        _J(i,1) =  _actuation_unit_vectors[i][1],
        _J(i,2) =  _actuation_unit_vectors[i][2],
        _J(i,3) =  lastcols[0],
        _J(i,4) =  lastcols[1],
        _J(i,5) =  lastcols[2];
    }

}

void FTSensor::stewartIK(std::vector<double> &_lengths_between_joints, std::vector<KDL::Vector> &_actuation_unit_vectors, const KDL::Frame &_F_p){
    KDL::Vector di;
    for (size_t i{0}; i < 6; i++){
        di = _F_p.p + _F_p.M*m_joints_P_in_moving_platform[i] - m_joints_P_in_fixed_platform[i];
        _lengths_between_joints[i] = di.Norm();
        _actuation_unit_vectors[i] = di/_lengths_between_joints[i];
    }
}

std::vector<double> FTSensor::GetWrench(std::vector<double> &_sensed_mv){
    
    mvToN(_sensed_mv);

    Eigen::Matrix<double,6,6> J(6,6);
    jacobian(J,m_actuation_unit_vectors);

    Eigen::Matrix<double,6,1> wrench = J.transpose() * m_sensed_forces;

    return std::vector<double>{wrench(0),wrench(1),wrench(2),wrench(3),wrench(4),wrench(5)};
}

void FTSensor::SetOffset(std::vector<double> &_sensed_mv){
    m_offset = _sensed_mv;
}
