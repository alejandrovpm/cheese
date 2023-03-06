// FT DRIVER BASED ON STEWART PLATFORM

#ifndef OROCOS_FT_DRIVER_HPP
#define OROCOS_FT_DRIVER_HPP

#include <vector>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <Eigen/Dense>

class FTSensor {
private:
    double m_atol;
    
    // Dimensions
    std::vector<KDL::Vector> m_joints_P_in_fixed_platform;
    std::vector<KDL::Vector> m_joints_P_in_moving_platform;
    std::vector<double> m_lengths_between_joints;
    std::vector<KDL::Vector> m_actuation_unit_vectors;
    KDL::Frame m_F_p;

    Eigen::Matrix<double,6,1> m_sensed_forces;
    std::vector<double> m_offset; 

    void mvToN(std::vector<double> &_sensed_mv);

    void jacobian(Eigen::Matrix<double,6,6> &_J, const std::vector<KDL::Vector> &_actuation_unit_vectors);
    void stewartIK(std::vector<double> &_lengths_between_joints, std::vector<KDL::Vector> &_actuation_unit_vectors, const KDL::Frame &_F_p);

public:
    FTSensor();    
    std::vector<double> GetWrench(std::vector<double> &_sensed_mv);
    void SetOffset(std::vector<double> &_sensed_mv);
 
};

#endif