#ifndef OROCOS_MICROCONTROLLER_DRIVER_HPP
#define OROCOS_MICROCONTROLLER_DRIVER_HPP

#include <rtt/RTT.hpp>
// #include "roboclaw_interface.hpp"
// #include "roboclaw.h"
#include <serial/serial.h>
#include "ft_sensor_stewart.hpp"


class MicrocontrollerDriver : public RTT::TaskContext{
  private:

    struct fsr_stream {
      uint32_t time_stamp;
      uint32_t fsr_value_mv[6];
    } m_fsr_value;

    struct mems_stream{
      uint32_t time_stamp;
      float temperature_degC;
      float angular_rate_mdps[3];
      float acceleration_mg[3];
    } m_mems_value;

    serial::Serial *m_port;

    std::string m_port_name; 
    uint32_t m_baudrate; 
    uint32_t m_timeout;

    std::vector<double> m_v_fsr_value;
    std::vector<double> m_v_mems_value;

    double m_time_zero_frs;
    double m_time_zero_mems;

    FTSensor m_ft_sensor;
    std::vector<double> m_wrench;

    // Ports
    RTT::OutputPort<std::vector<double>> m_outputport_fsr;
    RTT::OutputPort<std::vector<double>> m_outputport_mems;

    RTT::OutputPort<std::vector<double>> m_outputport_wrench;


  public:
    MicrocontrollerDriver(std::string const& name);
    ~MicrocontrollerDriver(){delete m_port;};
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void Tare();
};
#endif
