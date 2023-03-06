#include "microcontroller_driver.hpp"
#include <rtt/Component.hpp>
#include <iostream>
#include <unistd.h>

MicrocontrollerDriver::MicrocontrollerDriver(std::string const& name) : 
TaskContext(name),
m_port_name("/dev/ttyACM0"),
m_baudrate(115200),
m_timeout(200),
m_v_fsr_value(6,0.0),
m_v_mems_value(8,0.0),
m_time_zero_frs(0.0),
m_time_zero_mems(0.0),
m_ft_sensor(),
m_wrench(6,0.0)
{
  
  m_port = new serial::Serial(m_port_name, m_baudrate, serial::Timeout::simpleTimeout(m_timeout));

  // // Ports
  this->ports()->addPort("outputport_fsr",m_outputport_fsr).doc("Output port with the current force");
  this->ports()->addPort("outputport_mems",m_outputport_mems).doc("Output port with the current gyro imu data");
  this->ports()->addPort("outputport_wrench",m_outputport_wrench).doc("Output port with the wrench");

  std::cout << "MicrocontrollerDriver constructed !" <<std::endl;

}

bool MicrocontrollerDriver::configureHook(){
  uint8_t key;
  size_t req_size;

  key = '0';
  req_size = m_port -> write(&key,1);
  req_size = m_port -> read((uint8_t*)&m_fsr_value,sizeof(m_fsr_value));

  key = '1';
  req_size = m_port -> write(&key,1);
  req_size = m_port -> read((uint8_t*)&m_mems_value,sizeof(m_mems_value));

  // m_v_fsr_value[0] = (double)m_fsr_value.time_stamp/1000.0;
  m_v_fsr_value[0] = (double)m_fsr_value.fsr_value_mv[0];
  m_v_fsr_value[1] = (double)m_fsr_value.fsr_value_mv[1];
  m_v_fsr_value[2] = (double)m_fsr_value.fsr_value_mv[2];
  m_v_fsr_value[3] = (double)m_fsr_value.fsr_value_mv[3];
  m_v_fsr_value[4] = (double)m_fsr_value.fsr_value_mv[4];
  m_v_fsr_value[5] = (double)m_fsr_value.fsr_value_mv[5];

  m_v_mems_value[0] = (double)m_mems_value.time_stamp/1000.0;
  m_v_mems_value[1] = (double)m_mems_value.temperature_degC;
  m_v_mems_value[2] = (double)m_mems_value.angular_rate_mdps[0];
  m_v_mems_value[3] = (double)m_mems_value.angular_rate_mdps[1];
  m_v_mems_value[4] = (double)m_mems_value.angular_rate_mdps[2];
  m_v_mems_value[5] = (double)m_mems_value.acceleration_mg[0];
  m_v_mems_value[6] = (double)m_mems_value.acceleration_mg[1];
  m_v_mems_value[7] = (double)m_mems_value.acceleration_mg[2];

  m_outputport_fsr.write(m_v_fsr_value);
  m_outputport_mems.write(m_v_mems_value);

  Tare();

  m_wrench = m_ft_sensor.GetWrench(m_v_fsr_value);
  m_outputport_wrench.write(m_wrench);
  std::cout << "MicrocontrollerDriver configured !" <<std::endl;
  return true;
}

bool MicrocontrollerDriver::startHook(){
  uint8_t key;
  size_t req_size;

  key = '0';
  req_size = m_port -> write(&key,1);
  req_size = m_port -> read((uint8_t*)&m_fsr_value,sizeof(m_fsr_value));

  key = '1';
  req_size = m_port -> write(&key,1);
  req_size = m_port -> read((uint8_t*)&m_mems_value,sizeof(m_mems_value));

  m_time_zero_frs = (double)m_fsr_value.time_stamp/1000.0;
  m_time_zero_mems = (double)m_mems_value.time_stamp/1000.0;

  std::cout << "MicrocontrollerDriver started !" <<std::endl;
  return true;
}

void MicrocontrollerDriver::updateHook(){

 
  uint8_t key;
  size_t req_size;

  key = '0';
  req_size = m_port -> write(&key,1);
  req_size = m_port -> read((uint8_t*)&m_fsr_value,sizeof(m_fsr_value));

  key = '1';
  req_size = m_port -> write(&key,1);
  req_size = m_port -> read((uint8_t*)&m_mems_value,sizeof(m_mems_value));

  // m_v_fsr_value[0] = (double)m_fsr_value.time_stamp/1000.0 - m_time_zero_frs;
  m_v_fsr_value[0] = (double)m_fsr_value.fsr_value_mv[0];
  m_v_fsr_value[1] = (double)m_fsr_value.fsr_value_mv[1];
  m_v_fsr_value[2] = (double)m_fsr_value.fsr_value_mv[2];
  m_v_fsr_value[3] = (double)m_fsr_value.fsr_value_mv[3];
  m_v_fsr_value[4] = (double)m_fsr_value.fsr_value_mv[4];
  m_v_fsr_value[5] = (double)m_fsr_value.fsr_value_mv[5];


  m_v_mems_value[0] = (double)m_mems_value.time_stamp/1000.0 - m_time_zero_mems;
  m_v_mems_value[1] = (double)m_mems_value.temperature_degC;
  m_v_mems_value[2] = (double)m_mems_value.angular_rate_mdps[0];
  m_v_mems_value[3] = (double)m_mems_value.angular_rate_mdps[1];
  m_v_mems_value[4] = (double)m_mems_value.angular_rate_mdps[2];
  m_v_mems_value[5] = (double)m_mems_value.acceleration_mg[0];
  m_v_mems_value[6] = (double)m_mems_value.acceleration_mg[1];
  m_v_mems_value[7] = (double)m_mems_value.acceleration_mg[2];

  m_outputport_fsr.write(m_v_fsr_value);
  m_outputport_mems.write(m_v_mems_value);
  m_wrench = m_ft_sensor.GetWrench(m_v_fsr_value);

  m_outputport_wrench.write(m_wrench);

}

void MicrocontrollerDriver::stopHook() {
  std::cout << "MicrocontrollerDriver executes stopping !" <<std::endl;
  // for (auto &motor:m_motors){
  //   motor.MoveForward(0);
  // }
  m_port->close();
}

void MicrocontrollerDriver::cleanupHook() {
  std::cout << "MicrocontrollerDriver cleaning up !" <<std::endl;
  // for (auto &motor:m_motors){
  //   motor.MoveForward(0);
  // }
}

void MicrocontrollerDriver::Tare(){
  m_ft_sensor.SetOffset(m_v_fsr_value);
}


/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(MicrocontrollerDriver)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(MicrocontrollerDriver)
