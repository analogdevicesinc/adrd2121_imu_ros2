/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#ifndef ADI_IMU_BUF_ROS2
#define ADI_IMU_BUF_ROS2

//ROS2 Headers 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "std_srvs/srv/trigger.hpp"

//IMU Driver ROS2 headers
#include "adi_imu_ros2.hpp"
#include "imu_buf_ros2.hpp"
#include "adrd2121_imu_ros2_common.hpp"

//IMU Driver ROS2 custom srv header
#include "adrd2121_imu/srv/imu_glob_cmd.hpp"

//C IMU Driver headers
#include "adi_imu_driver.h"
#include "imu_spi_buffer.h"

//CPP headers
#include <memory>
#include <iostream>
#include <string>

//Namespace definitions 
using std::placeholders::_1;
using std::placeholders::_2;

//! \class AdiImuBufRos2
//! \brief Class for ADI IMU + ADRD2121
class AdiImuBufRos2
{
public:
  //! \brief Constructor for AdiImuBufRos2 
  AdiImuBufRos2(const rclcpp::Node::SharedPtr& node);

  //! \brief Destructor for AdiImuBufRos2
  ~AdiImuBufRos2();

  //! \brief Loads ROS-related, ADI IMU (AdiImuRos2), and
  //! ADRD2121 (ImuBufRos2) parameters
  void initParams(void);

  //! \brief Initializes the hardware
  //! Initializes the (1) communication with the board (i.e via USB), (2) ADRD2121,
  //! ADI IMU, and (4) advertises ROS Publishers and Service Servers. 
  bool init(void);
  
  //! \brief Configures the hardware based on the loaded parameters
  //! Configure the (1) ADRD2121, and (2) ADI IMU
  //! Also, sets the appropriate ROS Loop rate
  //! 
  //! \return Boolean if successful (true) or not (false)
  bool config(void);

  //! \brief Acquire publisher rate and advertise wall timer for looping 
  //!
  void readPubData(void);

  //! \brief Callback being called by wall timer to read and publish IMU data
  //!
  void dataReadAndPubCB(void);

  //! \brief Return ROS Loop Rate set during config
  //!
  //! \return ROS Loop Rate in Hz
  double getRosLoopRateHz(void);

  //! \brief Service Server callback for imu_glob_cmd_service_
  //!
  //! \param req, res Request and Response to the Service
  //!
  //! \return Boolean if successful (true) or not (false)
  bool triggerImuGlobCmd(adrd2121_imu::srv::ImuGlobCmd::Request::SharedPtr req,
    adrd2121_imu::srv::ImuGlobCmd::Response::SharedPtr res);

  //! \brief ROS2 Service Server to send commands to GLOB_CMD register of ADI IMU
  rclcpp::Service<adrd2121_imu::srv::ImuGlobCmd>::SharedPtr p_imu_glob_cmd_service_;

  //! \brief Determine what operation mode will the node run on.
  //! \brief 1: STREAM
  //! \brief 2: CONFIG
  uint8_t mode_of_operation_;

  //! \brief Checks if at least the HW is initialized. 
  bool b_getHWInitialized_;
  
  //! \brief Boolean for checking if ReadPubData call is successful 
  bool b_readPubDataSuccess; 

private:
  //! \brief Timer object meant to hold an instance of the wall timer
  rclcpp::TimerBase::SharedPtr timer_;

  //! \brief Local node meant to catch node from main function to be used for ROS2 APIs
  rclcpp::Node::SharedPtr node_;
  
  //! \brief Parameter Callback handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback;
  
  //! \brief Callback function that verifies if the inputted value for the parameter is 
  //! \brief within range or not. 
  rcl_interfaces::msg::SetParametersResult paramCbFunction(
    const std::vector<rclcpp::Parameter> &param);
  
  //! \brief Pointer to an AdiImuRos2 object
  AdiImuRos2* p_adi_imu_ros2_obj_;

  //! \brief Pointer to an ImuBufRos2 object
  ImuBufRos2* p_imu_buf_ros2_obj_;

  //! \brief ROS loop rate in Hz 
  double ros_loop_rate_hz_;

  //! \brief ROS Message type for publisher imu_pub_std_/imu_pub_adi_(ImuBufRos2) 
  msg_type_e msg_type_;

  //! \brief ROS Message type for publisher imu_pub_std_/imu_pub_adi_(ImuBufRos2) 
  std::string frame_name_;

  //! \brief Main struct for device
  adi_imu_Device_t imu_dev_;
};

#endif //ADI_IMU_BUF_ROS2
