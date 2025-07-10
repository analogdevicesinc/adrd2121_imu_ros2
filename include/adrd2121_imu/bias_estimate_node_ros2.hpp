/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#ifndef BIAS_ESTIMATE_ROS2
#define BIAS_ESTIMATE_ROS2

//ROS2 Header
#include "rclcpp/rclcpp.hpp"

//IMU ROS2 Driver Headers
#include "imu_state_checker_ros2.hpp"
#include "adrd2121_imu_ros2_common.hpp"
#include "adrd2121_imu/srv/bias_estimate_cmd.hpp"
#include "adrd2121_imu/srv/imu_glob_cmd.hpp"

//CPP Headers
#include <chrono>
#include <iostream>

//CONSTANTS
#define MAX_WAIT_SERVICE 5
#define TRIGGER_WAIT_TIME 40

//NAMESPACE DEFINITIONS
using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

//Shutdown functions
void biasGracefulShutdownHandler(int signal);
void shutdown(void);


//! \class BiasEstimateRos2
//! \brief Class for Bias Estimate Service
class BiasEstimateRos2
{
public:
  //! \brief Constructor for BiasEstimateRos2
  BiasEstimateRos2(const rclcpp::Node::SharedPtr& node);
  //! \brief Destructor for BiasEstimateRos2
  ~BiasEstimateRos2();
  //! \brief Initializes pre-service call requirements 
  //! Indicates if bias_estimate will run or not 
  //!
  //! \return Boolean if successful (true) or not (false)
  bool init(void);

  //! \brief Callback when /bias_estimate is called
  //!
  //! \param[in] req  Request 
  //! \param[in] res  Result 
  //!
  //! \return Boolean if successful (true) or not (false)
  bool biasEstimateCB(adrd2121_imu::srv::BiasEstimateCmd::Request::SharedPtr req,
    adrd2121_imu::srv::BiasEstimateCmd::Response::SharedPtr res);
  
  //! \brief This checks the STANDSTILL duration of the IMU
  //! Also checks if it is valid for trigger_imu_glob_cmd call
  //!
  //! \param[in] standstill_begin  The start time of standstill 
  //!
  //! \return Boolean if successful (true) or not (false)
  bool checkStandStillDuration(rclcpp::Time standstill_start, double &duration_secs);
  
  
private:
  //! \brief Local node meant to catch node from main function to be used for ROS2 APIs
  rclcpp::Node::SharedPtr node_;
  //! \brief Service Server for /bias_estimate
  rclcpp::Service<adrd2121_imu::srv::BiasEstimateCmd>::SharedPtr bias_estimate_service_;
  //! \brief Service client that calls /trigger_imu_glob_cmd service
  rclcpp::Client<adrd2121_imu::srv::ImuGlobCmd>::SharedPtr trigger_imu_glob_cmd_;
  //! \brief Sets the callback group for /bias_estimate and the call for /trigger_imu_glob_cmd
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  //! \brief Pointer to ImuStateCheckerRos2 class
  ImuStateCheckerRos2* p_state_checker_;
};

#endif //BIAS_ESTIMATE_ROS2