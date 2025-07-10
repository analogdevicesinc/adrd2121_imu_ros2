/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#ifndef IMU_STATE_CHECKER_ROS2
#define IMU_STATE_CHECKER_ROS2

//ROS2 Header
#include "rclcpp/rclcpp.hpp"

//IMU Driver ROS2 Header
#include "adrd2121_imu_ros2_common.hpp"

//Msgs for subscribers
#include "sensor_msgs/msg/imu.hpp"
#include "adrd2121_imu/msg/adi_imu.hpp"

//Msg for publisher
#include "adrd2121_imu/msg/imu_state.hpp"

//CPP Headers
#include <deque>
#include <cmath>
#include <unistd.h>

//NAMESPACE DEFINITIONS
using std::placeholders::_1;
using std::placeholders::_2;

//CONSTANTS
#define DATA_LIMIT 1000
#define BUFFER_LIMIT 9
#define WAIT_LIMIT 3

//! \enum e_imu_state
//! State of IMU
typedef enum e_imu_state
{
  STANDSTILL = 0,
  MOVING = 1
}e_imu_state;

//! \class ImuStateCheckerRos2
//! \brief Checks state of IMU (STANDSTILL or MOVING)
class ImuStateCheckerRos2
{
public:
  //! \brief Constructor for StateChecker
  ImuStateCheckerRos2(const rclcpp::Node::SharedPtr& node);
  
  //! \brief Destructor for StateChecker 
  ~ImuStateCheckerRos2();

  //! \brief Evaluates that state of the IMU based on the standard deviation of the magnitude
  //!
  //! \param[in] im_msg  The IMU data that will be evaluated
  //!
  //! \return state of the IMU
  e_imu_state evaluateState(const sensor_msgs::msg::Imu::SharedPtr imu_msg);

  //! \brief Callback when IMU topic is received
  //! 
  //! \param[in] msg  The IMU data (sensor_msgs)
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  //! \brief Callback when IMU topic is received
  //! 
  //! \param[in] msg  The IMU data (AdiImu)
  void imuCallback(const adrd2121_imu::msg::AdiImu::SharedPtr msg);

  //! \brief Calculates the standard deviation
  //! 
  //! \param[in] data  The deque of data
  //!
  //! \return The computed standard deviation
  float getStandardDev(std::deque<float> data);

  //! \brief Gets the Time Start when IMU is STANDSTILL
  void evaluateStandstillBegin(void);
  
  //! \brief Returns the Time Start when IMU is STANDSTILL
  //! 
  //! \return value of standstill_begin_
  rclcpp::Time getStandstillBegin(void);

  //! \brief Returns the current state
  //! 
  //! \return value of state_
  e_imu_state getState(void);

  //! \brief Loads parameters for the IMU state checker
  //! 
  //! \return Boolean if successful (true) or not (false)
  bool loadParams(void);

  //! \brief Initialize imu/data_raw subscription and publisher of imu_state
  //! 
  //! \return Boolean if successful (true) or not (false)
  bool init(void);


private:
  //! \brief Local node meant to catch node from main function to be used for ROS2 APIs
  rclcpp::Node::SharedPtr node_;

  //! \brief Publisher of IMU state topic
  rclcpp::Publisher<adrd2121_imu::msg::ImuState>::SharedPtr publish_imu_state_;
  
  //! \brief Subscriber to IMU topic(AdiImu)
  rclcpp::Subscription<adrd2121_imu::msg::AdiImu>::SharedPtr adi_imu_sub_;
  
  //! \brief Subscriber to IMU topic(sensor_msgs)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sensor_imu_sub_;
  
  //! \brief Current state of IMU
  e_imu_state state_;
  
  //! \brief Previous state of IMU
  e_imu_state prev_state_;

  //! \brief Time start when IMU is STANDSTILL
  rclcpp::Time standstill_begin_;
  
  //! \brief Contains the recent 10 magnitude of Gyroscope data
  std::deque<float> gyro_mag_q_;

  //! \brief Contains the recent 10 magnitude of Accelerometer data
  std::deque<float> accl_mag_q_;

  //! \brief Gyroscope Standard deviation threshold
  double gyro_std_thresh_;

  //! \brief Accelerometer Standard Deviation threshold
  double accl_std_thresh_;

  //! \brief IMU message type
  int imu_msg_type_;

  //! \brief IMU topic name
  std::string imu_topic_name_;
};
#endif //IMU_STATE_CHECKER_ROS2
