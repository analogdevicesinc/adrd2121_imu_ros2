/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#ifndef ADI_IMU_ROS2
#define ADI_IMU_ROS2

//ROS2 Headers
#include "rclcpp/rclcpp.hpp"

//C IMU Driver Headers
#include "adi_imu_driver.h"
#include "imu_spi_buffer.h"
#include "adi_imu_regmap.h"
#include "imu_spi_buffer_regmap.h"

//IMU Driver ROS2 Headers
#include "adrd2121_imu_ros2_common.hpp"

//CPP Headers
#include <memory>
#include <unistd.h>

//CONSTANTS
#define DATA_RATE_DEFAULT 100
#define PROD_ID_DEFAULT 16470
#define TIME_BASE_CTRL_DEFAULT 10
#define MODE_DEFAULT 0
#define DEC_RATE_CONST 1


const int TIME_BASE_MIN =  0;
const int TIME_BASE_MAX = 13;

//! \brief Constant for imu_accl_bias_ vector size
const int IMU_ACCL_BIAS_SIZE = 3;

//! \brief Constant for imu_gyro_bias_ vector size
const int IMU_GYRO_BIAS_SIZE = 3;

//! \brief Constant for imu_accl_scale_ vector size
const int IMU_ACCL_SCALE_SIZE = 3;

//! \brief Constant for imu_gyro_scale_ vector size
const int IMU_GYRO_SCALE_SIZE = 3;

//! \brief Constants for range of gravity
const double GRAVITY_DEFAULT = 1.0;
const double GRAVITY_MAX = 9.81;

//! \class AdiImuRos2
//! \brief Class for ADI IMU
class AdiImuRos2
{
public:
  //! \brief Constructor for AdiImuRos2
  //! 
  //! \param[in] node     Pointer to ROS2 node(adrd2121_imu_node)
  //!
  //! \param[in] p_device Pointer to adi_imu_device_t
  AdiImuRos2(const rclcpp::Node::SharedPtr& node, adi_imu_Device_t* p_device);
  
  //! \brief Destructor for AdiImuRos2
  ~AdiImuRos2();

  //! \brief Loads Parameters for ADI IMU
  void loadParams(void);
  
  //! \brief Initializes the ADI IMU
  //! 
  //! \return Boolean if successful (true) or not (false)  
  bool init(void);

  //! \brief Configures the ADI IMU based on parameters
  //! 
  //! \return Boolean if successful (true) or not (false)
  bool config(void);

  //! \brief Return IMU Data Rate
  //! 
  //! \return IMU Data rate in Hz
  double getImuDataRateHz(void);

  //! \brief Trigger IMU Bias Correction
  //! 
  //! \return Boolean if successful (true) or not (false)
  bool triggerBiasCorrectionUpdate(void);


private:
  //! \brief Pointer to adi_imu_Device_t
  adi_imu_Device_t* p_device_;

  //! \brief Local node meant to catch node from main function to be used for ROS2 APIs
  std::shared_ptr<rclcpp::Node> node_;

  /********PARAMETERS********/
  //! \brief IMU Product ID (e.g. 16495, 16470, 16500)
  int imu_prod_id_;

  //! \brief Gravity Constant; Multiplier to accelerometer data
  double gravity_;

  //! \brief IMU Data Format (for Burst Read or non-Burst Read); 16- or 32-bit
  int imu_data_format_;

  //! \brief IMU Data Rate in Hz
  int imu_data_rate_;

  //! \brief Boolean for enabling IMU Data Ready
  bool b_imu_data_rdy_;

  //! \brief IMU Data ready line selection
  int imu_data_rdy_line_;

  //! \brief IMU Data ready polarity
  int imu_data_rdy_pol_;
  
  //! \brief Boolean for enabling configuration IMU Sync clock input
  bool b_imu_sync_clk_;

  //! \brief IMU Sync clock mode
  int imu_sync_clk_mode_;

  //! \brief IMU Sync clock input line selection
  int imu_sync_clk_line_;

  //! \brief IMU Sync clock input polarity
  int imu_sync_clk_pol_;

  //! \brief Boolean for enabling disabling Linear g compensation for gyroscopes
  bool b_imu_linear_g_comp_;

  //! \brief Boolean for enabling disabling Point of Percussion alignment
  bool b_imu_pp_align_;

  //! \brief Boolean to Trigger Bias Correction Update in IMU_GLOB_CMD Register
  bool b_imu_update_bias_corr_;

  //! \brief Time Base Control (TBC)
  int imu_time_base_control_;

  //! \brief Enable/Disable Z-axis acceleration bias correction 
  int imu_accl_z_bias_null_;
  
  //! \brief Enable/Disable Y-axis acceleration bias correction
  int imu_accl_y_bias_null_;

  //! \brief Enable/Disable X-axis acceleration bias correction
  int imu_accl_x_bias_null_;

  //! \brief Enable/Disable Z-axis gyroscope bias correction
  int imu_gyro_z_bias_null_;

  //! \brief Enable/Disable Y-axis gyroscope bias correction
  int imu_gyro_y_bias_null_;

  //! \brief Enable/Disable X-axis gyroscope bias correction
  int imu_gyro_x_bias_null_;

  //! \brief Boolean to Set Accelerometer bias
  bool b_update_imu_accl_bias_;

  //! \brief Accelerometer bias values
  std::vector<long>imu_accl_bias_;

  //! \brief Boolean to Set Gyroscope bias
  bool b_update_imu_gyro_bias_;

  //! \brief Gyroscope bias values
  std::vector<long> imu_gyro_bias_;
  
  //! \brief Boolean to Set Accelerometer scale
  bool b_update_imu_accl_scale_;

  //! \brief Accelerometer scale values
  std::vector<long> imu_accl_scale_;

  //! \brief Boolean to Set Gyroscope scale
  bool b_update_imu_gyro_scale_;

  //! \brief Gyroscope scale values
  std::vector<long> imu_gyro_scale_;

  //! \brief IMU Data Rate in Hz
  double actual_imu_data_rate_;       
};

#endif //ADI_IMU_ROS2