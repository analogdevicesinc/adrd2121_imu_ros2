/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#ifndef IMU_BUF_ROS2
#define IMU_BUF_ROS2

//ROS2 Header
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

//IMU C Driver Headers
#include "adi_imu_driver.h"
#include "imu_spi_buffer.h"
#include "adrd2121_imu_ros2_common.hpp"

//Msg types for Publisher
#include "sensor_msgs/msg/imu.hpp"
#include "adrd2121_imu/msg/adi_imu.hpp"

//Srv types for Services
#include "adrd2121_imu/srv/buf_status.hpp"
#include "adrd2121_imu/srv/imu_glob_cmd.hpp"


//CPP Headers
#include <memory>
#include <chrono>
#include <string>
#include <stdint.h>
#include <sys/ioctl.h>

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

const char * BUFDIOtoString(int dio);

//CONSTANTS
#define DEFAULT_BAUD_RATE 921600
#define THROW_LIMIT 1000
#define BUFFER_LIMIT 65536
#define RADIAN_CONSTANT 180
#define MAX_CLEAR_RETRY 3
#define API_CALL_DELAY 2000000
#define MAX_CLEAR_TIMEOUT 3001
#define MIN_CLEAR_TIMEOUT 500
#define BURST_COUNT_MIN 1
#define BURST_COUNT_MAX 4


//! \enum Buffer DIO Pin
enum buf_dio_pin_e
{
  BUF_NONE=0,
  BUF_DIO1=1,
  BUF_DIO2=2,
  BUF_DIO3=4,
  BUF_DIO4=8,
};


//! \class ImuBufRos2 
//! \brief Class for ADRD2121
class ImuBufRos2
{
public:
  //! \brief ImuBufRos2 Constructor
  //!
  //! \param[in] node     Pointer to ROS2 node(adrd2121_imu_node)
  //! \param[in] p_device Pointer to adi_imu_device_t     
  ImuBufRos2(const rclcpp::Node::SharedPtr& node, adi_imu_Device_t* p_device);

  //! \brief ImuBufRos2 Destructor
  ~ImuBufRos2();

  //! \brief Loads Parameters for ADRD2121
  void loadParams(void);

  //! \brief Initializes the ADRD2121 
  //! 
  //! \return Boolean if successful (true) or not (false)  
  bool init(void);
  
  //! \brief Configures the ADRD2121 based on parameters
  //! \param[in] msg_type   Determine the msg_type the topic will be published on
  //! \return Boolean if successful (true) or not (false)  
  bool config(msg_type_e msg_type);

  //! \brief An all-in-one function for reading the burst_data from the IMU, handle the data, and
  //! publish it to a topic
  //! \param[in] frame_name   ROS Frame of data to be published
  //! \param[in] msg_type     Determine the msg_type the topic will be published on
  //! \return Boolean if successful (true) or not (false)  
  bool dataReadAndPub(std::string frame_name, msg_type_e msg_type);

  //! \brief Validate data
  //! 
  //! \return Boolean if successful (true) or not (false)  
  bool validateData(void);

  //! \brief Starts capture of ADRD2121 Data
  //! Performs an initial read and discards it
  //!
  //! \return Boolean if successful (true) or not (false)
  bool startBufferRead(void);

  //! \brief Stops capture of ADRD2121 Data
  //!
  //! \return Boolean if successful (true) or not (false)
  bool stopBufferRead(void);

  //! \brief Checks status of ADRD2121
  //! \param[in] status   Pointer to holder of status  
  //! \return Return if function call is successful
  int getBufferStatus(uint16_t * status);

  //! \brief Reverts the ADRD2121 to its factory settings
  //!
  //! \return Return if function call is successful
  bool factoryReset(void);

  //! \brief Clears FAULT status in Buffer Board
  //!
  //! \return Return if function call is successful
  bool clearFault(void);

  //! \brief Saves current board configuration in flash memory, to be loaded in next bootup
  //!
  //! \return Return if function call is successful
  bool flashUpdate(void);

  //! \brief Executes recovery operation to clear status back to 0000
  //! \brief Will only be called if enable_init_recovery is TRUE
  //! \return Return if function call is successful
  bool recoverBoard(void);

  //! \brief Initializes service servers based on operation mode
  //!
  //! \param[in] mode_of_operation Determine if operation is STREAM or RECOVERY
  //!
  //! \return Return if function call is successful
  void initServiceServers(uint8_t mode_of_operation);

  //! \brief Callback when /bias_estimate is called
  //!
  //! \param[in] status  Status to be evaluated 
  //! \param[in] res  Result 
  //!
  //! \return String of current status of Buffer Board. 
  std::string getStatusDescription(uint16_t status);

  //! \brief Callback when /factory_reset is called
  //!
  //! \param[in] req  Request 
  //! \param[in] res  Result 
  //!
  //! \return Boolean if successful (true) or not (false)
  bool factoryResetCB(std_srvs::srv::Trigger::Request::SharedPtr req, 
    std_srvs::srv::Trigger::Response::SharedPtr res);

  //! \brief Callback when /clear_fault is called
  //!
  //! \param[in] req  Request 
  //! \param[in] res  Result 
  //!
  //! \return Boolean if successful (true) or not (false)
  bool clearFaultCB(std_srvs::srv::Trigger::Request::SharedPtr req, 
    std_srvs::srv::Trigger::Response::SharedPtr res);

  //! \brief Callback when /flash_update is called
  //!
  //! \param[in] req  Request 
  //! \param[in] res  Result 
  //!
  //! \return Boolean if successful (true) or not (false)
  bool flashUpdateCB(std_srvs::srv::Trigger::Request::SharedPtr req, 
    std_srvs::srv::Trigger::Response::SharedPtr res);
  
  //! \brief Callback when /check_status is called
  //!
  //! \param[in] req  Request 
  //! \param[in] res  Result 
  //!
  //! \return Boolean if successful (true) or not (false)
  bool getBufferStatusCB(adrd2121_imu::srv::BufStatus::Request::SharedPtr req, 
    adrd2121_imu::srv::BufStatus::Response::SharedPtr res);

  //! \brief Initializes service servers based on operation mode
  //!
  //! \param[in] mode_of_operation Acquire value from param in AdiImuBufRos2
  //!
  //! \return Return if function call is successful
  void setModeofOperation(uint8_t mode_of_operation);

  //! \brief Checks if ADRD2121 device can be detected.
  //! 
  //! \return Return bool if device is found 
  bool detect(void);

private:
  
  //! \brief Publisher for topic using sensor_msgs as interface
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_std_;

  //! \brief Publisher for topic using ADI custom msg as interface
  rclcpp::Publisher<adrd2121_imu::msg::AdiImu>::SharedPtr imu_pub_adi_;

  //! \brief Name of Topic Publisher
  std::string topic_name_;

  //! \brief Local node meant to catch node from main function to be used for ROS2 APIs
  rclcpp::Node::SharedPtr node_;

  //! \brief Name of USB device 
  std::string usb_dev_;
  
  //! \brief USB Baud Rate 
  int usb_baud_;
  
  //! \brief Enable/Disable IMU Burst Read
  bool b_imu_burst_;

  //! \brief Buffer overflow behavior
  int buf_overflow_;

  //! \brief Boolean to trigger PPS_ENABLE
  bool b_buf_pps_;

  //! \brief IMU DIO output pin is treated as data ready
  int buf_data_rdy_sel_;

  //! \brief Data ready trigger polarity
  int buf_data_rdy_pol_;

  //! \brief PPS trigger polarity
  int buf_pps_pol_;

  //! \brief Host processor DIO output pin acts as a Pulse Per Second (PPS) Input
  int buf_pps_sel_;

  //! \brief PPS Input Frequency is (10 ^ (buf_pps_freq) Hz)
  int buf_pps_freq_;

  //! \brief Pins which are directly connected from the host processor to the IMU using an ADG1611
  int buf_pin_pass_;

  //! \brief Pin for the buffer watermark interrupt signal
  int buf_watermark_int_;

  //! \brief Pin for the buffer overflow interrupt signal
  int buf_overflow_int_;

  //! \brief Pin for the error interrupt signal
  int buf_error_int_;

  //! \brief Counter for Previous Data
  uint32_t prev_data_count_;

  //! \brief Counter for IMU Data
  uint32_t imu_data_count_;

  //! \brief Counter for initial data count
  uint64_t start_data_count_;

  //! \brief Counter for data received from Buffer Board
  uint64_t driver_data_count_;
  
  //! \brief Rollover Counter (in excess of 65536)
  uint32_t rollover_count_;

  //! \brief Counter of dropped data
  uint32_t drop_count_;

  //Initialize Data Array and Counters for Buffer Burst
  //! \brief Buffer Length
  uint16_t buf_len_;

  //! \brief Raw data from ADRD2121
  uint16_t burst_raw_[MAX_BUF_LEN_BYTES*10];

  //! \brief Struct for scaled data from ADRD2121
  imubuf_BurstOutput_t buf_burst_out_;

  //! \brief Time stamp
  uint32_t utc_time_;
  //! \brief Time stamp
  uint32_t utc_time_us_;

  //! \brief Struct for scaled IMU data from buf_burst_out_
  adi_imu_BurstOutput_t burst_out_;

  //! \brief Current buffer count
  uint16_t cur_buf_count_;

  //! \brief Boolean for first reading
  //! Needed when initializing before reading buffer data_raw
  bool b_first_read_;

  //! \brief POinter to main device struct
  adi_imu_Device_t * p_device_;

  //! \brief ROS Message type of imu_pub_[msg]
  msg_type_e msg_type_;
  
  //! \brief Buffer burst count
  int buf_burst_count_;
  
  //! \brief For enabling/disabling automatic recovery operation
  bool b_enable_init_recovery_;

  //! \brief Determine what operation mode will the node be running in
  uint8_t mode_of_operation_;

  //! \brief Timeout for clearing the USB buffer, in milliseconds
  int clear_buffer_timeout_; 

  //! \brief Service Server for /factory_reset
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr p_factoryReset;

  //! \brief Service Server for /clear_fault
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr p_clearFault;

  //! \brief Service Server for /flash_update
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr p_flashUpdate;

  //! \brief Service Server for /check_status
  rclcpp::Service<adrd2121_imu::srv::BufStatus>::SharedPtr p_getBufferStatus;
};

#endif //IMU_BUF_ROS2
