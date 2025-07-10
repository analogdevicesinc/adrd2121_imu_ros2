/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#include "imu_buf_ros2.hpp"

ImuBufRos2::ImuBufRos2(const rclcpp::Node::SharedPtr& node, adi_imu_Device_t* p_device)
: node_(node), p_device_(NULL)
{
  //Initializing object

  //Initializing burst_count 
  buf_burst_out_ = {RESET, RESET, RESET, RESET, RESET};
  burst_out_ = {RESET, RESET, RESET, RESET, RESET, RESET, RESET, RESET,RESET, RESET};

  //Initialize timeout for clearing USB buffer. 
  clear_buffer_timeout_ = RESET;
  
  RCLCPP_INFO(node_->get_logger(), "[ImuBufRos2] is now constructed");
  
  //Initialize data counters for checking CRC
  prev_data_count_ = RESET;
  imu_data_count_ = RESET;
  start_data_count_ = RESET;
  driver_data_count_ = RESET;
  rollover_count_ = RESET;
  drop_count_ = RESET;

  //Initialize Data Array and Counters for Buffer Burst
  buf_len_ = RESET;
  cur_buf_count_ = RESET;
  b_first_read_ = true;
  p_device_ = p_device;
}
ImuBufRos2::~ImuBufRos2()
{
  //Destroy object
  bool b_success = false;
  RCLCPP_INFO(node_->get_logger(), "[ImuBufRos2] is now destructed");
    
  if (p_device_->uartDev.status >= IMUBUF_UART_CONFIGURED)
  {
    b_success = this->stopBufferRead();
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (!b_success),
      "[ImuBufRos2] Stop Buffer Read unsuccessful.");
    RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (b_success),
      "[ImuBufRos2] Stop Buffer Read successful");
  }

  node_ = nullptr;
}
void ImuBufRos2::loadParams()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  std::ostringstream long_param_desc;
  param_desc.read_only = true;
  RCLCPP_INFO(node_->get_logger(), "[ImuBufRos2] Loading ADRD2121 Parameters");

  /***************************** TOPIC NAME PARAMETER *****************************/
  param_desc.name = "topic_name";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "ROS Topic Name";
  param_desc.additional_constraints.clear();

  node_->declare_parameter(param_desc.name, "imu/data_raw", param_desc);
  node_->get_parameter(param_desc.name, topic_name_);
  param_desc.integer_range.clear();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBuf] Loaded topic name: "<< topic_name_);
  /********************************************************************************/
  
  /***************************** DEVICE ID PARAMETER ******************************/
  param_desc.name = "usb_dev";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "USB Device Name";
  param_desc.additional_constraints = "Possible device names: /dev/ttyACM0, /dev/ttyACM1...";
  
  node_->declare_parameter(param_desc.name, "/dev/ttyACM0", param_desc);
  usb_dev_= node_->get_parameter(param_desc.name).as_string();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter usb_dev: " << usb_dev_);
  /********************************************************************************/

  /*************************** USB BAUD RATE PARAMETER ****************************/
  param_desc.name = "usb_baud";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Baud Rate on which the USB will run on";
  param_desc.additional_constraints.clear();

  node_->declare_parameter(param_desc.name, static_cast<int>(DEFAULT_BAUD_RATE), param_desc);
  usb_baud_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter usb_baud: " << usb_baud_);
  /********************************************************************************/

  /************************* ENABLE IMU BURST PARAMETER ***************************/
  param_desc.name = "enable_imu_burst";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Determine if IMU Burst Read will be enabled";
  param_desc.additional_constraints.clear();

  node_->declare_parameter(param_desc.name, true, param_desc);
  b_imu_burst_= node_->get_parameter(param_desc.name).as_bool();
  RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), b_imu_burst_,
    "[ImuBufRos2] Loaded parameter " << param_desc.name << ": true");
  RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), !b_imu_burst_,
    "[ImuBufRos2] Loaded parameter " << param_desc.name << ": false");
  /********************************************************************************/  

  /************************** BUFFER OVERFLOW PARAMETER ***************************/
  param_desc.name = "buf_overflow";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Determine behavior of Buffer board on overflow";
  param_desc.additional_constraints = "Allowed values: 0 - stop sampling; 1 - replace oldest data";

  param_int_range.from_value = BIT_LOW;
  param_int_range.step = 1;
  param_int_range.to_value = BIT_HIGH;
  param_desc.integer_range.push_back(param_int_range);

  node_->declare_parameter(param_desc.name, static_cast<int>(BIT_LOW), param_desc);
  buf_overflow_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_overflow: " <<
    buf_overflow_);
  param_desc.integer_range.clear();
  /********************************************************************************/

  /************************** ENABLE BUFFER PPS PARAMETER ***************************/
  param_desc.name = "enable_buf_pps";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Determine if PPS_ENABLE will be triggered";
  param_desc.additional_constraints.clear();
  
  node_->declare_parameter(param_desc.name, false, param_desc);
  b_buf_pps_= node_->get_parameter(param_desc.name).as_bool();
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_buf_pps_, 
    "[ImuBufRos2] Loaded parameter enable_buf_pps: true");
  RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), !b_buf_pps_, 
    "[ImuBufRos2] Loaded parameter enable_buf_pps: false");
  /********************************************************************************/

  /********************** BUFFER DATA READY SELECT PARAMETER **********************/
  param_desc.name = "buf_data_rdy_sel";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Select what pin will be used as data ready pin";
  param_desc.additional_constraints = "1-BUF_DIO1; 2-BUF_DIO2; 4-BUF_DIO3; 8-BUF_DIO4";

  node_->declare_parameter("buf_data_rdy_sel", static_cast<int>(BUF_DIO1), param_desc);
  buf_data_rdy_sel_= node_->get_parameter("buf_data_rdy_sel").as_int();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[ImuBufRos2] Setting DR_SELECT: " <<
    BUFDIOtoString(buf_data_rdy_sel_));
  /********************************************************************************/

  /********************* BUFFER DATA READY POLARITY PARAMETER *********************/
  param_desc.name = "buf_data_rdy_pol";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Set the Buffer data ready trigger polarity";
  param_desc.additional_constraints = "0 - trigger on falling edge; 1 - trigger on rising edge";

  param_int_range.from_value = BIT_LOW;
  param_int_range.step = 1;
  param_int_range.to_value = BIT_HIGH;
  param_desc.integer_range.push_back(param_int_range);

  node_->declare_parameter(param_desc.name, static_cast<int>(IMU_POS_POLARITY), param_desc);
  buf_data_rdy_pol_= node_->get_parameter("buf_data_rdy_pol").as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_data_rdy_pol: " <<
    buf_data_rdy_pol_);
  param_desc.integer_range.clear();
  /********************************************************************************/

  /************************* BUFFER PPS SELECT PARAMETER **************************/
  param_desc.name = "buf_pps_sel";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Host processor DIO output pin acts as a Pulse Per Second(PPS) input";
  param_desc.additional_constraints = "1-BUF_DIO1; 2-BUF_DIO2; 4-BUF_DIO3; 8-BUF_DIO4";

  node_->declare_parameter(param_desc.name, static_cast<int>(BUF_NONE), param_desc);
  buf_pps_sel_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_pps_sel: " <<
    BUFDIOtoString(buf_pps_sel_));
  /********************************************************************************/

  /************************* BUFFER PPS POLARITY PARAMETER **************************/
  param_desc.name = "buf_pps_pol";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Set the PPS trigger polarity";
  param_desc.additional_constraints = "0 - trigger on falling edge; 1 - trigger on rising edge";

  param_int_range.from_value = BIT_LOW;
  param_int_range.step = 1;
  param_int_range.to_value = BIT_HIGH;
  param_desc.integer_range.push_back(param_int_range);

  node_->declare_parameter(param_desc.name, static_cast<int>(IMU_NEG_POLARITY), param_desc);
  buf_pps_pol_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_pps_pol: " <<
    buf_pps_pol_);
  param_desc.integer_range.clear();
  /********************************************************************************/

  /************************ BUFFER PPS FREQUENCY PARAMETER ************************/
  param_desc.name = "buf_pps_frequency";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Set SPI Buffer Board PPS Frequency";
  param_desc.additional_constraints = "PPS Input Frequency is (10 ^ [buf_pps_freq] Hz). Range: 0-3";

  param_int_range.from_value = IMUBUF_PPS_FREQ_1HZ;
  param_int_range.step = 1;
  param_int_range.to_value = IMUBUF_PPS_FREQ_1000HZ;
  param_desc.integer_range.push_back(param_int_range);

  node_->declare_parameter(param_desc.name, static_cast<int>(IMUBUF_PPS_FREQ_1HZ), param_desc);
  buf_pps_freq_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_pps_freq: " <<
    buf_pps_freq_);
  param_desc.integer_range.clear();
  /********************************************************************************/

  /*************************** BUFFER PIN PASS PARAMETER **************************/
  param_desc.name = "buf_pin_pass";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  long_param_desc << "Pins which are directly connected from the host processor to the IMU " << 
    "using an ADG1611";
  param_desc.description = long_param_desc.str();
  long_param_desc.clear();
  param_desc.additional_constraints = "1-BUF_DIO1; 2-BUF_DIO2; 4-BUF_DIO3; 8-BUF_DIO4";

  node_->declare_parameter(param_desc.name, static_cast<int>(BUF_NONE), param_desc); 
  buf_pin_pass_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_pin_pass: " <<
    BUFDIOtoString(buf_pin_pass_));
  /********************************************************************************/
  
  /********************* BUFFER WATERMARK INTERRUPT PARAMETER *********************/
  param_desc.name = "buf_watermark_int";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Set pin for the Buffer Watermark interrupt signal";
  param_desc.additional_constraints = "1-BUF_DIO1; 2-BUF_DIO2; 4-BUF_DIO3; 8-BUF_DIO4";

  node_->declare_parameter(param_desc.name, static_cast<int>(BUF_NONE), param_desc);
  buf_watermark_int_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_watermark_int: " <<
    BUFDIOtoString(buf_watermark_int_));
  /********************************************************************************/

  /********************** BUFFER OVERFLOW INTERRUPT PARAMETER *********************/
  param_desc.name = "buf_overflow_int";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Set pin for the Buffer Overflow interrupt signal";
  param_desc.additional_constraints = "1-BUF_DIO1; 2-BUF_DIO2; 4-BUF_DIO3; 8-BUF_DIO4";

  //Set pin for the overflow interrupt signal
  node_->declare_parameter(param_desc.name, static_cast<int>(BUF_NONE), param_desc);
  buf_overflow_int_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_overflow_int: " <<
    BUFDIOtoString(buf_overflow_int_));
  /********************************************************************************/
  
  /********************** BUFFER ERROR INTERRUPT PARAMETER ***********************/
  param_desc.name = "buf_error_int";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Set pin for the Buffer Error interrupt signal";
  param_desc.additional_constraints = "1-BUF_DIO1; 2-BUF_DIO2; 4-BUF_DIO3; 8-BUF_DIO4";
  
  //Set pin for the error interrupt signal
  node_->declare_parameter(param_desc.name , static_cast<int>(BUF_NONE), param_desc);
  buf_error_int_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_error_int: " <<
    BUFDIOtoString(buf_error_int_));
  /********************************************************************************/

  /************************ BUFFER BURST COUNT PARAMETER *************************/
  param_desc.name = "buf_burst_count";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Buffer Burst count";
  param_desc.additional_constraints = "Recommended value: 1-4";

  param_int_range.from_value = BURST_COUNT_MIN;
  param_int_range.step = 1;
  param_int_range.to_value = BURST_COUNT_MAX;
  param_desc.integer_range.push_back(param_int_range);
  
  node_->declare_parameter(param_desc.name, static_cast<int>(BURST_COUNT_MIN), param_desc);
  buf_burst_count_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter buf_burst_count: " <<
    buf_burst_count_);
  param_desc.integer_range.clear();
  /********************************************************************************/

  /************************ CLEAR BUFFER TIMEOUT PARAMETER ************************/
  param_desc.name = "clear_buffer_timeout";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Timeout duration for buffer clearing (in milliseconds)";
  param_desc.additional_constraints = "Allowed range: 500-3000";

  param_int_range.from_value = MIN_CLEAR_TIMEOUT;
  param_int_range.step = 1;
  param_int_range.to_value = MAX_CLEAR_TIMEOUT;
  param_desc.integer_range.push_back(param_int_range);

  node_->declare_parameter(param_desc.name, static_cast<int>(MIN_CLEAR_TIMEOUT), param_desc);
  clear_buffer_timeout_ = node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Loaded parameter clear_buffer_timeout: " <<
    clear_buffer_timeout_);
  param_desc.integer_range.clear();
  /********************************************************************************/

  /************************ ENABLE INIT RECOVERY PARAMETER ************************/
  param_desc.name = "enable_init_recovery";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Determines whether board recovery will be done automatically at init";
  param_desc.additional_constraints.clear();

  node_->declare_parameter(param_desc.name, true, param_desc);
  b_enable_init_recovery_= node_->get_parameter(param_desc.name).as_bool();
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_enable_init_recovery_, 
    "[ImuBufRos2] Loaded parameter enable_buf_pps: true");
  RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), !b_enable_init_recovery_, 
    "[ImuBufRos2] Loaded parameter enable_buf_pps: false");
  /********************************************************************************/

  //Update the adi_imu_Device_t
  //Only USB is supported
  p_device_->devType =IMU_HW_UART;
  p_device_->uartDev.dev=usb_dev_.c_str();
  p_device_->uartDev.baud=(uint32_t)usb_baud_;
  p_device_->enable_buffer=IMU_TRUE;

  RCLCPP_INFO(node_->get_logger(), "[ImuBufRos2] Loaded Parameters");
}
bool ImuBufRos2::init(void)
{
  bool b_success = false;
  int ret = -1;
  uint16_t status = 0;

  if (b_enable_init_recovery_)
  {
    b_success = recoverBoard();
  }

  ret = this->getBufferStatus(&status);
  b_success = (Err_imu_Success_e <= ret)? true:false;
  
  if ((status == 0) && b_success)
  {
    b_success = this->detect();
  }
  else
  {
    b_success = false;
  }

  if (b_success)
  {
    //Initialize ADRD2121
    ret = imubuf_init(p_device_);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Board initialization was unsuccessful. ERROR CODE: "<< ret <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Board initialization successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }

  return b_success;
}

bool ImuBufRos2::config(msg_type_e msg_type)
{
  bool b_success = false;
  int ret = -1;
  const int page_addr_one = 1;
  const int page_addr_zero = 0;
  imubuf_ImuDioConfig_t buf_dio_config;  

  //Configure ADRD2121 DIO Pins
  buf_dio_config.dataReadyPin = buf_data_rdy_sel_;
  buf_dio_config.dataReadyPolarity = buf_data_rdy_pol_;

  if (b_buf_pps_)
  {
    buf_dio_config.ppsPin = buf_pps_sel_;
    buf_dio_config.ppsPolarity = buf_pps_pol_;
  }
  else
  {
    buf_dio_config.ppsPin = BIT_LOW;
    buf_dio_config.ppsPolarity = BIT_LOW;
  }

  buf_dio_config.passThruPin = buf_pin_pass_;
  buf_dio_config.watermarkIrqPin = buf_watermark_int_;
  buf_dio_config.overflowIrqPin = buf_overflow_int_;
  buf_dio_config.errorIrqPin = buf_error_int_;

  ret = imubuf_SetDioConfig(p_device_, &buf_dio_config);
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
    "[ImuBufRos2] Setting DIO Pin configuration was unsuccessful. ERROR CODE: "<< ret <<
    " Refer to adi_imu_Error_e.");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
    "[ImuBufRos2] Setting DIO Pin successful.");
  b_success = (Err_imu_Success_e <= ret)? true:false;

  //Enable/Disable PPS Sync
  if (b_success && b_buf_pps_)
  {
    ret = imubuf_EnablePPSSync(p_device_);

    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Setting PPS Sync was unsuccessful. ERROR CODE: "<< ret <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Setting PPS Sync successful.");
        
    b_success = (Err_imu_Success_e <= ret)? true:false;

    //Check if UTC time is properly set
    uint32_t epoch_time = time(NULL);
    uint32_t epoch_readback = RESET;

    if (b_success)
    {
      ret = imubuf_SetUTC(p_device_, epoch_time);
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
        "[ImuBufRos2] Setting UTC was unsuccessful. ERROR CODE: "<< ret <<
        " Refer to adi_imu_Error_e.");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
        "[ImuBufRos2] Setting UTC successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;
    }

    if (b_success)
    {
      ret = imubuf_GetUTC(p_device_, &epoch_readback);
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[ImuBufRos2] Acquiring UTC was unsuccessful. ERROR CODE: "<< ret <<
        " Refer to adi_imu_Error_e.");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
        "[ImuBufRos2] UTC acquisition successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;
    }

    if (b_success && (epoch_readback < epoch_time))
    {
      RCLCPP_WARN(node_->get_logger(), "[ImuBufRos2] UTC Time was not configured properly.");
      b_success = false;
    }
  }
  else if (b_success && !b_buf_pps_)
  {
    ret = imubuf_DisablePPSSync(p_device_);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
      "[ImuBufRos2] Disabling PPS Sync was unsuccessful. ERROR CODE: "<< ret <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
      "[ImuBufRos2] Disabling PPS Sync successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }
  else
  {
    //For future methods
  }

  if (b_success && b_imu_burst_)
  {
    ret = imubuf_SetPatternImuBurst(p_device_);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
      "[ImuBufRos2] Setting Pattern Imu Burst was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
      "[ImuBufRos2] Setting Pattern Imu Burst successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }
  else if (b_success && !b_imu_burst_)
  {
    if (ADIS16495 == p_device_->prodId)
    {
      uint16_t buf_pattern[] = {
        IMUBUF_PATTERN_READ_REG(REG_SYS_E_FLAG_49x),\
        IMUBUF_PATTERN_READ_REG(REG_TEMP_OUT_49x),\
        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_LOW_49x),\
        IMUBUF_PATTERN_READ_REG(REG_X_GYRO_OUT_49x),\
        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_LOW_49x),\
        IMUBUF_PATTERN_READ_REG(REG_Y_GYRO_OUT_49x),\
        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_LOW_49x),\
        IMUBUF_PATTERN_READ_REG(REG_Z_GYRO_OUT_49x),\
        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_LOW_49x),\
        IMUBUF_PATTERN_READ_REG(REG_X_ACCL_OUT_49x),\
        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_LOW_49x),\
        IMUBUF_PATTERN_READ_REG(REG_Y_ACCL_OUT_49x),\
        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_LOW_49x),\
        IMUBUF_PATTERN_READ_REG(REG_Z_ACCL_OUT_49x),\
        IMUBUF_PATTERN_READ_REG(REG_DATA_CNT_49x),\
        IMUBUF_PATTERN_READ_REG(REG_CRC_LWR_49x),\
        IMUBUF_PATTERN_READ_REG(REG_CRC_UPR_49x), 0x0000,\
      };
      uint16_t buf_pattern_len = (uint16_t) (sizeof(buf_pattern)/sizeof(uint16_t));
      ret = imubuf_SetPatternRaw(p_device_, buf_pattern_len, buf_pattern);

      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[ImuBufRos2] Setting Pattern Imu was unsuccessful. ERROR CODE: " << ret <<
        " Refer to adi_imu_Error_e.");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
        "[ImuBufRos2] Setting Pattern Imu successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;
    }
    else if ((ADIS16470 == p_device_->prodId) || (ADIS16500 == p_device_->prodId))
    {
      //TODO on future versions.
    }
  }

  //Configure and Set Burst Mode
  imubuf_BufConfig_t buf_config;
  buf_config.overflowAction = buf_overflow_;
  buf_config.imuBurstEn = b_imu_burst_;
  buf_config.bufBurstEn = RESET; //Always disabled for USB
  if (ADIS16495 == p_device_->prodId)
  {
    buf_config.imuPageAddr = page_addr_one;
  }
  else if ((ADIS16470 == p_device_->prodId) || (ADIS16500 == p_device_->prodId))
  {
    buf_config.imuPageAddr = page_addr_zero;
  }

  if(b_success)
  {
    ret = imubuf_SetBufConfig(p_device_, &buf_config);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), "[ImuBufRos2]"<<
    " Setting Pattern Imu was unsuccessful. ERROR CODE: " << ret << " Refer to adi_imu_Error_e.");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), "[ImuBufRos2]"<<
    " Setting Pattern Imu successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;    
  }

  imubuf_DevInfo_t imuBufInfo;
  ret = imubuf_GetInfo(p_device_, &imuBufInfo);
  b_success = (Err_imu_Success_e <= ret)? true:false;  
  
  if (b_success)
  {
    ret = imubuf_PrintInfo(p_device_, &imuBufInfo);
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }

  if (b_success)
  {
    //Advertising publishers 
    if (SENSOR_MSGS_IMU == msg_type)
    {
      imu_pub_std_ = node_->create_publisher<sensor_msgs::msg::Imu> \
        (topic_name_.c_str(), THROW_LIMIT);
    }
    else if (ADI_IMU_MSG == msg_type)
    {
      imu_pub_adi_ = node_->create_publisher<adrd2121_imu::msg::AdiImu> \
        (topic_name_.c_str(), THROW_LIMIT);
    }
    /*else
    {
      for any other msgs 
    }*/  
  }

  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_success, "[ImuBufRos2] Buffer board"<<
  " configuration successful.");

  return b_success;
}

bool ImuBufRos2::startBufferRead(void)
{
  int ret = -1;
  bool b_success = false;

  //Start Capture in ADRD2121
  ret = imubuf_StartCapture(p_device_, IMU_FALSE, &cur_buf_count_);
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
    "[ImuBufRos2] Start Capture was unsuccessful. ERROR CODE: " << ret <<
    " Refer to adi_imu_Error_e.");
  RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
    "[ImuBufRos2] Start Capture successful.");
  b_success = (Err_imu_Success_e <= ret)? true:false;

  if (b_success)
  {
    //Do initial burst read and discard it
    ret = imubuf_ReadBurstN(p_device_, 5, (uint16_t*)burst_raw_, &buf_len_);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Burst Read process was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Burst Read process successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }
  return b_success;
}

bool ImuBufRos2::validateData(void)
{
  bool b_success = false;

  //Initialize Data Counters for Checking CRC
  const int data_cnt_start = 0;
  uint32_t total_data_count = 0; //driverCntPlusDropCnt
  uint32_t drop_count_current = 0; //dropCountCurrent
  uint32_t calculated_crc = 0;

  #if defined(ADRD2121_IMU_TIMING_DEBUG)
    print_time(std::string("T4 @"));
  #endif
  
  //Calculate checksum/CRC
  if (p_device_->dataFormat ==IMU_DATA_32BIT)
  {
    if (ADIS16500 == p_device_->prodId)
    {
    /** BURST READ FORMAT
      * For 16500, BRF for:
      *   0x0000, DIAG_STAT, X_GYRO_LOW, X_GYRO_OUT, Y_GYRO_LOW, Y_GYRO_OUT, Z_GYRO_LOW, Z_GYRO_OUT,
      *   X_ACCL_LOW, X_ACCL_OUT, Y_ACCL_LOW, Y_ACCL_OUT, Z_ACCL_LOW, Z_ACCL_OUT, 
      *   TEMP_OUT, DATA_CNT, CRC
      *
      * CRC = DIAG_STAT[15:8] + DIAG_STAT[7:0] +
      *       X_GYRO_LOW[15:8] + X_GYRO_LOW[7:0] + X_GYRO_OUT[15:8] + X_GYRO_OUT[7:0] +
      *       Y_GYRO_LOW[15:8] + Y_GYRO_LOW[7:0] + Y_GYRO_OUT[15:8] + Y_GYRO_OUT[7:0] +
      *       Z_GYRO_LOW[15:8] + Z_GYRO_LOW[7:0] + Z_GYRO_OUT[15:8] + Z_GYRO_OUT[7:0] +
      *       X_ACCL_LOW[15:8] + X_ACCL_LOW[7:0] + X_ACCL_OUT[15:8] + X_ACCL_OUT[7:0] +
      *       Y_ACCL_LOW[15:8] + Y_ACCL_LOW[7:0] + Y_ACCL_OUT[15:8] + Y_ACCL_OUT[7:0] +
      *       Z_ACCL_LOW[15:8] + Z_ACCL_LOW[7:0] + Z_ACCL_OUT[15:8] + Z_ACCL_OUT[7:0] +
      *       TEMP_OUT[15:8] + TEMP_OUT[7:0] + DATA_CNT[15:8] + DATA_CNT[7:0]
      *
      **/

      RCLCPP_DEBUG(node_->get_logger(), "[DEBUG] Burst IMU Data Raw: %X %X %X %X %X %X %X %X %X \
        %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X",
        buf_burst_out_.data[0], buf_burst_out_.data[1], buf_burst_out_.data[2],
        buf_burst_out_.data[3], buf_burst_out_.data[4], buf_burst_out_.data[5], 
        buf_burst_out_.data[6], buf_burst_out_.data[7], buf_burst_out_.data[8],
        buf_burst_out_.data[9], buf_burst_out_.data[10], buf_burst_out_.data[11],
        buf_burst_out_.data[12], buf_burst_out_.data[13], buf_burst_out_.data[14],
        buf_burst_out_.data[15], buf_burst_out_.data[16], buf_burst_out_.data[17],
        buf_burst_out_.data[18], buf_burst_out_.data[19], buf_burst_out_.data[20],
        buf_burst_out_.data[21], buf_burst_out_.data[22], buf_burst_out_.data[23],
        buf_burst_out_.data[24], buf_burst_out_.data[25], buf_burst_out_.data[26],
        buf_burst_out_.data[27], buf_burst_out_.data[28], buf_burst_out_.data[29],
        buf_burst_out_.data[30], buf_burst_out_.data[31], buf_burst_out_.data[32],
        buf_burst_out_.data[33]);

      calculated_crc = (uint16_t)(buf_burst_out_.data[2] + buf_burst_out_.data[3] + 
        buf_burst_out_.data[4] + buf_burst_out_.data[5] + buf_burst_out_.data[6] + 
        buf_burst_out_.data[7] + buf_burst_out_.data[8] + buf_burst_out_.data[9] + 
        buf_burst_out_.data[10] + buf_burst_out_.data[11] + buf_burst_out_.data[12] +
        buf_burst_out_.data[13] + buf_burst_out_.data[14] + buf_burst_out_.data[15] +
        buf_burst_out_.data[16] + buf_burst_out_.data[17] + buf_burst_out_.data[18] +
        buf_burst_out_.data[19] + buf_burst_out_.data[20] + buf_burst_out_.data[21] +
        buf_burst_out_.data[22] + buf_burst_out_.data[23] + buf_burst_out_.data[24] +
        buf_burst_out_.data[25] + buf_burst_out_.data[26] + buf_burst_out_.data[27] + 
        buf_burst_out_.data[28] + buf_burst_out_.data[29] +buf_burst_out_.data[30] +
        buf_burst_out_.data[31]);
    }
    else if(ADIS16495 == p_device_->prodId)
    {
    /** BURST READ FORMAT
      * For 16495, BRF for (fsclk < 3MHz):
      *   0x0000, 0xA5A5 (BURST_ID), SYS_E_FLAG, TEMP_OUT, X_GYRO_LOW, X_GYRO_OUT, Y_GYRO_LOW, 
      *   Y_GYRO_OUT, Z_GYRO_LOW, Z_GYRO_OUT, X_ACCL_LOW, X_ACCL_OUT, Y_ACCL_LOW, Y_ACCL_OUT, 
      *   Z_ACCL_LOW, Z_ACCL_OUT, DATA_CNT, CRC_LWR, CRC_UPR
      *
      * CRC is based on CRC32 Calculation
      **/

      RCLCPP_DEBUG(node_->get_logger(), "[DEBUG] Burst IMU Data Raw: %X %X %X %X %X %X %X %X %X \
      %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X",
        buf_burst_out_.data[0], buf_burst_out_.data[1], buf_burst_out_.data[2], 
        buf_burst_out_.data[3], buf_burst_out_.data[4], buf_burst_out_.data[5],
        buf_burst_out_.data[6], buf_burst_out_.data[7], buf_burst_out_.data[8],
        buf_burst_out_.data[9], buf_burst_out_.data[10], buf_burst_out_.data[11],
        buf_burst_out_.data[12], buf_burst_out_.data[13], buf_burst_out_.data[14],
        buf_burst_out_.data[15], buf_burst_out_.data[16], buf_burst_out_.data[17],
        buf_burst_out_.data[18], buf_burst_out_.data[19], buf_burst_out_.data[20],
        buf_burst_out_.data[21], buf_burst_out_.data[22], buf_burst_out_.data[23],
        buf_burst_out_.data[24], buf_burst_out_.data[25], buf_burst_out_.data[26],
        buf_burst_out_.data[27], buf_burst_out_.data[28], buf_burst_out_.data[29],
        buf_burst_out_.data[30], buf_burst_out_.data[31], buf_burst_out_.data[32],
        buf_burst_out_.data[33], buf_burst_out_.data[34], buf_burst_out_.data[35],
        buf_burst_out_.data[36], buf_burst_out_.data[37], buf_burst_out_.data[38],
        buf_burst_out_.data[39]);

      calculated_crc = (unsigned)0xFFFFFFFF;
      uint16_t *p_data = (uint16_t*)(buf_burst_out_.data + 6);
      calculated_crc = crc32_block(calculated_crc,p_data,15);
      calculated_crc = calculated_crc ^ (unsigned)0xFFFFFFFF;
    }
  }
  else if (p_device_->dataFormat == IMU_DATA_16BIT)
  {
  /** BURST READ FORMAT
    * For 16500 or 16470, BRF for:
    *   0x0000, DIAG_STAT, X_GYRO_OUT, Y_GYRO_OUT, Z_GYRO_OUT, X_ACCL_OUT, Y_ACCL_OUT, Z_ACCL_OUT,
    *   TEMP_OUT, DATA_CNT, CRC
    *
    * CRC = DIAG_STAT[15:8] + DIAG_STAT[7:0] +
    *       X_GYRO_OUT[15:8] + X_GYRO_OUT[7:0] +
    *       Y_GYRO_OUT[15:8] + Y_GYRO_OUT[7:0] +
    *       Z_GYRO_OUT[15:8] + Z_GYRO_OUT[7:0] +
    *       X_ACCL_OUT[15:8] + X_ACCL_OUT[7:0] +
    *       Y_ACCL_OUT[15:8] + Y_ACCL_OUT[7:0] +
    *       Z_ACCL_OUT[15:8] + Z_ACCL_OUT[7:0] +
    *       TEMP_OUT[15:8] + TEMP_OUT[7:0] +
    *       DATA_CNT[15:8] + DATA_CNT[7:0]
    **/

    RCLCPP_DEBUG(node_->get_logger(), "[DEBUG] Burst IMU Data Raw: %X %X %X %X %X %X \
    %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X %X",
      buf_burst_out_.data[0], buf_burst_out_.data[1], buf_burst_out_.data[2],
      buf_burst_out_.data[3], buf_burst_out_.data[4], buf_burst_out_.data[5],
      buf_burst_out_.data[6], buf_burst_out_.data[7], buf_burst_out_.data[8],
      buf_burst_out_.data[9], buf_burst_out_.data[10], buf_burst_out_.data[11],
      buf_burst_out_.data[12], buf_burst_out_.data[13], buf_burst_out_.data[14],
      buf_burst_out_.data[15], buf_burst_out_.data[16], buf_burst_out_.data[17], 
      buf_burst_out_.data[18], buf_burst_out_.data[19], buf_burst_out_.data[20],
      buf_burst_out_.data[21]);

    calculated_crc = (uint16_t)(buf_burst_out_.data[2] + buf_burst_out_.data[3] +
      buf_burst_out_.data[4] + buf_burst_out_.data[5] + buf_burst_out_.data[6] + 
      buf_burst_out_.data[7] + buf_burst_out_.data[8] + buf_burst_out_.data[9] +
      buf_burst_out_.data[10] + buf_burst_out_.data[11] + buf_burst_out_.data[12] +
      buf_burst_out_.data[13] +buf_burst_out_.data[14] + buf_burst_out_.data[15] +
      buf_burst_out_.data[16] +buf_burst_out_.data[17] +buf_burst_out_.data[18] +
      buf_burst_out_.data[19]);
  }

  #if defined(ADRD2121_IMU_TIMING_DEBUG)
    print_time(std::string("T5 @"));
  #endif
  
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"[DEBUG] CRC: " << burst_out_.crc);
  RCLCPP_DEBUG_STREAM(node_->get_logger(),"[DEBUG] Calculated CRC: " << calculated_crc);

  //Process only valid burst data
  if (burst_out_.crc == calculated_crc)
  {
    //Update data counters for the first time
    if (data_cnt_start == driver_data_count_)
    {
      if (data_cnt_start < burst_out_.dataCntOrTimeStamp)
      {
        prev_data_count_ = burst_out_.dataCntOrTimeStamp - 1;
      }
      else
      {
        prev_data_count_ = RESET;
      }
      driver_data_count_ = burst_out_.dataCntOrTimeStamp;
      start_data_count_ = driver_data_count_;
    }
    else
    {
      driver_data_count_++;
    }

    //Update rollover count on every overflow (i.e. 65535 to 0 transition)
    if ((RESET < prev_data_count_) && (burst_out_.dataCntOrTimeStamp < prev_data_count_))
    {
      rollover_count_++;
    }

    prev_data_count_ = burst_out_.dataCntOrTimeStamp;
    imu_data_count_ = burst_out_.dataCntOrTimeStamp + (rollover_count_ * BUFFER_LIMIT);
    total_data_count = driver_data_count_ + drop_count_;

    if (imu_data_count_ > total_data_count)
    {
      drop_count_current = imu_data_count_ - total_data_count;
      drop_count_ += drop_count_current;
      RCLCPP_DEBUG_STREAM(node_->get_logger(), "Current drop_count = "<< drop_count_);
    }
    else if (imu_data_count_ < total_data_count)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Invalid data count. Current IMU Count = " <<
        imu_data_count_ << " Driver count = " << driver_data_count_);
    }

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "IMU Count = "<< imu_data_count_ << " Driver count" <<
      driver_data_count_ << "Drop Count = " << drop_count_ << "dataCntOrTimeStamp: " <<
      burst_out_.dataCntOrTimeStamp << "rollover_count = " << rollover_count_ << 
      " total_data_count = " << total_data_count);

    b_success = true;
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Invalid CRC. Received %X Actual: %X", burst_out_.crc, 
      calculated_crc);
  }
  return b_success;
}

bool ImuBufRos2::dataReadAndPub(std::string frame_name, msg_type_e msg_type)
{
  int ret = -1;    
  bool b_success = false;
  bool b_valid_data = false;

  if (b_first_read_)
  {
    b_success = this->startBufferRead();
    //Set message type
    msg_type_ = msg_type;
    b_first_read_ = false;
  }
  else
  {
    b_success = true;
  }

  if(b_success)
  {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "[ImuBufRos2] Now reading Buffer Data Stream (USB)");

    //Read buffer data
    #if defined(ADRD2121_IMU_TIMING_DEBUG)
      print_time(std::string("T0 @"));
    #endif
    ret = imubuf_ReadBurstN(p_device_, buf_burst_count_, (uint16_t*)burst_raw_, &buf_len_); 
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
      "[ImuBufRos2] Reading of Buffer Burst was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
      "[ImuBufRos2] Reading of Buffer Burst successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

    #if defined(ADRD2121_IMU_TIMING_DEBUG)
      print_time(std::string("T1 @"));
    #endif

    for(int i = RESET; buf_burst_count_ > i; i++)
    {
      //imubuf_BurstOutputRaw_t to imubuf_BurstOutput_t
      if(b_success)
      {
        ret = imubuf_ScaleBurstOut(p_device_, (imubuf_BurstOutputRaw_t*)(burst_raw_+buf_len_*i), 
          &(buf_burst_out_));
        RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
          "[ImuBufRos2] Scale Burst Out successful");
        b_success = (Err_imu_Success_e <= ret)? true:false;
        if (!b_success)
        {
          RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
            "[ImuBufRos2] Reading of Buffer Burst was unsuccessful. ERROR CODE: " << ret <<
            " Refer to adi_imu_Error_e.");
          i = buf_burst_count_ + BIT_HIGH;
        }
      }
      
      //Check if bytes need to be swapped
      memset((uint8_t*)&burst_out_, RESET, sizeof(burst_out_));
      adi_imu_Boolean_e en_byte_swap;

      if((p_device_->devType == IMU_HW_UART) && (p_device_->uartDev.status >= IMUBUF_UART_READY))
      {
        en_byte_swap = IMU_FALSE;
      }
      else
      {
        en_byte_swap = IMU_TRUE;
      }
      
      if (b_success)
      {
        if (ADIS16495 == p_device_->prodId)
        {
          ret = adi_imu_ScaleBurstOut(p_device_, buf_burst_out_.data, IMU_TRUE, en_byte_swap, 
            &burst_out_);
        }
        else if ((ADIS16470 == p_device_->prodId) || (ADIS16500 == p_device_->prodId))
        {
          //no Burst ID in ADIS16470 and ADIS16500
          ret = adi_imu_ScaleBurstOut(p_device_, buf_burst_out_.data, IMU_FALSE, en_byte_swap,
            &burst_out_);
        }
        
        RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
          "[ImuBufRos2] ADI IMU Scale Burst Out was unsuccessful. ERROR CODE: " << ret <<
          " Refer to adi_imu_Error_e.");
        RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
          "[ImuBufRos2] ADI IMU Scale Burst Out successful.");
        b_success = (Err_imu_Success_e <= ret)? true:false;

        if (ret == Err_imu_BurstFrameInvalid_e)
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), 
            "ADI IMU Scale Burst Out: BurstFrameInvalid; continuing loop..." );
          b_success = true;
          continue;
        }
      }
      if (b_success)
      {
        //Parse UTC timestamps
        utc_time_ = buf_burst_out_.bufUtcTime;
        utc_time_us_ = buf_burst_out_.bufTimestamp;
        b_valid_data = this->validateData();

        RCLCPP_DEBUG_EXPRESSION(node_->get_logger(), b_valid_data, "VALIDATE DATA SUCCESSFUL");
        if (b_valid_data)
        {
          RCLCPP_DEBUG(node_->get_logger(), "PUBLISHING NOW");
          #if defined(ADI_IMU_ROS_TIMING_DEBUG)
            print_time(std::string("T2 @"));
          #endif

          //Publishing to topic with sensor_msgs/msg/Imu msg type
          if(msg_type_ == SENSOR_MSGS_IMU)
          {
            sensor_msgs::msg::Imu imu_msg;
            imu_msg.header.stamp = node_->now();
            imu_msg.header.frame_id = frame_name.c_str();
            imu_msg.angular_velocity.x = burst_out_.gyro.x*(M_PI/RADIAN_CONSTANT);
            imu_msg.angular_velocity.y = burst_out_.gyro.y*(M_PI/RADIAN_CONSTANT);
            imu_msg.angular_velocity.z = burst_out_.gyro.z*(M_PI/RADIAN_CONSTANT);
            imu_msg.linear_acceleration.x = burst_out_.accl.x;
            imu_msg.linear_acceleration.y = burst_out_.accl.y;
            imu_msg.linear_acceleration.z = burst_out_.accl.z;
            imu_pub_std_->publish(imu_msg);
          }
          //Publishing to topic with adrd2121_imu/msg/AdiImu msg type
          else if(msg_type_ == ADI_IMU_MSG)
          {
            adrd2121_imu::msg::AdiImu imu_msg;
            imu_msg.header.stamp = node_->now();
            imu_msg.header.frame_id = frame_name.c_str();
            imu_msg.angular_velocity.x = burst_out_.gyro.x*(M_PI/RADIAN_CONSTANT);
            imu_msg.angular_velocity.y = burst_out_.gyro.y*(M_PI/RADIAN_CONSTANT);
            imu_msg.angular_velocity.z = burst_out_.gyro.z*(M_PI/RADIAN_CONSTANT);
            imu_msg.linear_acceleration.x = burst_out_.accl.x;
            imu_msg.linear_acceleration.y = burst_out_.accl.y;
            imu_msg.linear_acceleration.z = burst_out_.accl.z;
            imu_msg.imu_count = imu_data_count_;
            imu_msg.driver_count = driver_data_count_;
            imu_msg.drop_count = drop_count_;
            imu_msg.buf_utc_sec = utc_time_;
            imu_msg.buf_utc_usec = utc_time_us_;
            imu_pub_adi_->publish(imu_msg);
          }
        }
      }
    }
  }
  return b_success;
}

bool ImuBufRos2::stopBufferRead(void)
{
  bool b_success = false;
  int ret = -1;
  int duration_ms;
  int buffer_size = 0;
  typedef std::chrono::high_resolution_clock Clock;
  Clock::time_point start_time;
  Clock::time_point current_time;
  std::chrono::duration<double,std::milli> duration;  

  if (p_device_->uartDev.status >= IMUBUF_UART_CONFIGURED)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "[ImuBufRos2] Stop Capture sequence initiated.");
    ret = imubuf_StopCapture(p_device_, &cur_buf_count_);
        
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Stop Capture was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Stop Capture successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }

  if (b_success)
  {
    //Reset driver data count
    driver_data_count_ = RESET;
    rollover_count_ = RESET;

    //This is for clearing the USB buffer to avoid overloading 
    start_time = Clock::now();
    do
    {
      current_time = Clock::now();
      duration = current_time - start_time; 
      duration_ms = (int) duration.count();

      //API for flushing buffer for UART
      hw_FlushInput(p_device_);
      //Check buffer contents, bytes left in buffer. 
      ioctl(p_device_->uartDev.fd, FIONREAD, &buffer_size);


      //If buffer is cleared, end loop. 
      if (buffer_size == 0)
      {
        break;
      }
    }
    while (duration_ms < clear_buffer_timeout_);

    b_success = (buffer_size == 0) ? true:false;
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (!b_success), 
      "[ImuBufRos2] Clearing of USB buffer unsuccessful. Reached timeout." << 
      "Refer to YAML file for set duration.");
    RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (b_success), 
      "[ImuBufRos2] Clearing of USB buffer successful.");
  }

  return b_success;
}

//Helper Function
const char * BUFDIOtoString(int dio)
{
  const char* output= new char[20];

  switch(dio)
  {
    case 0:
      output="Disabled/None";
      break;
    case 1:
      output="DIO1";
      break;
    case 2:
      output="DIO2";
      break;
    case 4:
      output="DIO3";
      break;
    case 8:
      output="DIO4";
      break;
  }
  return output;
}

int ImuBufRos2::getBufferStatus(uint16_t * status)
{
  int ret = -1;
  bool b_success = false;
  std::string status_description = "";
  imubuf_SysStatus_t buf_status;
  
  ret = imubuf_GetSysStatus(p_device_, &buf_status);
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Check Status was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
  RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Check Status successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

  if (b_success)
  {
    //Concatenate Buffer Board status in one value
    uint16_t buf_status_decimal = (buf_status.bufWaterMark << BITP_ISENSOR_STATUS_BUF_WTRMRK) | 
      (buf_status.bufFull << BITP_ISENSOR_STATUS_BUF_FULL) | \
      (buf_status.spiError << BITP_ISENSOR_STATUS_SPI_ERROR) | \
      (buf_status.spiOverflow << BITP_ISENSOR_STATUS_SPI_OVRFLW) | \
      (buf_status.overrun << BITP_ISENSOR_STATUS_OVERRUN) | \
      (buf_status.dmaError << BITP_ISENSOR_STATUS_DMA_ERROR) | \
      (buf_status.ppsUnlock << BITP_ISENSOR_STATUS_PPS_UNLOCK) | \
      (buf_status.tempWarning << BITP_ISENSOR_STATUS_TEMP_WARNING) | \
      (buf_status.scriptError << BITP_ISENSOR_STATUS_SCRIPT_ERROR) | \
      (buf_status.scriptActive << BITP_ISENSOR_STATUS_SCRIPT_ACTIVE) | \
      (buf_status.flashError << BITP_ISENSOR_STATUS_FLASH_ERROR) | \
      (buf_status.flashUpdateError << BITP_ISENSOR_STATUS_FLASH_UPD_ERROR) | \
      (buf_status.fault << BITP_ISENSOR_STATUS_FAULT) | \
      (buf_status.watchdog<< BITP_ISENSOR_STATUS_WATCHDOG);
    
    *status = buf_status_decimal;

    RCLCPP_INFO(node_->get_logger(), "IMU BUF System Status: 0x%04X", buf_status_decimal);
    
    status_description = this->getStatusDescription(*status);

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "\nBuffer Status: \n" <<
      "  BUF_WATERMARK: " << static_cast<int>(buf_status.bufWaterMark) << "\n" <<
      "  BUF_FULL: " << static_cast<int>(buf_status.bufFull) << "\n" <<
      "  SPI_ERROR: " << static_cast<int>(buf_status.spiError) << "\n" <<
      "  SPI_OVERFLOW: " << static_cast<int>(buf_status.spiOverflow) << "\n" <<
      "  OVERRUN: " << static_cast<int>(buf_status.overrun) << "\n" <<
      "  DMA_ERROR: " << static_cast<int>(buf_status.dmaError) << "\n" <<
      "  PPS_UNLOCK: " << static_cast<int>(buf_status.ppsUnlock) << "\n" <<
      "  TEMP_WARNING: " << static_cast<int>(buf_status.tempWarning) << "\n" <<
      "  SCRIPT_ERROR: " << static_cast<int>(buf_status.scriptError) << "\n" <<
      "  SCRIPT_ACTIVE: " << static_cast<int>(buf_status.scriptActive) << "\n" <<
      "  FLASH_ERROR: " << static_cast<int>(buf_status.flashError) << "\n" <<
      "  FLASH_UPDATE_ERROR: " << static_cast<int>(buf_status.flashUpdateError) << "\n" <<
      "  FAULT: " << static_cast<int>(buf_status.fault)<< "\n" <<
      "  WATCHDOG: " << static_cast<int>(buf_status.watchdog) << "\n");
  }
  return ret;
}

std::string ImuBufRos2::getStatusDescription(uint16_t status)
{
  std::string message="";

  if(BITM_ISENSOR_STATUS_BUF_WTRMRK & status)
  {
    message.append("  [INFO] Buffer watermark is set.");
    RCLCPP_INFO(node_->get_logger(), "  Buffer watermark is set.");
  }
  if(BITM_ISENSOR_STATUS_BUF_FULL & status)
  {
    message.append("  [INFO] Buffer is FULL.");
    RCLCPP_INFO(node_->get_logger(), "  Buffer is FULL.");
  }
  if(BITM_ISENSOR_STATUS_SPI_ERROR & status)
  {
    message.append("  [ERROR] SPI Error.");
    RCLCPP_INFO(node_->get_logger(), "  SPI Error.");
  }
  if(BITM_ISENSOR_STATUS_SPI_OVRFLW & status)
  {
    message.append("  [ERROR] SPI overflow error.");
    RCLCPP_INFO(node_->get_logger(), "  SPI overflow error.");
  }
  if(BITM_ISENSOR_STATUS_OVERRUN & status)
  {
    message.append("  [ERROR] Data capture overrun error.");
    RCLCPP_INFO(node_->get_logger(), "  Data capture overrun error.");
  }
  if(BITM_ISENSOR_STATUS_DMA_ERROR & status)
  {
    message.append("  [ERROR] DMA error.");
    RCLCPP_INFO(node_->get_logger(), "  DMA error.");
  }
  if(BITM_ISENSOR_STATUS_PPS_UNLOCK & status)
  {
    message.append("  [INFO] PPS is UNLOCKED.");
    RCLCPP_INFO(node_->get_logger(), "  PPS is UNLOCKED.");
  }
  if(BITM_ISENSOR_STATUS_TEMP_WARNING & status)
  {
    message.append("  [WARN] Temperature outside safe range [-40C to 85C].");
    RCLCPP_INFO(node_->get_logger(), "  Temperature outside safe range [-40C to 85C].");
  }
  if(BITM_ISENSOR_STATUS_SCRIPT_ERROR & status)
  {
    message.append("  [ERROR] Script launch error.");
    RCLCPP_INFO(node_->get_logger(), "  Script launch error.");
  }
  if(BITM_ISENSOR_STATUS_SCRIPT_ACTIVE & status)
  {
    message.append("  [INFO] Script is ACTIVE.");
    RCLCPP_INFO(node_->get_logger(), "  Script is ACTIVE.");
  }
  if(BITM_ISENSOR_STATUS_FLASH_ERROR & status)
  {
    message.append("  [ERROR] FLASH verify failed.");
    RCLCPP_INFO(node_->get_logger(), "  FLASH verify failed.");
  }
  if(BITM_ISENSOR_STATUS_FLASH_UPD_ERROR & status)
  {
    message.append("  [ERROR] FLASH update failed.");
    RCLCPP_INFO(node_->get_logger(), "  FLASH update failed.");
  }
  if(BITM_ISENSOR_STATUS_FAULT & status)
  {
    message.append("  [ERROR] Processor fault occured.");
    RCLCPP_INFO(node_->get_logger(), "  Processor fault occured.");
  }
  if(BITM_ISENSOR_STATUS_WATCHDOG & status)
  {
    message.append("  [ERROR] Processor reset due to watchdog.");
    RCLCPP_INFO(node_->get_logger(), "  Processor reset due to watchdog.");
  }
  return message;
}

bool ImuBufRos2::factoryReset(void)
{
  int ret = -1;
  bool b_success = false;

  ret = imubuf_FactoryReset(p_device_);
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Factory Reset was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
  RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Factory Reset successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

  return b_success;
}

bool ImuBufRos2::flashUpdate(void)
{
  int ret = -1;
  bool b_success = false;

  ret = imubuf_FlashUpdate(p_device_);
  delay_MicroSeconds(API_CALL_DELAY);
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Flash Update was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
  RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Flash Update successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

  return b_success;
}

bool ImuBufRos2::clearFault(void)
{
  int ret = -1;
  bool b_success = false;

  ret = imubuf_ClearFault(p_device_);
  delay_MicroSeconds(API_CALL_DELAY);
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Clear Fault was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
  RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Clear Fault successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

  return b_success;
}

bool ImuBufRos2::recoverBoard(void)
{
  uint8_t retry_count = 0;
  uint8_t ret_value = 0;
  uint16_t status = 0xFFFF;
  bool b_buffer_detected = false;
  bool b_success = false;
  while (MAX_CLEAR_RETRY >= retry_count)
  {
    b_buffer_detected = this->detect();
    ret_value = getBufferStatus(&status);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret_value),
      "[ImuBufRos2] Check Status was unsuccessful. ERROR CODE: " << ret_value <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret_value),
      "[ImuBufRos2] Check Status successful.");
    b_success = (Err_imu_Success_e <= ret_value)? true:false;

    if (b_success)
    {
      if ((status == 0) && (b_buffer_detected))
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), 
          "[ImuBufRos2] Status is cleared, will not proceed on recovery operation");
        break;
      }
      else
      {
        if (MAX_CLEAR_RETRY == retry_count)
        {
          RCLCPP_ERROR_STREAM(node_->get_logger(), "[ImuBufRos2] Board recovery has failed.");
          b_success = false;
        }
      }
    }

    RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), (b_success), 
      "[ImuBufRos2] Now starting board recovery operation");
    if (b_success)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[ImuBufRos2] Now calling Factory Reset");
      b_success = this->factoryReset();
    }

    if (b_success)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[ImuBufRos2] Now calling Clear Fault");
      b_success = this->clearFault();
    }

    if (b_success)
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), "[ImuBufRos2] Now calling Flash Update");
      b_success = this->flashUpdate();
    }
    retry_count++;
  }

  return b_success;
}

void ImuBufRos2::initServiceServers(uint8_t mode_of_operation)
{
  if (RECOVERY == mode_of_operation)
  {
    //Advertise services for RECOVERY
    p_factoryReset = node_->create_service<std_srvs::srv::Trigger>
      ("factory_reset", std::bind(&ImuBufRos2::factoryResetCB, this, _1, _2));
    p_clearFault = node_->create_service<std_srvs::srv::Trigger>
      ("clear_fault", std::bind(&ImuBufRos2::clearFaultCB, this, _1, _2));
    p_flashUpdate = node_->create_service<std_srvs::srv::Trigger>
      ("flash_update", std::bind(&ImuBufRos2::flashUpdateCB, this, _1, _2));
  }
    //Advertise service for checking status of board
    p_getBufferStatus = node_->create_service<adrd2121_imu::srv::BufStatus>
      ("get_buffer_status", std::bind(&ImuBufRos2::getBufferStatusCB, this, _1, _2));    
}

bool ImuBufRos2::factoryResetCB(std_srvs::srv::Trigger::Request::SharedPtr req, 
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req;
  bool b_success = false;

  b_success = this->factoryReset();

  if (b_success)
  {
    res->success = true;
    res->message = "Factory Reset successful";
  }
  else
  {
    res->success = false;
    res->message = "Factory Reset failed";
  }

  return true;
}

bool ImuBufRos2::clearFaultCB(std_srvs::srv::Trigger::Request::SharedPtr req, 
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req;
  bool b_success = false;

  b_success = this->clearFault();

  if (b_success)
  {
    res->success = true;
    res->message = "Clear Fault successful";
  }
  else
  {
    res->success = false;
    res->message = "Clear Fault failed";
  }
  return true;
}

bool ImuBufRos2::flashUpdateCB(std_srvs::srv::Trigger::Request::SharedPtr req, 
  std_srvs::srv::Trigger::Response::SharedPtr res)
{
  (void)req;
  bool b_success = false;

  b_success = this->flashUpdate();

  if (b_success)
  {
    res->success = true;
    res->message = "Flash Update successful";
  }
  else
  {
    res->success = false;
    res->message = "Flash Update failed";
  }

  return true;
}

bool ImuBufRos2::getBufferStatusCB(adrd2121_imu::srv::BufStatus::Request::SharedPtr req, 
  adrd2121_imu::srv::BufStatus::Response::SharedPtr res)
{
  (void)req;
  bool b_success = false;
  uint16_t status = 0;
  int ret = -1;
  std::string message = "";

  if (STREAM == mode_of_operation_)
  {
    b_success = stopBufferRead();
  }
  else if (RECOVERY == mode_of_operation_)
  {
    b_success = true;
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown operation.");  
  }

  if (b_success)
  {
    ret = this->getBufferStatus(&status);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[ImuBufRos2] Check Status was unsuccessful. ERROR CODE: " << ret <<
      " Refer to adi_imu_Error_e.");
    RCLCPP_DEBUG_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[ImuBufRos2] Check Status successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

    if (b_success)
    {
      res->success = true;
      res->status = status;

      // Convert status to hex
      std::stringstream status_hex;
      status_hex << std::hex << status;
      res->status_hex = status_hex.str();

      message=this->getStatusDescription(status);
      res->message=message;
    }
    else
    {
      res->success = false;
      res->message = "Failed to get status";
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to stop stream");  
  }

  if (STREAM == mode_of_operation_)
  {
    b_success = startBufferRead();
    
    if (b_success)
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Streaming resumed");
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed to restart stream"); 
    }
  }

  return true;
}

void ImuBufRos2::setModeofOperation(uint8_t mode_of_operation)
{
  mode_of_operation_ = mode_of_operation;
}

bool ImuBufRos2::detect(void)
{
  int ret = -1;
  bool b_success = false;
  
  //Detect if ADRD2121 is connected
  ret = imubuf_Detect(p_device_);
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
    "[ImuBufRos2] Board NOT detected. Please check if board is connected. ERROR CODE: "<< ret <<
    " Refer to adi_imu_Error_e.");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
    "[ImuBufRos2] Board is now connected to Host PC.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

  return b_success;
}
