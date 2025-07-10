/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/
#include <iostream>
#include <unistd.h>
using namespace std;

#include "adi_imu_buf_ros2.hpp"

AdiImuBufRos2::AdiImuBufRos2(const rclcpp::Node::SharedPtr& node):
  node_(node)
{
  //Object initialization
  RCLCPP_INFO(node_->get_logger(), "[AdiImuBufRos2] is now constructed.");
  p_adi_imu_ros2_obj_ = new AdiImuRos2(node_, &imu_dev_);
  p_imu_buf_ros2_obj_ = new ImuBufRos2(node_, &imu_dev_);
  param_callback = node_->add_on_set_parameters_callback(
    std::bind(&AdiImuBufRos2::paramCbFunction, this, _1));
  b_readPubDataSuccess = true;
}

AdiImuBufRos2::~AdiImuBufRos2()
{

  //Destroying objects and cleaning of node shared_ptr
  RCLCPP_INFO(node_->get_logger(), "[AdiImuBufRos2] is now destructed");
  node_ = nullptr;
    
  delete p_adi_imu_ros2_obj_;
  p_adi_imu_ros2_obj_ = nullptr;
  
  delete p_imu_buf_ros2_obj_;
  p_imu_buf_ros2_obj_ = nullptr;

  close(imu_dev_.uartDev.fd);
}

void AdiImuBufRos2::initParams()
{
  rcl_interfaces::msg::ParameterDescriptor parameter_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  //For catching the incoming parameter to be able to typecast it into msg_type_e
  int msg_type_receive_;

  //This applies to all of the parameters
  parameter_desc.read_only = true;
  
  p_adi_imu_ros2_obj_->loadParams();
  p_imu_buf_ros2_obj_->loadParams();
  
  RCLCPP_INFO(node_->get_logger(), "[AdiImuBufRos2] Loading ROS Parameters.");  

  /***************************** FRAME NAME PARAMETER *****************************/
  parameter_desc.name = "frame_name";
  parameter_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  parameter_desc.description = "ROS Frame name";
  parameter_desc.additional_constraints.clear();

  node_->declare_parameter(parameter_desc.name, "imu", parameter_desc);
  frame_name_ = node_->get_parameter(parameter_desc.name).as_string();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] Loaded frame name: " << frame_name_);
  /********************************************************************************/
  
  /**************************** MESSAGE TYPE PARAMETER ****************************/
  parameter_desc.name = "msg_type";
  parameter_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  parameter_desc.description = "Message Type of ROS topic";
  parameter_desc.additional_constraints = "Supported msgs are: sensor_msgs_imu - 1; adi_imu - 2";

  param_int_range.from_value = SENSOR_MSGS_IMU;
  param_int_range.step = 1;
  param_int_range.to_value = ADI_IMU_MSG;
  parameter_desc.integer_range.push_back(param_int_range);

  node_->declare_parameter(parameter_desc.name, (int)SENSOR_MSGS_IMU, parameter_desc);
  msg_type_receive_ = node_->get_parameter(parameter_desc.name).as_int();
  msg_type_ = (msg_type_e)msg_type_receive_;
  parameter_desc.integer_range.clear();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] Loaded message type: " << msg_type_);
  /********************************************************************************/

  /************************* MODE OF OPERATION PARAMETER **************************/
  parameter_desc.name = "mode_of_operation";
  parameter_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  parameter_desc.description = "Mode of operation";
  parameter_desc.additional_constraints = "Supported operation modes are: STREAM - 1; RECOVER - 2";

  param_int_range.from_value = STREAM;
  param_int_range.step = 1;
  param_int_range.to_value = RECOVERY;
  parameter_desc.integer_range.push_back(param_int_range);

  node_->declare_parameter(parameter_desc.name, STREAM, parameter_desc);
  mode_of_operation_ = node_->get_parameter(parameter_desc.name).as_int();
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] Loaded Mode of Operation: " << 
    ((mode_of_operation_ == STREAM) ? "STREAM" : "RECOVERY"));
  p_imu_buf_ros2_obj_->setModeofOperation(mode_of_operation_);
  /********************************************************************************/

  //ADI IMU Info
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] imu.prodId = " << imu_dev_.prodId);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] imu.g = " << imu_dev_.g);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] imu.enable_buffer = " <<
    imu_dev_.enable_buffer);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] imu.devtype = " <<
    imu_dev_.devType);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] imu.uartDev.dev = " <<
    imu_dev_.uartDev.dev);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBufRos2] imu.uartDev.baud = " <<
    imu_dev_.uartDev.baud);
}

bool AdiImuBufRos2::init(void)
{
  RCLCPP_INFO(node_->get_logger(), "[AdiImuBufRos2] Starting device initialization");
  bool b_success = false;
  //For storing the return values of the C Driver APIs
  int ret = -1;
  //For storing board status
  uint16_t status;
  
  //Check Lib Build Info
  adi_imu_BuildInfo_t binfo = adi_imu_GetBuildInfo(&imu_dev_);
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuBufRos2] IMU_LIB_VERSION = " << binfo.version);
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuBufRos2] IMU_LIB_BUILD_TIME = " <<
    binfo.build_time);
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuBufRos2] IMU_LIB_BUILD_TYPE = " <<
    binfo.build_type);

  //Initialize connection between device and host
  ret = hw_Init(&imu_dev_);
  b_getHWInitialized_ = (Err_imu_Success_e == ret)? true:false;
  RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
    "[AdiImuBufRos2] Device initialization failed. ERROR CODE: " << ret <<
    " Refer to adi_imu_Error_e");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
    "[AdiImuBufRos2] Device initialization successful");
  b_success = (Err_imu_Success_e == ret)? true:false;

  if (b_success)
  {
    p_imu_buf_ros2_obj_->detect();
    ret = p_imu_buf_ros2_obj_->getBufferStatus(&status);
    b_success = (Err_imu_Success_e <= ret)? true:false;
    
    if ((STREAM == mode_of_operation_) && b_success)
    {
      //Initialize ADRD2121
      b_success = p_imu_buf_ros2_obj_->init();
      if (b_success)
      {
        //Initialize ADI IMU
        b_success = p_adi_imu_ros2_obj_->init();
      }
      
      if (!b_success)
      {
        mode_of_operation_ = RECOVERY;
        p_imu_buf_ros2_obj_->setModeofOperation(mode_of_operation_);
      }
    } 

    if ((RECOVERY == mode_of_operation_) || (STREAM == mode_of_operation_))
    {
      p_imu_buf_ros2_obj_->initServiceServers(mode_of_operation_);
    }
    else
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),"Unknown operation mode");
    }
  }

  return b_success;
}

bool AdiImuBufRos2::config(void)
{
  bool b_success = false;
  
  double imu_data_rate_hz = 0.0;

  //Default for Buffer Burst Count 
  int buf_burst_count_ = 1;

  node_->get_parameter("buf_burst_count", buf_burst_count_);

  RCLCPP_INFO(node_->get_logger(), "[AdiImuBufRos2] Starting ADI IMU device configuration.");
  b_success = p_adi_imu_ros2_obj_->config();
    
  if (b_success)
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuBufRos2] Starting ADRD2121 SPI\
    Buffer Board configuration.");
    b_success = p_imu_buf_ros2_obj_->config(msg_type_);
  }

  //Data Rate won't be used if device configuration failed. 
  if (b_success)
  {    
    // Make sure the ROS loop rate is greater than (IMU data rate/Burst Count)
    imu_data_rate_hz = p_adi_imu_ros2_obj_->getImuDataRateHz();
    ros_loop_rate_hz_ = TWICE * (imu_data_rate_hz / buf_burst_count_);
    RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuBufRos2] Setting ROS2 Loop Rate to: " <<
      ros_loop_rate_hz_ << "Hz");
    
    //Advertisement of TrigGlobCmd
    if ((16470 == imu_dev_.prodId) || (16495 == imu_dev_.prodId))
    {
      //Advertise service
      RCLCPP_INFO(node_->get_logger(), "[AdiImuBufRos2] Advertising trigger_imu_glob_cmd service");
      p_imu_glob_cmd_service_ = node_->create_service<adrd2121_imu::srv::ImuGlobCmd>
        ("trigger_imu_glob_cmd", std::bind(&AdiImuBufRos2::triggerImuGlobCmd, this, _1, _2)); 
    }
    else
    {
      RCLCPP_WARN_STREAM(node_->get_logger(), 
        "[AdiImuBufRos2] trigger_imu_glob_cmd is not supported in ADIS16500");
    }
  }
  return b_success;
}


void AdiImuBufRos2::readPubData(void)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuBufRos2] Starting publishing IMU data");
  std::chrono::duration<double, std::milli> timer_duration;
  double get_ros_rate_hz = 0.0;
  
  get_ros_rate_hz = this->getRosLoopRateHz();
  
  timer_duration = std::chrono::duration<double, std::milli>(1000/get_ros_rate_hz);
  
  //Initializes the wall_timer which will be called depending on 
  //the rate that will be acquired from adi_imu_ros2.
  timer_ = node_->create_wall_timer(timer_duration,
    std::bind(&AdiImuBufRos2::dataReadAndPubCB, this));
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuBufRos2] TimerCallback loaded");
}

void AdiImuBufRos2::dataReadAndPubCB(void)
{
  b_readPubDataSuccess = p_imu_buf_ros2_obj_->dataReadAndPub(frame_name_, (msg_type_e)msg_type_);
}

double AdiImuBufRos2::getRosLoopRateHz(void)
{
  RCLCPP_INFO(node_->get_logger(), "[AdiImuBufRos2] Acquiring Loop Rate.");
  return ros_loop_rate_hz_;
}

rcl_interfaces::msg::SetParametersResult AdiImuBufRos2::paramCbFunction(
  const std::vector<rclcpp::Parameter> &param)
{
  rcl_interfaces::msg::SetParametersResult result;
  const int valid_DIO = 0;
  const int invalid_DIO = 6;
      
  for(const rclcpp::Parameter &read_param : param)
  {
    
    if(read_param.get_name() == "buf_data_rdy_sel")
    {
      if ((BUF_DIO1 > read_param.as_int()) || (BUF_DIO4 < read_param.as_int()))
      {
        result.reason = "Parameter value is out of range";
        result.successful = false;
      }
      else if ((BUF_DIO1 == read_param.as_int()) || ((valid_DIO == read_param.as_int()%TWICE) && 
        (invalid_DIO != read_param.as_int())))
      {
        result.reason = "Pass";
        result.successful = true;
      }
      else
      {
        result.reason = "Parameter value is invalid";
        result.successful = false;
      }
    }
    else if((read_param.get_name() == "buf_pps_sel") || 
      (read_param.get_name() == "buf_pin_pass") || 
      (read_param.get_name() == "buf_watermark_int") ||
      (read_param.get_name() == "buf_overflow_int") ||
      (read_param.get_name() == "buf_error_int"))
    {
      if ((BUF_NONE > read_param.as_int()) || (BUF_DIO4 < read_param.as_int()))
      {
        result.reason = "Parameter value is out of range";
        result.successful = false;
      }
      else if ((BUF_DIO1 == read_param.as_int()) || ((valid_DIO == read_param.as_int()%TWICE) && 
        (invalid_DIO != read_param.as_int())))
      {
        result.reason = "Pass";
        result.successful = true;
      }
      else
      {
        result.reason = "Parameter value is invalid";
        result.successful = false;
      }
    }
    else
    {
      result.reason = "Pass";
      result.successful = true;
    }
  }
  
  return result;
}

bool AdiImuBufRos2::triggerImuGlobCmd(
    std::shared_ptr<adrd2121_imu::srv::ImuGlobCmd::Request> req,
    std::shared_ptr<adrd2121_imu::srv::ImuGlobCmd::Response> res)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), 
    "[trigger_imu_glob_cmd service] Initiating write sequence to IMU GLOB_CMD Register...");
    
  bool b_success = false;
  bool b_valid_data = false; 
  uint16_t data = BIT_HIGH<<req->bit;

  // Check Validity of input
  // Currently only GLOB_CMD_BIAS_CORR_UPD is supported (Bit 0)
  uint16_t bitm_glob_cmd_bias_corr_upd = RESET;
  if (ADIS1647x == imu_dev_.imuProd)
  {
    bitm_glob_cmd_bias_corr_upd = BITM_GLOB_CMD_BIAS_CORR_UPD_47x;
  }
  else if (ADIS1649x == imu_dev_.imuProd)
  {
    bitm_glob_cmd_bias_corr_upd = BITM_GLOB_CMD_BIAS_CORR_UPD_49x;
  }    
  /*else
  {
    Intended for future devices
  }*/
        
  if (data & bitm_glob_cmd_bias_corr_upd)
  {
    b_valid_data = true;
  }
  else
  {
    res->success = false;
    res->message = "Loaded triggering bit is not supported in current device.";
    RCLCPP_WARN_STREAM(node_->get_logger(), 
      "[trigger_imu_glob_cmd service] SERVICE RUN FAIL. Triggering bit is unsupported.");
  }

  if (b_valid_data)
  {
    //Stop data capture first
    RCLCPP_INFO_STREAM(node_->get_logger(), 
      "Now stopping ADRD2121 Data Capture. IMU data publishing is paused.");
    b_success = p_imu_buf_ros2_obj_->stopBufferRead();
    if (b_success)
    {
      if (data & bitm_glob_cmd_bias_corr_upd)
      {
        b_success = p_adi_imu_ros2_obj_->triggerBiasCorrectionUpdate();
      }
      else
      {
        b_success = false;
      }
    }

    if (b_success)
    {
      res->success = true;
      res->message = "Trigger sequence for IMU Bias Correction Update is SUCCESSFUL.";
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[trigger_imu_glob_cmd service] Trigger Successful.");
    }
    else
    {
      res->success = false;
      res->message = "Trigger sequence for IMU Bias Correction Update has FAILED.";
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[trigger_imu_glob_cmd service] Trigger Failed.");
    }

    //Start data capture again. 
    RCLCPP_INFO_STREAM(node_->get_logger(), 
      "[trigger_imu_glob_cmd service] Now resuming ADRD2121 Data Capture. " <<
      "IMU data is now publishing.");
    b_success = p_imu_buf_ros2_obj_->startBufferRead();

    if (!b_success)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
        "[trigger_imu_glob_cmd service] Data capture has failed to restart. " <<
        "Please relaunch the node.");
    }
  }
  else
  {
    res->success = false;
    res->message ="Trigger sequence for IMU Bias Correction Update has FAILED.";
    RCLCPP_ERROR(node_->get_logger(),
      "[trigger_imu_glob_cmd service] Unable to trigger since IMU data capture was not stopped");
  }

  return true;
}
