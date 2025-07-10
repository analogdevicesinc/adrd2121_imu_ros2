/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#include "adi_imu_ros2.hpp"

AdiImuRos2::AdiImuRos2(const rclcpp::Node::SharedPtr& node, adi_imu_Device_t* p_device)
: p_device_(NULL), node_(node), imu_accl_bias_(IMU_ACCL_BIAS_SIZE, RESET),
  imu_gyro_bias_(IMU_GYRO_BIAS_SIZE, RESET),imu_accl_scale_(IMU_ACCL_SCALE_SIZE, RESET),
  imu_gyro_scale_(IMU_GYRO_SCALE_SIZE, RESET)
{

  //Initialize object
  p_device_ = p_device;
  RCLCPP_INFO(node_->get_logger(), "[AdiImuRos2] is now constructed");
}

AdiImuRos2::~AdiImuRos2()
{
  //Clean node shared_ptr and device struct
  RCLCPP_INFO(node_->get_logger(), "[AdiImuRos2] is now destructed");
  node_ = nullptr;
  p_device_ = nullptr;
}

void AdiImuRos2::loadParams()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  rcl_interfaces::msg::IntegerRange param_int_range;
  rcl_interfaces::msg::FloatingPointRange param_float_range;
  std::ostringstream long_param_desc;
  param_desc.read_only = true;
  RCLCPP_INFO(node_->get_logger(), "[AdiImuRos2] Loading ADI IMU Parameters");

  /*************************** IMU PRODUCT ID PARAMETER ***************************/
  param_desc.name = "imu_prod_id";
  param_desc.type = rclcpp::ParameterType::PARAMETER_STRING;
  param_desc.description = "IMU Product ID";
  param_desc.additional_constraints = "Supported IMUs: 16470, 16500";

  node_->declare_parameter(param_desc.name, static_cast<int>(PROD_ID_DEFAULT), param_desc);
  node_->get_parameter(param_desc.name, imu_prod_id_);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[AdiImuBuf] Loaded parameter imu_prod_id: "
    << imu_prod_id_);
  /********************************************************************************/

  /******************************* GRAVITY PARAMETER ******************************/
  param_desc.name = "gravity";
  param_desc.type = rclcpp::ParameterType::PARAMETER_DOUBLE;
  param_desc.description = "Gravity constant used in computation";
  param_desc.additional_constraints = "If 1.0, accelerometer output is normalized to g";

  param_float_range.from_value = GRAVITY_DEFAULT;
  param_float_range.step = 0.1;
  param_float_range.to_value = GRAVITY_MAX;
  param_desc.floating_point_range.push_back(param_float_range);

  node_->declare_parameter(param_desc.name, static_cast<double>(GRAVITY_DEFAULT), param_desc);
  gravity_ = node_->get_parameter(param_desc.name).as_double();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded parameter gravity: "<<
    gravity_ );
  /********************************************************************************/

  /*************************** IMU DATA FORMAT PARAMETER **************************/
  param_desc.name = "imu_data_format";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "IMU Data Format";
  param_desc.additional_constraints = "For ADIS16470, 16-bit is fixed for Burst Read";

  param_int_range.from_value = IMU_DATA_16BIT;
  param_int_range.step = 1;
  param_int_range.to_value = IMU_DATA_32BIT;
  param_desc.integer_range.push_back(param_int_range);
  
  node_->declare_parameter(param_desc.name, static_cast<int>(IMU_DATA_32BIT), param_desc);
  imu_data_format_ = node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded parameter imu_data_format: "<<
    imu_data_format_);
  param_desc.integer_range.clear();
  /********************************************************************************/
  
  /**************************** IMU DATA RATE PARAMETER ***************************/
  param_desc.name = "imu_data_rate";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description= "Data rate (in Hz) that the IMU will run in";
  param_desc.additional_constraints.clear();

  node_->declare_parameter(param_desc.name, static_cast<int>(DATA_RATE_DEFAULT), param_desc);
  imu_data_rate_ = node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded parameter imu_data_rate: "<<
    imu_data_rate_);
  /********************************************************************************/
  
  /************************* IMU DATA READY LINE PARAMETER ************************/
  param_desc.name = "imu_data_rdy_line";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "DIO Pin for IMU Data Ready Line";
  param_desc.additional_constraints = "0 - IMU_DIO1; 1 - IMU_DIO2; 2 - IMU_DIO3; 3 - IMU_DIO4"; 

  param_int_range.from_value = IMU_DIO1;
  param_int_range.step = 1;
  param_int_range.to_value = IMU_DIO4;
  param_desc.integer_range.push_back(param_int_range);

  node_->declare_parameter("imu_data_rdy_line", static_cast<int>(IMU_DIO1), param_desc);
  imu_data_rdy_line_ = node_->get_parameter("imu_data_rdy_line").as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded parameter imu_data_rdy_line: "<<
    imu_data_rdy_line_);
  param_desc.integer_range.clear();
  /********************************************************************************/

  /********************** IMU DATA READY POLARITY PARAMETER ***********************/
  param_desc.name = "imu_data_rdy_pol";
  param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
  param_desc.description = "Polarity of IMU Data Ready";
  param_desc.additional_constraints = "0 - IMU_NEG_POLARITY; 1 - IMU_POS_POLARITY";

  param_int_range.from_value = IMU_NEG_POLARITY;
  param_int_range.step = 1;
  param_int_range.to_value = IMU_POS_POLARITY;
  param_desc.integer_range.push_back(param_int_range);
  
  node_->declare_parameter(param_desc.name, static_cast<int>(IMU_POS_POLARITY), param_desc);
  imu_data_rdy_pol_= node_->get_parameter(param_desc.name).as_int();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded parameter imu_data_rdy_pol: "<<
    imu_data_rdy_pol_);
  param_desc.integer_range.clear();
  /********************************************************************************/
  
  /*********************** ENABLE IMU SYNC CLOCK PARAMETER ************************/
  param_desc.name = "enable_imu_sync_clk";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Determine if IMU Clock Sync will be enabled";
  param_desc.additional_constraints.clear();

  node_->declare_parameter(param_desc.name, false, param_desc);
  b_imu_sync_clk_ = node_->get_parameter(param_desc.name).as_bool();
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_imu_sync_clk_, 
    "[AdiImuRos2] Loaded parameter enable_imu_sync_clk: true");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), !b_imu_sync_clk_, 
    "[AdiImuRos2] Loaded parameter enable_imu_sync_clk: false");
  /********************************************************************************/
  if (b_imu_sync_clk_)
  {
    /*********************** IMU SYNC CLOCK MODE PARAMETER ************************/
    param_desc.name = "imu_sync_clk_mode";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "Mode of IMU Sync Clock";
    long_param_desc << "0- IMU_INTERNAL_CLOCK; 1 - IMU_DIRECT_SYNC;" << 
      "2 - IMU_SCALED_SYNC; 3 - IMU_OUTPUT_SYNC";
    param_desc.additional_constraints = long_param_desc.str();
    long_param_desc.clear();

    param_int_range.from_value = IMU_INTERNAL_CLOCK;
    param_int_range.step = 1;
    param_int_range.to_value = IMU_OUTPUT_SYNC;
    param_desc.integer_range.push_back(param_int_range);

    node_->declare_parameter(param_desc.name, static_cast<int>(IMU_SYNC), param_desc);
    imu_sync_clk_mode_ = node_->get_parameter(param_desc.name).as_int();
    RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded parameter imu_sync_clk_mode: "
      <<imu_sync_clk_mode_);
    param_desc.integer_range.clear();
    /********************************************************************************/
    
    /************************ IMU SYNC CLOCK LINE PARAMETER *************************/
    param_desc.name = "imu_sync_clk_line";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "Set IMU Sync Clock Line";
    param_desc.additional_constraints = "0 - IMU_DIO1; 1 - IMU_DIO2; 2 - IMU_DIO3; 3 - IMU_DIO4";

    param_int_range.from_value = IMU_DIO1;
    param_int_range.step = 1;
    param_int_range.to_value = IMU_DIO4;
    param_desc.integer_range.push_back(param_int_range);
    
    node_->declare_parameter(param_desc.name, static_cast<int>(IMU_DIO1), param_desc);
    imu_sync_clk_line_ = node_->get_parameter(param_desc.name).as_int();
    RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded parameter imu_sync_clk_line: "
      <<imu_sync_clk_line_);
    param_desc.integer_range.clear();
    /********************************************************************************/

    /********************** IMU SYNC CLOCK POLARITY PARAMETER ***********************/
    param_desc.name = "imu_sync_clk_pol";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
    param_desc.description = "Set Polarity of IMU Sync Clock";
    param_desc.additional_constraints = "0 - IMU_NEG_POLARITY; 1 - IMU_POS_POLARITY";

    param_int_range.from_value = IMU_NEG_POLARITY;
    param_int_range.step = 1;
    param_int_range.to_value = IMU_POS_POLARITY;
    param_desc.integer_range.push_back(param_int_range);
    
    node_->declare_parameter("imu_sync_clk_pol", static_cast<int>(IMU_POS_POLARITY), param_desc);
    imu_sync_clk_pol_= node_->get_parameter("imu_sync_clk_pol").as_int();
    RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded parameter imu_sync_clk_pol: "
      <<imu_sync_clk_pol_);
    param_desc.integer_range.clear();
    /********************************************************************************/
  }
  /****************** ENABLE IMU LINEAR G COMPENSATION PARAMETER ********************/
  param_desc.name = "enable_imu_lin_g_comp";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Determine if IMU Linear g Compensation will be enabled";
  param_desc.additional_constraints.clear();
  
  node_->declare_parameter(param_desc.name, false, param_desc);
  b_imu_linear_g_comp_ = node_->get_parameter(param_desc.name).as_bool();
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_imu_linear_g_comp_, 
    "[AdiImuRos2] Loaded parameter enable_imu_lin_g_comp: true");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), !b_imu_linear_g_comp_, 
    "[AdiImuRos2] Loaded parameter enable_imu_lin_g_comp: false");
  /********************************************************************************/
  
  /************* ENABLE IMU POINT OF PERCUSSION ALIGNMENT PARAMETER **************/
  param_desc.name = "enabel_imu_pp_align";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Determine if IMU Point of Percussion Alignment will be enabled";
  param_desc.additional_constraints.clear();

  node_->declare_parameter(param_desc.name, false, param_desc);
  b_imu_pp_align_ = node_->get_parameter(param_desc.name).as_bool();
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_imu_pp_align_, 
    "[AdiImuRos2] Loaded parameter enable_imu_pp_align: true");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), !b_imu_pp_align_, 
    "[AdiImuRos2] Loaded parameter enable_imu_pp_align: false");
  /********************************************************************************/

  //Bias Correction is not supported by 16500
  if (ADIS16500 != imu_prod_id_) 
  {
    /**************** ENABLE UPDATE IMU BIAS CORRECTION PARAMETER *****************/
    param_desc.name = "update_imu_bias_corr";
    param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
    param_desc.description = "Determine if IMU Bias Correction will be performed.";
    param_desc.additional_constraints.clear();

    node_->declare_parameter(param_desc.name, false, param_desc);
    b_imu_update_bias_corr_ = node_->get_parameter(param_desc.name).as_bool();
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_imu_update_bias_corr_, 
      "[AdiImuRos2] Loaded parameter update_imu_bias_corr: true");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), !b_imu_update_bias_corr_, 
      "[AdiImuRos2] Loaded parameter update_imu_bias_corr: false");
    /********************************************************************************/

    if (b_imu_update_bias_corr_)
    {
      /********************** IMU TIME BASE CONTROL PARAMETER ***********************/
      param_desc.name = "imu_time_base_control";
      param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER;
      param_desc.description = "IMU Time Base Control";
      param_desc.additional_constraints.clear();

      param_int_range.from_value = TIME_BASE_MIN;
      param_int_range.step = 1;
      param_int_range.to_value = TIME_BASE_MAX;
      param_desc.integer_range.push_back(param_int_range);

      node_->declare_parameter(param_desc.name, static_cast<int>(TIME_BASE_CTRL_DEFAULT),
        param_desc);
      imu_time_base_control_= node_->get_parameter(param_desc.name).as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded parameter imu_time_base_control: " << imu_time_base_control_);
      param_desc.integer_range.clear();
      /********************************************************************************/

      /*********** ENABLE IMU Z-AXIS ACCELERATION BIAS CORRECTION PARAMETER ***********/
      param_desc.name = "enable_imu_accl_z_bias_null";
      param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
      param_desc.description = "Enable/Disable Z-axis IMU Acceleration Bias Correction.";
      param_desc.additional_constraints.clear();

      node_->declare_parameter(param_desc.name, static_cast<int>(IMU_DISABLE),
      param_desc);
      imu_accl_z_bias_null_= node_->get_parameter(param_desc.name).as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded parameter enable_imu_accl_z_bias_null: " << imu_accl_z_bias_null_);
      /********************************************************************************/

      /*********** ENABLE IMU Y-AXIS ACCELERATION BIAS CORRECTION PARAMETER ***********/
      param_desc.name = "enable_imu_accl_y_bias_null";
      param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
      param_desc.description = "Enable/Disable Y-axis IMU Acceleration Bias Correction";
      param_desc.additional_constraints.clear();

      node_->declare_parameter(param_desc.name, static_cast<int>(IMU_DISABLE),
        param_desc);
      imu_accl_y_bias_null_= node_->get_parameter(param_desc.name).as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded parameter enable_imu_accl_y_bias_null: " << imu_accl_y_bias_null_);
      /********************************************************************************/
      
      /*********** ENABLE IMU X-AXIS ACCELERATION BIAS CORRECTION PARAMETER ***********/
      param_desc.name = "enable_imu_accl_x_bias_null";
      param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
      param_desc.description = "Enable/Disable X-axis IMU Acceleration Bias Correction";
      param_desc.additional_constraints.clear();

      node_->declare_parameter(param_desc.name, static_cast<int>(IMU_DISABLE),
        param_desc);
      imu_accl_x_bias_null_= node_->get_parameter(param_desc.name).as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded parameter enable_imu_accl_x_bias_null: " << imu_accl_x_bias_null_);
      /********************************************************************************/

      /************* ENABLE IMU Z-AXIS GYROSCOPE BIAS CORRECTION PARAMETER ************/
      param_desc.name = "enable_imu_gyro_z_bias_null";
      param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
      param_desc.description = "Enable/Disable Z-axis IMU Gyroscope Bias Correction";
      param_desc.additional_constraints.clear();

      node_->declare_parameter(param_desc.name, static_cast<int>(IMU_ENABLE),
        param_desc);
      imu_gyro_z_bias_null_= node_->get_parameter(param_desc.name).as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded parameter enable_imu_gyro_z_bias_null: " <<imu_gyro_z_bias_null_);
      /********************************************************************************/

      /************* ENABLE IMU Y-AXIS GYROSCOPE BIAS CORRECTION PARAMETER ************/
      param_desc.name = "enable_imu_gyro_y_bias_null";
      param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
      param_desc.description = "Enable/Disable Y-axis IMU Gyroscope Bias Correction";
      param_desc.additional_constraints.clear();
      
      node_->declare_parameter(param_desc.name, static_cast<int>(IMU_ENABLE),
        param_desc);
      imu_gyro_y_bias_null_= node_->get_parameter(param_desc.name).as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded parameter enable_imu_gyro_y_bias_null: " <<imu_gyro_y_bias_null_);
      /********************************************************************************/
      
      /************* ENABLE IMU X-AXIS GYROSCOPE BIAS CORRECTION PARAMETER ************/
      param_desc.name = "enable_imu_gyro_x_bias_null";
      param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
      param_desc.description = "Enable/Disable X-axis IMU Gyroscope Bias Correction";
      param_desc.additional_constraints.clear();

      node_->declare_parameter(param_desc.name, static_cast<int>(IMU_ENABLE),
        param_desc);
      imu_gyro_x_bias_null_= node_->get_parameter(param_desc.name).as_int();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded parameter enable_imu_gyro_x_bias_null: " <<imu_gyro_x_bias_null_);
      /********************************************************************************/
    }
  }
  /****************** ENABLE IMU ACCELEROMETER BIAS UPDATE PARAMETER ******************/
  param_desc.name = "update_imu_accl_bias";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Determine if IMU Accelerometer Bias will be set";
  param_desc.additional_constraints.clear();
  
  node_->declare_parameter(param_desc.name, false, param_desc);
  b_update_imu_accl_bias_= node_->get_parameter(param_desc.name).as_bool();
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_update_imu_accl_bias_, 
    "[AdiImuRos2] Loaded parameter update_imu_accl_bias: true");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), !b_update_imu_accl_bias_, 
    "[AdiImuRos2] Loaded parameter update_imu_accl_bias: false");
  /********************************************************************************/
  if (b_update_imu_accl_bias_)
  {
    /********************* IMU ACCELEROMETER BIAS PARAMETER ***********************/
    param_desc.name = "imu_accl_bias";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    param_desc.description = "Set values for IMU Accelerometer Bias";
    param_desc.additional_constraints.clear();

    node_->declare_parameter<std::vector<long>>(param_desc.name, imu_accl_bias_, param_desc);
    imu_accl_bias_ = node_->get_parameter(param_desc.name).as_integer_array();
    RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded vector parameter imu_accl_bias: "
      << imu_accl_bias_[0] << ", " << imu_accl_bias_[1] << ", " << imu_accl_bias_[2]);
    /********************************************************************************/
  }
  /********************* ENABLE IMU GYROSCOPE BIAS PARAMETER ***********************/
  param_desc.name = "update_imu_gyro_bias";
  param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
  param_desc.description = "Determine if IMU Gyroscope Bias will be set";
  param_desc.additional_constraints.clear();

  node_->declare_parameter(param_desc.name, false, param_desc);
  b_update_imu_gyro_bias_= node_->get_parameter(param_desc.name).as_bool();
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_update_imu_gyro_bias_,
    "[AdiImuRos2] Loaded parameter update_imu_gyro_bias: true");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), !b_update_imu_gyro_bias_,
    "[AdiImuRos2] Loaded parameter update_imu_gyro_bias: false");
  /********************************************************************************/

  if (b_update_imu_gyro_bias_)
  {
    /********************* IMU ACCELEROMETER BIAS PARAMETER ***********************/
    param_desc.name = "imu_gyro_bias";
    param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
    param_desc.description = "Set values for IMU Gyroscope Bias";
    param_desc.additional_constraints.clear();
    
    node_->declare_parameter<std::vector<long>>(param_desc.name, imu_gyro_bias_, param_desc);
    imu_gyro_bias_ = node_->get_parameter(param_desc.name).as_integer_array();
    RCLCPP_INFO_STREAM(node_->get_logger(), "[AdiImuRos2] Loaded vector parameter imu_gyro_bias: "
      << imu_gyro_bias_[0] << ", " << imu_gyro_bias_[1] << ", " << imu_gyro_bias_[2]);
    /********************************************************************************/
  }
  //Accl/Gyro Scale Adjustment is only available in 16495
  if (ADIS16495 == imu_prod_id_)
  {
    /****************** ENABLE IMU ACCELEROMETER SCALE PARAMETER ********************/
    param_desc.name = "update_imu_accl_scale";
    param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
    param_desc.description = "Determine if IMU Accelerometer Scale Adjustment will be enabled";
    param_desc.additional_constraints.clear();
    
    node_->declare_parameter(param_desc.name, false, param_desc);
    b_update_imu_accl_scale_ = node_->get_parameter(param_desc.name).as_bool();
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_update_imu_accl_scale_,
      "[AdiImuRos2] Loaded parameter update_imu_accl_scale: true");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), !b_update_imu_accl_scale_,
      "[AdiImuRos2] Loaded parameter update_imu_accl_scale: false");
    /********************************************************************************/
    if (b_update_imu_accl_scale_)
    {
      /********************** IMU ACCELEROMETER SCALE PARAMETER ***********************/
      param_desc.name = "imu_accl_scale";
      param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
      param_desc.description = "Set values for IMU Accelerometer scale";
      param_desc.additional_constraints.clear();
      
      node_->declare_parameter<std::vector<long>>("imu_accl_scale", imu_accl_scale_, param_desc);
      imu_accl_scale_ = node_->get_parameter("imu_accl_scale").as_integer_array();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded vector parameter imu_accl_scale: " <<imu_accl_scale_[0] <<
        ", " << imu_accl_scale_[1] << ", " << imu_accl_scale_[2]);
      /********************************************************************************/
    }
    /******************* ENABLE IMU GYROSCOPE SCALE PARAMETER ***********************/
    param_desc.name = "update_imu_gyro_scale";
    param_desc.type = rclcpp::ParameterType::PARAMETER_BOOL;
    param_desc.description = "Determine if IMU Gyroscope Scale Adjustment will be enabled";
    param_desc.additional_constraints.clear();

    node_->declare_parameter(param_desc.name, false, param_desc);
    b_update_imu_gyro_scale_ = node_->get_parameter(param_desc.name).as_bool();
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_update_imu_gyro_scale_, 
      "[AdiImuRos2] Loaded parameter update_imu_gyro_scale: true");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), !b_update_imu_gyro_scale_, 
      "[AdiImuRos2] Loaded parameter update_imu_gyro_scale: false");
    /********************************************************************************/

    if (b_update_imu_gyro_scale_)
    {
      /******************* ENABLE IMU GYROSCOPE SCALE PARAMETER ***********************/
      param_desc.name = "imu_gyro_scale";
      param_desc.type = rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY;
      param_desc.description = "Set values for IMU Gyroscope Scale";
      param_desc.additional_constraints.clear();
      
      node_->declare_parameter<std::vector<long>>(param_desc.name, imu_gyro_scale_, param_desc);
      imu_gyro_scale_ = node_->get_parameter(param_desc.name).as_integer_array();
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Loaded vector parameter imu_gyro_scale: " << imu_gyro_scale_[0] <<
        ", " << imu_gyro_scale_[1] << ", "<<imu_gyro_scale_[2]);
      /********************************************************************************/
    }
  }
  
  //Update the adi_imu_Device_t
  p_device_->prodId=imu_prod_id_;
  p_device_->g=gravity_;
  p_device_->dataFormat=static_cast<adi_imu_DataFormat_e>(imu_data_format_);

  RCLCPP_INFO(node_->get_logger(), "[AdiImuRos2] Loaded Parameters");
}

bool AdiImuRos2::init(void)
{
  bool b_success = false;
  uint8_t retry_id_check = 0;
  int ret = -1;

  //Initialize IMU
  while ((retry_id_check < 3))
  {
    ret = adi_imu_Init(p_device_);

    if (Err_imu_ProdIdVerifyFailed_e == ret)
    {
      retry_id_check++;
      RCLCPP_WARN_STREAM(node_->get_logger(), "[AdiImuRos2] ID Check has failed." <<
        "Retrying IMU initialization...");
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (retry_id_check == 3), 
        "[AdiImuRos2] Max retries reached.");
    }
    else
    {
      break;
    }
  }
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
    "[AdiImuRos2] ADI IMU device initialization was unsuccessful. ERROR CODE: " << ret << 
    " Kindly refer to adi_imu_Error_e");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
    "[AdiImuRos2] ADI IMU device initialization successful.");
  b_success = (Err_imu_Success_e <= ret)? true:false;

  return b_success;
}

bool AdiImuRos2::config(void)
{
  bool b_success = false;
  int ret = -1;

  //Constants from Data Sheet regarding the frequency of the data.
  const int nominal_sample_rate_1 = 2000;
  const int nominal_sample_rate_2 = 4250;

  //Set Output Data Rate
  ret = adi_imu_SetOutputDataRate(p_device_, imu_data_rate_);
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
    "[AdiImuRos2] Setting IMU Data Rate was unsuccessful. ERROR CODE: " << ret << 
    " Kindly refer to adi_imu_Error_e");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
    "[AdiImuRos2] Setting IMU Data Rate successful.");
  b_success = (Err_imu_Success_e <= ret)? true:false;

  //Configure and Enable Data Ready
  if (b_success)
  {
    if (ADIS16495 == p_device_->prodId)
    {
      ret = adi_imu_ConfigDataReady(p_device_, static_cast<adi_imu_GPIO_e>(imu_data_rdy_line_),
        static_cast<adi_imu_Polarity_e>(imu_data_rdy_pol_));
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[AdiImuRos2] Configuring Data Ready was unsuccessful. ERROR CODE: " << ret << 
        " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
        "[AdiImuRos2] Configuring Data Ready successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;

      ret = adi_imu_SetDataReady(p_device_, IMU_ENABLE);
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[AdiImuRos2] Enabling Data Ready was unsuccessful. ERROR CODE: " << ret << 
        " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
        "[AdiImuRos2] Enabling Data Ready successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;
    }
    else if ((ADIS16470 == p_device_->prodId) || (ADIS16500 == p_device_->prodId))
    {
      ret = adi_imu_ConfigDataReady(p_device_, static_cast<adi_imu_GPIO_e>(IMU_NULL),\
      static_cast<adi_imu_Polarity_e>(imu_data_rdy_pol_));
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[AdiImuRos2] Configuring Data Ready was unsuccessful. ERROR CODE: " << ret << 
        " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
        "[AdiImuRos2] Configuring Data Ready successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;

      RCLCPP_INFO_STREAM(node_->get_logger(),"[AdiImuRos2] Data Ready is constantly enabled in " <<
        p_device_->prodId);
    }
  }

  //Configure and Enable Sync Clock
  if (b_success && b_imu_sync_clk_)
  {
    if (ADIS16495 == p_device_->prodId)
    {
      
      ret = adi_imu_ConfigSyncClkMode(p_device_, 
        static_cast<adi_imu_ClockMode_e>(imu_sync_clk_mode_), IMU_ENABLE,
        static_cast<adi_imu_EdgeType_e>(imu_sync_clk_pol_), 
        static_cast<adi_imu_GPIO_e>(imu_sync_clk_line_));
    }
    else if ((ADIS16470 == p_device_->prodId) || (ADIS16500 == p_device_->prodId))
    {
      ret = adi_imu_ConfigSyncClkMode(p_device_,
        static_cast<adi_imu_ClockMode_e>(imu_sync_clk_mode_),
        static_cast<adi_imu_EnDis_e>(IMU_NULL),
        static_cast<adi_imu_EdgeType_e>(imu_sync_clk_pol_),
        static_cast<adi_imu_GPIO_e>(IMU_NULL));
    }
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[AdiImuRos2] Configuration of Sync Clk was unsuccessful. ERROR CODE: " << ret <<
      " Kindly refer to adi_imu_Error_e");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
      "[AdiImuRos2] Configuration of Sync Clk successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }

  //Set Linear G Compensation
  if (b_success)
  {
    adi_imu_EnDis_e val = (b_imu_linear_g_comp_)? IMU_ENABLE:IMU_DISABLE;

    ret = adi_imu_SetLineargComp(p_device_, val);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
      "[AdiImuRos2] Setting IMU Linear g Comp was unsuccessful. ERROR CODE: " << ret <<
      " Kindly refer to adi_imu_Error_e");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[AdiImuRos2] Setting IMU Linear g Comp successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }

  //Set Point of Percussion Alignment
  if (b_success)
  {
    adi_imu_EnDis_e val = (b_imu_pp_align_)? IMU_ENABLE:IMU_DISABLE;

    ret = adi_imu_SetPPercAlignment(p_device_, val);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[AdiImuRos2] Setting IMU Point of Percussion (PP) was unsuccessful. ERROR CODE: " << ret <<
      " Kindly refer to adi_imu_Error_e");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[AdiImuRos2] Setting IMU POINT of Percussion (PP) successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;
  }
    
  if ((ADIS16495 == p_device_->prodId) || (ADIS16470 == p_device_->prodId))
  {
    //Configure CBE
    if (b_success && b_imu_update_bias_corr_)
    {
      //Not declared in header
      ret = adi_imu_ConfigBiasCorrectionTime(p_device_, imu_time_base_control_); 
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[AdiImuRos2] Configuring the IMU Bias Correction Accumulation Time was unsuccessful." <<
        "ERROR CODE: " << ret << " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
        "[AdiImuRos2] Configuring the IMU Bias Correction Accumulation Time successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;

      //Trigger Bias Correction Update
      ret = adi_imu_SelectBiasConfigAxes(p_device_, 
        static_cast<adi_imu_EnDis_e>(imu_gyro_x_bias_null_),
        static_cast<adi_imu_EnDis_e>(imu_gyro_y_bias_null_),
        static_cast<adi_imu_EnDis_e>(imu_gyro_z_bias_null_),
        static_cast<adi_imu_EnDis_e>(imu_accl_x_bias_null_),
        static_cast<adi_imu_EnDis_e>(imu_accl_y_bias_null_),
        static_cast<adi_imu_EnDis_e>(imu_accl_z_bias_null_));
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
        "[AdiImuRos2] Configuring the IMU Sensors to be nulled was unsuccessful. ERROR CODE: " <<
        ret << " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
        "[AdiImuRos2] Configuring the IMU Sensors to be nulled successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;

      //Setting Accelerometer Bias (*_ACCL_BIAS Register)
      ret = adi_imu_UpdateBiasCorrection(p_device_);
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
        "[AdiImuRos2] Triggering the IMU Bias Correction was unsuccessful. ERROR CODE: " << ret <<
        " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
        "[AdiImuRos2] Triggering the IMU Bias Correction successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;
    }
  }

  if (b_success && b_update_imu_accl_bias_)
  {
    adi_imu_AcclBiasRaw32_t accl_bias;
    accl_bias.x = static_cast<int32_t>(imu_accl_bias_[0]);
    accl_bias.y = static_cast<int32_t>(imu_accl_bias_[1]);
    accl_bias.z = static_cast<int32_t>(imu_accl_bias_[2]);

    ret = adi_imu_SetAcclBias(p_device_, accl_bias);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[AdiImuRos2] Configuring the IMU Accl Bias was unsuccessful. ERROR CODE: " << ret <<
      " Kindly refer to adi_imu_Error_e");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
      "[AdiImuRos2] Configuring the IMU Accl Bias successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

    ret = adi_imu_GetAcclBias(p_device_, &accl_bias);
    b_success = (Err_imu_Success_e <= ret)? true:false;
    if (!b_success) return ret;
    RCLCPP_INFO_STREAM(node_->get_logger(), 
      "[AdiImuRos2] Results of reading the Accelerometer Bias: x - " << accl_bias.x << ", y - " <<
      accl_bias.y << ", z: " << accl_bias.z);
  }

  //Setting Gyroscope Bias (*_GYRO_BIAS Register)
  if (b_success && b_update_imu_gyro_bias_)
  {
    adi_imu_GyroBiasRaw32_t gyro_bias;
    gyro_bias.x = static_cast<int32_t>(imu_gyro_bias_[0]);
    gyro_bias.y = static_cast<int32_t>(imu_gyro_bias_[1]);
    gyro_bias.z = static_cast<int32_t>(imu_gyro_bias_[2]);

    ret = adi_imu_SetAcclBias(p_device_, gyro_bias);
    RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
      "[AdiImuRos2] Configuring the IMU Gyro Bias was unsuccessful. ERROR CODE: " << ret <<
      " Kindly refer to adi_imu_Error_e");
    RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
      "[AdiImuRos2] Configuring the IMU Gyro Bias successful.");
    b_success = (Err_imu_Success_e <= ret)? true:false;

    ret = adi_imu_GetAcclBias(p_device_, &gyro_bias);
    b_success = (Err_imu_Success_e <= ret)? true:false;
    if (!b_success) return ret;
    RCLCPP_INFO_STREAM(node_->get_logger(), 
      "[AdiImuRos2] Results of reading the Gyroscope Bias: x - " << gyro_bias.x << ", y - " <<
      gyro_bias.y << ", z: " << gyro_bias.z);
  }
    
  if (ADIS16495 == p_device_->prodId)
  {
    //Setting Accelerometer Scale(*_ACCL_SCALE Register)
    if (b_success && b_update_imu_accl_scale_)
    {
      adi_imu_AcclScale_t accl_scale;
      accl_scale.x = static_cast<int16_t>(imu_accl_scale_[0]);
      accl_scale.y = static_cast<int16_t>(imu_accl_scale_[1]);
      accl_scale.z = static_cast<int16_t>(imu_accl_scale_[2]);

      ret = adi_imu_SetAcclScale(p_device_, accl_scale);
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[AdiImuRos2] Configuring the IMU Accelerometer Scale was unsuccessful. ERROR CODE: " <<
        ret << " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
      "[AdiImuRos2] Configuring the IMU Accelerometer Scale successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;

      ret = adi_imu_GetAcclScale(p_device_, &accl_scale);
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[AdiImuRos2] Acquiring IMU Accelerometer Scale was unsuccessful. ERROR CODE: " <<
        ret << " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
        "[AdiImuRos2] Acquiring IMU Accelerometer Scale successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;
      
      if (!b_success) return ret;
      
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Results of reading the Gyroscope Bias: x - "<< accl_scale.x << ", y - " <<
        accl_scale.y << ", z: " << accl_scale.z);
    }

    //Setting Gyroscope Scale(*_GYRO_SCALE Register)
    if (b_success && b_update_imu_gyro_scale_)
    {
      adi_imu_GyroScale_t gyro_scale;
      gyro_scale.x = static_cast<int32_t>(imu_gyro_scale_[0]);
      gyro_scale.y = static_cast<int32_t>(imu_gyro_scale_[1]);
      gyro_scale.z = static_cast<int32_t>(imu_gyro_scale_[2]);
      
      ret = adi_imu_SetGyroScale(p_device_, gyro_scale);
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
        "[AdiImuRos2] Configuring the IMU Gyro Scale was unsuccessful. ERROR CODE: " << ret <<
        " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
        "[AdiImuRos2] Configuring the IMU Gyro Scale successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;

      ret = adi_imu_GetGyroScale(p_device_, &gyro_scale);
      b_success = (Err_imu_Success_e <= ret)? true:false;
      if (!b_success) return ret;

      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[AdiImuRos2] Results of reading the Gyroscope Bias: x - "<< gyro_scale.x << ", y - " <<
        gyro_scale.y << ", z: " << gyro_scale.z);
    }
  }
    
  if (ADIS16500 == p_device_->prodId)
  {
    if (b_success)
    {
      //Set IMU Burst Data Format
      ret = adi_imu_ConfigBurstDataFormat(p_device_, p_device_->dataFormat);
      RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret), 
        "[AdiImuRos2] Configuring the Burst Data Format was unsuccessful. ERROR CODE: " << ret <<
        " Kindly refer to adi_imu_Error_e");
      RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret), 
        "[AdiImuRos2] Configuring the Burst Data Format successful.");
      b_success = (Err_imu_Success_e <= ret)? true:false;
    }
  }
  
  //Read and print IMU device info and config
  adi_imu_DevInfo_t imuInfo;
  ret = adi_imu_GetDevInfo(p_device_, &imuInfo);
  b_success = (Err_imu_Success_e <= ret)? true:false;
  
  if (b_success)
  {
    ret = adi_imu_PrintDevInfo(p_device_, &imuInfo);
    b_success = (Err_imu_Success_e <= ret)? true:false;
    if (!b_success)
    {
      return b_success;
    }
  }
  else
  {
    return b_success;
  }

  if ((ADIS16470 == imu_prod_id_) || (ADIS16500 == imu_prod_id_))
  {
    actual_imu_data_rate_ = nominal_sample_rate_1 / (imuInfo.decimationRate + DEC_RATE_CONST);
  }
  else if (ADIS16495 == imu_prod_id_ )
  {
    actual_imu_data_rate_ = nominal_sample_rate_2 / (imuInfo.decimationRate + DEC_RATE_CONST);
  }
  //for future IMU device
  /*else
  {   
  }*/

  RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), (actual_imu_data_rate_ != imu_data_rate_),
    "[AdiImuRos2] Actual IMU Rate set: ~" << actual_imu_data_rate_ << " Hz");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (actual_imu_data_rate_ == imu_data_rate_),
    "[AdiImuRos2] Actual IMU Rate set: ~" << actual_imu_data_rate_ << " Hz");

  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), b_success, 
    "[AdiImuRos2] Configuration process successful.");
  return b_success;
}

double AdiImuRos2::getImuDataRateHz(void)
{
  return actual_imu_data_rate_;
}

bool AdiImuRos2::triggerBiasCorrectionUpdate(void)
{
  bool b_success = false;
  int ret = -1;

  //Trigger Bias Correction Update
  RCLCPP_INFO(node_->get_logger(), "[AdiImuRos2] Trigger IMU Bias Correction Update");
  ret = adi_imu_UpdateBiasCorrection(p_device_);
  
  RCLCPP_WARN_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e > ret),
  "[AdiImuRos2] Triggering the IMU Bias Correction was unsuccessful. ERROR CODE: " << ret << 
  " Kindly refer to adi_imu_Error_e");
  RCLCPP_INFO_STREAM_EXPRESSION(node_->get_logger(), (Err_imu_Success_e <= ret),
  "[AdiImuRos2] Triggering the IMU Bias Correction successful.");
  b_success = (Err_imu_Success_e <= ret)? true:false;

  return b_success;
}
