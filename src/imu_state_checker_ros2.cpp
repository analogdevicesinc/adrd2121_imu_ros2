/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#include "imu_state_checker_ros2.hpp"

ImuStateCheckerRos2::ImuStateCheckerRos2(const rclcpp::Node::SharedPtr& node):
node_(node)
{
  //Object initialization
  RCLCPP_INFO(node_->get_logger(), "[IMU State Check] Constructed");
}

ImuStateCheckerRos2::~ImuStateCheckerRos2()
{
  //Object destruction
  RCLCPP_INFO(node_->get_logger(), "[IMU State Check] Destructed");
  node_ = nullptr;
}

bool ImuStateCheckerRos2::init(void)
{
  RCLCPP_DEBUG(node_->get_logger(), "[IMU State Check] Initializing buffer board for state check");
  bool b_success = false;

  b_success = this->loadParams();

  //Load Parameters
  if (b_success)
  {
    //Subscribe to topic with AdiImu type
    if(SENSOR_MSGS_IMU == imu_msg_type_)
    {
      adi_imu_sub_ = node_->create_subscription<adrd2121_imu::msg::AdiImu>(
        imu_topic_name_, DATA_LIMIT, std::bind(static_cast<void(ImuStateCheckerRos2::*)
        (const adrd2121_imu::msg::AdiImu::SharedPtr)>
        (&ImuStateCheckerRos2::imuCallback), this, _1));
      
      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[IMU State Check] Bias Estimate node is now subscribed to " << imu_topic_name_ <<
        " with interface adrd2121_imu/AdiImu");
    }
    //Subscribe to topic with sensor_msgs type
    else if (ADI_IMU_MSG == imu_msg_type_)
    {
      sensor_imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_name_, DATA_LIMIT, std::bind(static_cast<void(ImuStateCheckerRos2::*)
        (const sensor_msgs::msg::Imu::SharedPtr)>(&ImuStateCheckerRos2::imuCallback), this, _1));

      RCLCPP_INFO_STREAM(node_->get_logger(), 
        "[IMU State Check] Bias Estimate node is now subscribed to "<<imu_topic_name_<<
        " with interface adrd2121_imu/AdiImu");
    }
    publish_imu_state_ = node_->create_publisher<adrd2121_imu::msg::ImuState>
      ("imu_state", DATA_LIMIT);

    RCLCPP_INFO_STREAM(node_->get_logger(), "[IMU State Check] ImuState published.");
    prev_state_ = MOVING;
  }
  return b_success;
}

//Callback for sensor_msgs
void ImuStateCheckerRos2::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  state_ = this->evaluateState(msg);
  this->evaluateStandstillBegin();
}

//Callback for AdiImu topic
void ImuStateCheckerRos2::imuCallback(const adrd2121_imu::msg::AdiImu::SharedPtr msg)
{
  sensor_msgs::msg::Imu::SharedPtr imu;
  imu->angular_velocity = msg->angular_velocity;
  imu->linear_acceleration = msg->linear_acceleration;
  state_ = this->evaluateState(imu);
  this->evaluateStandstillBegin();
}

//Checking if the IMU is moving or not 
e_imu_state ImuStateCheckerRos2::evaluateState(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  bool gyro_standstill = false;
  bool accl_standstill = false;
  e_imu_state state;
  float gyro_std = RESET;
  float accl_std = RESET;

  //Acquiring gyro Magnitude 
  float gyro_mag = sqrt(
    (imu_msg->angular_velocity.x * imu_msg->angular_velocity.x) + 
    (imu_msg->angular_velocity.y * imu_msg->angular_velocity.y) + 
    (imu_msg->angular_velocity.z * imu_msg->angular_velocity.z));
  //Acquiring accl Magnitude
  float accl_mag = sqrt(
    (imu_msg->linear_acceleration.x * imu_msg->linear_acceleration.x) + 
    (imu_msg->linear_acceleration.y * imu_msg->linear_acceleration.y) +
    (imu_msg->linear_acceleration.z * imu_msg->linear_acceleration.z));

  //Make sure only the last 10 magnitude are stored in the deque
  if(BUFFER_LIMIT < gyro_mag_q_.size())
  {
    gyro_mag_q_.pop_front();
  }
  
  gyro_mag_q_.push_back(gyro_mag);

  if(BUFFER_LIMIT < accl_mag_q_.size())
  {
    accl_mag_q_.pop_front();
  }
  
  accl_mag_q_.push_back(accl_mag);

  //Calculate the standard deviation of gyro and acc magnitude
  gyro_std = this->getStandardDev(gyro_mag_q_);
  accl_std = this->getStandardDev(accl_mag_q_);

  //Note: If the Standard deviationare less than threshold, then assume standstill
  //These threshold are manually tested;
  //The lower the value, the more sensitive (i.e slight IMU movement may be considered as MOVING)
  if (gyro_std < gyro_std_thresh_)
  {
    gyro_standstill = true;
  }
  if (accl_std < accl_std_thresh_)
  {
    accl_standstill = true;
  }
  state = (accl_standstill && gyro_standstill)? STANDSTILL:MOVING;

  return state;
}

float ImuStateCheckerRos2::getStandardDev(std::deque<float> data)
{
  int size_data = data.size();
  int counter = RESET;
  float sum = RESET;
  float mean = RESET;
  float var = RESET;
  float std = RESET;

  //Get Sum of data
  for(counter = RESET; (counter < size_data); counter++)
  {
    sum+=data[counter];
  }
  //Get Mean of data
  mean = sum/size_data;

  //Get variance
  for(counter = RESET; (counter < size_data); counter++)
  {
    var+=pow(data[counter] - mean, TWICE);
  }

  //Get standard deviation
  std = sqrt(var/size_data);
  return std;    
}

void ImuStateCheckerRos2::evaluateStandstillBegin(void)
{
  adrd2121_imu::msg::ImuState imu_state;
  std::ostringstream ss;
  rclcpp::Clock clock; 
  rclcpp::Time current_time = clock.now();
  
  std::string s_prev_state, s_state;
  if((MOVING == prev_state_) && (STANDSTILL == state_))
  {
    RCLCPP_DEBUG(node_->get_logger(), "IMU STATE: Now transitioning to standstill state.");
    standstill_begin_ = clock.now();
  }

  
  s_prev_state = (MOVING == prev_state_)? "MOVING":"STANDSTILL";
  s_state = (MOVING == state_)? "MOVING":"STANDSTILL";
  ss << "Previous State: " << s_prev_state << " to Current State: " << s_state;

  //Set current state as previous state
  prev_state_ = state_;
  
  //Update and publish topic
  imu_state.header.stamp = current_time;
  imu_state.message = ss.str();
  imu_state.state = s_state;
  rclcpp::Duration standstill_time = current_time - standstill_begin_;
  imu_state.stand_still_secs = standstill_time.seconds();
  publish_imu_state_->publish(imu_state);  
}

rclcpp::Time ImuStateCheckerRos2::getStandstillBegin(void)
{
  return standstill_begin_;
}

e_imu_state ImuStateCheckerRos2::getState(void)
{
  return state_;
}

bool ImuStateCheckerRos2::loadParams(void)
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  const double gyro_thresh = 0.02;
  const double accl_thresh = 0.08;
  param_desc.read_only = true;
  bool b_success = false;
  int repeat = RESET;

  RCLCPP_INFO(node_->get_logger(), "Currently loading IMU State Checker Parameters");

  //Default values were based on manual testing
  node_->declare_parameter("gyro_std_thresh", static_cast<double>(gyro_thresh), param_desc);
  gyro_std_thresh_ = node_->get_parameter("gyro_std_thresh").as_double();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[IMU State Check] gyro_std_thresh: " <<
    gyro_std_thresh_);

  node_->declare_parameter("accl_std_thresh", static_cast<double>(accl_thresh), param_desc);
  accl_std_thresh_ = node_->get_parameter("accl_std_thresh").as_double();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[IMU State Check] accl_std_thresh: " <<
    accl_std_thresh_);

  node_->declare_parameter("imu_topic_name", "/imu/data_raw", param_desc);
  imu_topic_name_ = node_->get_parameter("imu_topic_name").as_string();
  RCLCPP_INFO_STREAM(node_->get_logger(), "[IMU State Check] Subscribed to IMU Topic: " <<
    imu_topic_name_);

  //Acquiring the topics
  while (!b_success && (WAIT_LIMIT > repeat))
  {
    RCLCPP_INFO_STREAM(node_->get_logger(), "[IMU State Check] Waiting for topics to be published");
    std::map<std::string,std::vector<std::string>>topic_infos = node_->get_topic_names_and_types();
    //Only needed 3 secs here since it loops for 2 more times if the topic needed is not found yet.
    rclcpp::sleep_for(std::chrono::seconds(WAIT_LIMIT));  
    for (std::pair<const std::string, std::vector<std::string, std::allocator<std::string>>> 
      topic_it : topic_infos)
    {
      std::string topic_name = topic_it.first;
      std::vector<std::string> topic_types = topic_it.second;

      //Check if acquired topic name is the same with published topic_name
      if (imu_topic_name_.compare(topic_name) == 0)
      {
        //Check type of topic
        for (std::string topic_type : topic_types)
        {
          if (topic_type.compare("adrd2121_imu/msg/AdiImu"))
          {
            imu_msg_type_ = ADI_IMU_MSG;
          }
          else if (topic_type.compare("sensor_msgs/msg/Imu"))
          {
            imu_msg_type_ = SENSOR_MSGS_IMU;
          }
        }
        b_success = true;
      }
    }
  repeat++;
  }
  RCLCPP_ERROR_STREAM_EXPRESSION(node_->get_logger(), !b_success, 
    "[IMU State Check] Cannot find topic " << imu_topic_name_ <<
    " with configuration of adrd2121_imu/msg/AdiImu or sensor_msgs/msg/Imu");
  
  return b_success;
}
