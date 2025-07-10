/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#include "bias_estimate_node_ros2.hpp"

//Pointer to BiasEstimateRos2 class
BiasEstimateRos2 * p_bias_estimate;

/*****************************************************************************/
/*************************MAIN FUNCTION BIAS_ESTIMATE*************************/
/*****************************************************************************/

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  //Create the Shared Node
  rclcpp::Node::SharedPtr bias_estimate_node_;
  bias_estimate_node_ = rclcpp::Node::make_shared("bias_estimate_node_ros2");
  p_bias_estimate = new BiasEstimateRos2(bias_estimate_node_);

  //Registering the new signal handler
  std::signal(SIGINT, biasGracefulShutdownHandler);
  
  //Setting executor to be multithreaded
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(bias_estimate_node_);

  if ((p_bias_estimate->init()) && (rclcpp::ok()))
  {
      //Switched to multithreaded spin as it's needed for the async call of trigger_imu_glob_cmd
      exec.spin();
  }
  shutdown();
  return 0;
}

/*****************************************************************************/
/*********************END OF MAIN FUNCTION BIAS_ESTIMATE**********************/
/*****************************************************************************/

/*****************************************************************************/
/***************************BIASESTIMATEROS2 CLASS****************************/
/*****************************************************************************/

BiasEstimateRos2::BiasEstimateRos2(const rclcpp::Node::SharedPtr& node)
: node_(node), p_state_checker_()
{
  //Object initialization
  RCLCPP_INFO(node_->get_logger(), "[Bias Estimate] Bias Estimate Node has been constructed");
  p_state_checker_ = new ImuStateCheckerRos2(node_);
}

BiasEstimateRos2::~BiasEstimateRos2()
{
  //Destroying objects and cleaning of node shared_ptr
  RCLCPP_INFO(node_->get_logger(), "[Bias Estimate] Bias Estimate Node has been destructed.");
  delete p_state_checker_;
  p_state_checker_ = nullptr;
  node_ = nullptr;
}

bool BiasEstimateRos2::init()
{
  bool b_success = false;
  int count = RESET;
  b_success = p_state_checker_->init();
  //Creating the callback group to be Reentrant 
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
  if (b_success)
  {
    //Create service server(bias_estimate) and client(trigger_imu_glob_cmd) as part of CB group
    //and set middleware setting to default
    bias_estimate_service_ = node_->create_service<adrd2121_imu::srv::BiasEstimateCmd>
      ("bias_estimate", std::bind(&BiasEstimateRos2::biasEstimateCB, this, _1, _2),
      rmw_qos_profile_services_default, callback_group_);
    
    trigger_imu_glob_cmd_ = node_->create_client<adrd2121_imu::srv::ImuGlobCmd>
      ("trigger_imu_glob_cmd", rmw_qos_profile_services_default, callback_group_);

    //Check if trigger_imu_glob_cmd service is available. If not, terminate bias_estimate node. 
    while(!trigger_imu_glob_cmd_->wait_for_service() && b_success)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(node_->get_logger(), "[BiasEstimateRos2] Waiting for service advertisement");
      count++;
      if (MAX_WAIT_SERVICE == count)
      {
        b_success = false;
      }
    }
    RCLCPP_WARN_EXPRESSION(node_->get_logger(), b_success, "Service and client created.");
    RCLCPP_ERROR_EXPRESSION(node_->get_logger(), !b_success,
      "[BiasEstimateRos2] trigger_imu_glob_cmd service is not advertised.");
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "BIAS_ESTIMATE SERVICE IS NOT ADVERTISED");
  }

  return b_success;
}


bool BiasEstimateRos2::biasEstimateCB(adrd2121_imu::srv::BiasEstimateCmd::Request\
  ::SharedPtr req, std::shared_ptr<adrd2121_imu::srv::BiasEstimateCmd::Response> res)
{
  //req is casted to void in order to technically "use" it, avoiding any warning during build
  (void)req;
  rclcpp::Time standstill_begin;
  bool b_standstill_duration = false;
  bool b_bias_service_success = false;
  double duration_in_secs = RESET;
    
  std::ostringstream ss;
  
  //Check if current state is standstill
  if(p_state_checker_->getState() == STANDSTILL)
  {
    standstill_begin = p_state_checker_->getStandstillBegin();
    b_standstill_duration = this->checkStandStillDuration(standstill_begin, duration_in_secs);
    if (b_standstill_duration)
    {
      RCLCPP_INFO(node_->get_logger(),
        "[Bias Estimate] Now triggering IMU bias correction update (triger_imu_glob_cmd_)");
      adrd2121_imu::srv::ImuGlobCmd::Request::SharedPtr imu_request_data = 
        std::make_shared<adrd2121_imu::srv::ImuGlobCmd::Request>();
      
      //Send request data to trigger_imu_glob_cmd service
      imu_request_data->bit = 0;
      RCLCPP_INFO(node_->get_logger(), "Request data has been seen to trigger_imu_glob_cmd...");

      //Create object that can receive the response data from future call
      rclcpp::Client<adrd2121_imu::srv::ImuGlobCmd>::FutureAndRequestId result = 
        trigger_imu_glob_cmd_->async_send_request(imu_request_data);
      
      //This will set the wait time for the response for service
      RCLCPP_INFO(node_->get_logger(), "Now waiting for service response...");
      std::future_status service_status = result.wait_for(std::chrono::seconds(5));

      //If the response is ready then it will catch the response value from trigger_imu_glob_cmd
      if (std::future_status::ready == service_status)
      {
        adrd2121_imu::srv::ImuGlobCmd::Response::SharedPtr response = result.get();
        res->message = "Bias Correction update successfully triggered";
        res->success = response.get()->success;
        b_bias_service_success = true;
      }
      else
      {
        res->message = "Bias Correction Update unsuccessful.";
        res->success = false;
        b_bias_service_success = false;
      }
    }
    else
    {
      ss << "[Bias Estimate] Please make sure IMU is standstill for at least 40s. Standstill time:" 
         << duration_in_secs << "s";
      RCLCPP_WARN_STREAM(node_->get_logger(), ss.str());
      res->success = false;
      res->message = ss.str();
    }
    res->state = "STANDSTILL";
    res->stand_still_secs = duration_in_secs;
  }
  else
  {
    ss << "[Bias Estimate] Please make sure IMU is standstill for at least 40s." <<
      " IMU is still at MOVING state";
    RCLCPP_WARN_STREAM(node_->get_logger(), ss.str());
    res->success = false;
    res->message = ss.str();
    res->state = "MOVING";
    res->stand_still_secs = duration_in_secs;
  }
  return b_bias_service_success;
}

bool BiasEstimateRos2::checkStandStillDuration(rclcpp::Time standstill_begin, 
  double &duration_in_secs)
{
  bool b_success = false;
  
  //Acquire current time as of call
  rclcpp::Time current_time = node_->now();

  //Check duration standstill start up until the call of this function
  duration_in_secs = current_time.seconds() - standstill_begin.seconds();
  
  //Set that the IMU should be Standstil for at least 40s
  b_success = (TRIGGER_WAIT_TIME <= duration_in_secs)? true:false;
  return b_success;
}

/*****************************************************************************/
/***********************END OF BIASESTIMATEROS2 CLASS*************************/
/*****************************************************************************/


/*****************************************************************************/
/*****************************SHUTDOWN FUNCTIONS******************************/
/*****************************************************************************/
void biasGracefulShutdownHandler(int signal)
{
  if (SIGINT == signal)
  {
    cout << "======= BIAS_ESTIMATE GRACEFULSHUTDOWN CALLED======="<<endl;
    rclcpp::shutdown();
  }
}
void shutdown(void)
{
  cout << "SHUTTING DOWN NOW."<<endl;

  delete p_bias_estimate;
  p_bias_estimate = nullptr;
  
}
/*****************************************************************************/
/**************************END OF SHUTDOWN FUNCTIONS**************************/
/*****************************************************************************/
