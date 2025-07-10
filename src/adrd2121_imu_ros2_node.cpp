/*
 * Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
 * This software is proprietary to Analog Devices, Inc. and its licensors.
*/

#include "adrd2121_imu_ros2_node.hpp"

//This is a pointer for the AdiImuBufRos2 class
AdiImuBufRos2* p_adi_imu_buf_ros2 = nullptr;
bool g_sigint_called = false;

/*****************************************************************************/
/********************************MAIN FUNCTION********************************/
/*****************************************************************************/
int main (int argc, char* argv[])
{
  bool b_success = false;

  rclcpp::init(argc, argv);
    
  //Remove all default signal handler
  rclcpp::uninstall_signal_handlers();

  //Registering the new signal handler
  std::signal(SIGINT, gracefulShutdownHandler);
    
  //Create the Shared Node
  rclcpp::Node::SharedPtr node;
  node = rclcpp::Node::make_shared("adrd2121_imu");
  
  //Create an object of AdiImuBufRos2 class 
  p_adi_imu_buf_ros2 = new AdiImuBufRos2(node);

  //Check if object AdiImuBufRos2 is created
  if (p_adi_imu_buf_ros2 == nullptr)
  {
    //Terminate all pointers that have been used. 
    RCLCPP_INFO(node->get_logger(), "AdiImuBufRos2 class is not available");
  }
  else
  {
    //Start loading the Parameters
    p_adi_imu_buf_ros2->initParams();
    
    //Start initialization of IMU and SPI Buffer Board
    b_success = p_adi_imu_buf_ros2->init();

    //Device configuration is directly reliant on device initialization. 
    if (b_success)
    {
      if (STREAM == p_adi_imu_buf_ros2->mode_of_operation_)
      {
        b_success = p_adi_imu_buf_ros2->config();
        if (b_success)
        {
          //After initialization, proceed preparing for spin
          p_adi_imu_buf_ros2->readPubData();
          while (rclcpp::ok() && !g_sigint_called && (p_adi_imu_buf_ros2->b_readPubDataSuccess))
          {
          rclcpp::spin_some(node); 
          }
        }
        else
        {
        RCLCPP_ERROR(node->get_logger(), 
          "Device Initialization/Configuration failed. Program is now ending...");
        }
      }
      else if (RECOVERY == p_adi_imu_buf_ros2->mode_of_operation_)
      {
        RCLCPP_WARN(node->get_logger(), "Now in RECOVERY MODE");
        while (rclcpp::ok() && !g_sigint_called)
        {
          rclcpp::spin_some(node);
        }
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Mode is unknown. Check README for available modes.");
      }
    }
    else
    {
      if (p_adi_imu_buf_ros2->b_getHWInitialized_)
      {
        if (RECOVERY == p_adi_imu_buf_ros2->mode_of_operation_)
        {
          RCLCPP_WARN(node->get_logger(), "Now in RECOVERY MODE");
          while ((rclcpp::ok()) && (!g_sigint_called))
          {
            rclcpp::spin_some(node);
          }
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(), "Mode is unknown. Check README for available modes.");
        }
      }
      else
      {
      RCLCPP_ERROR(node->get_logger(), "Hardware did not initialize.");
      }
    }
  }

  shutdown();
  return 0; 
}

/*****************************************************************************/
/****************************END OF MAIN FUNCTION*****************************/
/*****************************************************************************/

/*****************************************************************************/
/*****************************SHUTDOWN FUNCTIONS******************************/
/*****************************************************************************/

//Runs when SIGINT signal has been called.
void gracefulShutdownHandler(int signal)
{
  if (SIGINT == signal)
  {
    cout << "=======MAIN_NODE GRACEFULSHUTDOWN CALLED======="<<endl;
    g_sigint_called = true;
  }
}

//Shutting down the node. 
void shutdown()
{
  cout << "SHUTTING DOWN NOW."<<endl;

  //Delete all object instances
  delete p_adi_imu_buf_ros2;
  p_adi_imu_buf_ros2 = nullptr;
  
  //Shutdown ROS2 
  rclcpp::shutdown();
}
/*****************************************************************************/
/**************************END OF SHUTDOWN FUNCTIONS**************************/
/*****************************************************************************/


