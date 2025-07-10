#  Copyright (c) 2023-2024 Analog Devices, Inc. All Rights Reserved.
#  This software is proprietary to Analog Devices, Inc. and its licensors.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    arg_log_level = DeclareLaunchArgument('log_level', default_value='INFO')
    log_level = LaunchConfiguration('log_level')

    yaml_load = os.path.join(
        get_package_share_directory('adrd2121_imu'),
        'config/'
        'adis16500.yaml'
    )

    main_node_run=Node(
        package="adrd2121_imu",
        name="adrd2121_imu_node",
        executable="adrd2121_imu_node",
        parameters=[yaml_load],
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", 
            PythonExpression(expression=["'","adrd2121_imu_node:=",log_level,"'"])]
        )
    
    return LaunchDescription([
    	arg_log_level,
        main_node_run])
