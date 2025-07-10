cmake_minimum_required(VERSION 3.8)
project(adrd2121_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################### FINDING DEPENDENCIES #########################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
######################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include/adrd2121_imu
)

message(STATUS " CMAKE_CURRENT_SOURCE_DIR is: " ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS " CMAKE_BINARY_DIR is: " ${CMAKE_BINARY_DIR})
message(STATUS " CMAKE_INSTALL_PREFIX is: " ${CMAKE_INSTALL_PREFIX})
set(CURRENT_CMAKE_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
####################### EXTERNAL PROJECT DEPENDENCIES #########################
include(ExternalProject)
ExternalProject_Add(adi_imu_driver
   SOURCE_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/adi_imu_driver
   STAMP_DIR           ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/adi_imu_driver-stamp
   TMP_DIR             ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/adi_imu_driver-tmp
   GIT_REPOSITORY      https://github.com/analogdevicesinc/adi-imu-driver.git
   GIT_TAG             4a932433df7a27bad86f90c3f2d182df67705592
   GIT_REMOTE_UPDATE_STRATEGY CHECKOUT
   BINARY_DIR          ${CMAKE_BINARY_DIR}/dependencies/adi_imu_driver
   INSTALL_DIR         ${CURRENT_CMAKE_INSTALL_PREFIX}
   BUILD_COMMAND       cmake -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR> <SOURCE_DIR>
   INSTALL_COMMAND     make install
   TEST_COMMAND        echo ">>> adi_imu_driver_dirs: " COMMAND echo <SOURCE_DIR> COMMAND echo <BINARY_DIR>
   LOG_DOWNLOAD        1
   LOG_BUILD           1
   LOG_INSTALL         1 
 )

 ExternalProject_Get_Property(adi_imu_driver SOURCE_DIR)
 message(STATUS " adi_imu_driver's source_dir is: " ${SOURCE_DIR})

 ExternalProject_Get_Property(adi_imu_driver BINARY_DIR)
 message(STATUS " adi_imu_driver's binary_dir is: " ${BINARY_DIR})

 ExternalProject_Get_Property(adi_imu_driver INSTALL_DIR)
 message(STATUS " adi_imu_driver's install_dir is: " ${INSTALL_DIR})

 set(ADI_IMU_DRIVER_IMU_LIB ${INSTALL_DIR}/lib/libadimu.so)
 set(ADI_IMU_DRIVER_BUF_LIB ${INSTALL_DIR}/lib/libadimubuf.so)
 set(ADI_IMU_DRIVER_HWCOMM_LIB ${INSTALL_DIR}/lib/libhwcomm.so)

 set(ADI_IMU_DRIVER_INCLUDE ${INSTALL_DIR}/include)

 add_library(adi_imu_driver_imu_target SHARED IMPORTED)
 set_target_properties(adi_imu_driver_imu_target PROPERTIES IMPORTED_LOCATION ${ADI_IMU_DRIVER_IMU_LIB})
 add_dependencies(adi_imu_driver_imu_target adi_imu_driver)

 add_library(adi_imu_driver_imubuf_target SHARED IMPORTED)
 set_target_properties(adi_imu_driver_imubuf_target PROPERTIES IMPORTED_LOCATION ${ADI_IMU_DRIVER_BUF_LIB})
 add_dependencies(adi_imu_driver_imubuf_target adi_imu_driver)

 add_library(adi_imu_driver_hwcomm_target SHARED IMPORTED)
 set_target_properties(adi_imu_driver_hwcomm_target PROPERTIES IMPORTED_LOCATION ${ADI_IMU_DRIVER_HWCOMM_LIB})
 add_dependencies(adi_imu_driver_hwcomm_target adi_imu_driver)



 ####################### INTERFACES LIBRARY #########################
 rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ImuGlobCmd.srv"
  "srv/BufStatus.srv" 
  "msg/AdiImu.msg"
  "msg/ImuState.msg"
  "srv/BiasEstimateCmd.srv"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

######################################################################
include_directories(
  include
  ${ADI_IMU_DRIVER_INCLUDE}
)

install(
  DIRECTORY include
  DESTINATION include
)

####################### ADRD2121_IMU_COMMON LIBRARY #########################

 add_library(adrd2121_imu_ros2_common_lib src/adrd2121_imu_ros2_common.cpp)

 target_include_directories(adrd2121_imu_ros2_common_lib
   PUBLIC
   include
 )
 install(
   TARGETS adrd2121_imu_ros2_common_lib
   LIBRARY DESTINATION lib
   ARCHIVE DESTINATION lib
   RUNTIME DESTINATION bin
   INCLUDES DESTINATION include
 )

####################### ADI_IMU_ROS2 LIBRARY #########################
add_library(adi_imu_ros2_library src/adi_imu_ros2.cpp)
ament_target_dependencies(adi_imu_ros2_library rclcpp std_msgs rcl_interfaces)

target_include_directories(adi_imu_ros2_library PUBLIC
  include
  ${ADI_IMU_DRIVER_INCLUDE}
)

target_link_libraries(adi_imu_ros2_library
  adi_imu_driver_imu_target
  ${cpp_typesupport_target}
  adrd2121_imu_ros2_common_lib
)

install(
  TARGETS adi_imu_ros2_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

######################################################################

####################### IMU_BUF_ROS2 LIBRARY #########################
add_library(imu_buf_ros2_library src/imu_buf_ros2.cpp)
ament_target_dependencies(imu_buf_ros2_library rclcpp std_srvs std_msgs rcl_interfaces sensor_msgs)

target_include_directories(imu_buf_ros2_library 
  PUBLIC
  include
  ${ADI_IMU_DRIVER_INCLUDE}
)

target_link_libraries(imu_buf_ros2_library
  ${cpp_typesupport_target}  
  adi_imu_driver_imu_target
  adi_imu_driver_imubuf_target
  adrd2121_imu_ros2_common_lib
)

install(
  TARGETS imu_buf_ros2_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

######################################################################

####################### ADI_IMU_BUF_ROS2 LIBRARY #########################
add_library(adi_imu_buf_ros2_library src/adi_imu_buf_ros2.cpp)
ament_target_dependencies(adi_imu_buf_ros2_library rclcpp std_msgs rcl_interfaces)
target_link_libraries(adi_imu_buf_ros2_library 
  adi_imu_ros2_library 
  imu_buf_ros2_library
  adi_imu_driver_hwcomm_target
)

target_include_directories(adi_imu_buf_ros2_library
  PUBLIC
  include
  ${ADI_IMU_DRIVER_INCLUDE}
)

install(
  TARGETS adi_imu_buf_ros2_library
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

####################### IMU_STATE_CHECK LIBRARY #########################

add_library(imu_state_checker_ros2_lib src/imu_state_checker_ros2.cpp)
ament_target_dependencies(imu_state_checker_ros2_lib rclcpp sensor_msgs)

target_include_directories(imu_state_checker_ros2_lib
  PUBLIC
  include
)

target_link_libraries(imu_state_checker_ros2_lib
  adrd2121_imu_ros2_common_lib
  ${cpp_typesupport_target}
)

install(
  TARGETS imu_state_checker_ros2_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


################## ADRD2121_IMU_ROS2 EXECUTABLE ####################

add_executable(adrd2121_imu_node src/adrd2121_imu_ros2_node.cpp)
target_link_libraries(adrd2121_imu_node adi_imu_buf_ros2_library)
ament_target_dependencies(adrd2121_imu_node rclcpp rcl_interfaces std_msgs geometry_msgs)

add_executable(bias_estimate_node_ros2 src/bias_estimate_node_ros2.cpp)
target_link_libraries(bias_estimate_node_ros2 imu_state_checker_ros2_lib ${cpp_typesupport_target})
ament_target_dependencies(bias_estimate_node_ros2 rclcpp sensor_msgs)

install(
  TARGETS adrd2121_imu_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS bias_estimate_node_ros2
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
