cmake_minimum_required(VERSION 3.8)
project(crazyflie_sitl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crtp_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  include
)

add_executable(crazyflie_sitl src/crazyflie_sitl.cpp)
target_include_directories(crazyflie_sitl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(crazyflie_sitl PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Build quadcopter_model library
file(GLOB_RECURSE QUADCOPTER_MODEL_SRCS
    src/quadcopter_model/common/*.cpp
    src/quadcopter_model/dynamics/*.cpp
    src/quadcopter_model/objects/*.cpp
    src/quadcopter_model/sensors/*.cpp
)

add_library(quadcopter_model
  ${QUADCOPTER_MODEL_SRCS}
)
set_target_properties(quadcopter_model PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_compile_features(quadcopter_model PUBLIC cxx_std_17)
target_include_directories(quadcopter_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crazyflie_sitl>
  $<INSTALL_INTERFACE:include>
)


target_link_libraries(quadcopter_model
  Eigen3::Eigen
  yaml-cpp
)

## Build connection library

set(
    COMMUNICATION_SRCS
    src/communication/firmwarelink.cpp
    src/communication/radiolink.cpp
    src/communication/sitl_communication.cpp
)

add_library(communication_lib
  ${COMMUNICATION_SRCS}
)
set_target_properties(communication_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(communication_lib PUBLIC cxx_std_17)

target_include_directories(communication_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/crazyflie_sitl>
  $<INSTALL_INTERFACE:include>
)

## Build the firmware laucher lib
add_library(firmware_launcher_lib
  src/firmware_launcher.cpp
)
set_target_properties(firmware_launcher_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(firmware_launcher_lib PUBLIC cxx_std_17)

target_include_directories(firmware_launcher_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link the quadcopter_model and connection library to the crazyflie_sitl executable
target_link_libraries(crazyflie_sitl
  quadcopter_model
  communication_lib
  firmware_launcher_lib
)
ament_target_dependencies(crazyflie_sitl
  rclcpp
  tf2_ros
  geometry_msgs
  crtp_interfaces
  rclcpp_components
)

install(TARGETS crazyflie_sitl
  DESTINATION lib/${PROJECT_NAME})

# Also register the SITL crazyflie as a component
add_library(crazyflie_sitl_component SHARED src/crazyflie_sitl.cpp)
target_link_libraries(crazyflie_sitl_component 
  quadcopter_model
  communication_lib
  firmware_launcher_lib
)
ament_target_dependencies(crazyflie_sitl_component
  rclcpp
  tf2_ros
  geometry_msgs
  crtp_interfaces
  rclcpp_components)

rclcpp_components_register_nodes(crazyflie_sitl_component "CrazyflieSITL")
install(TARGETS crazyflie_sitl_component DESTINATION lib)

# And build the container
add_executable(container src/crazyflie_sitl_container.cpp)
target_include_directories(container PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(container PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  container
  rclcpp
  rclcpp_components
  ament_index_cpp
)
target_link_libraries(container
  yaml-cpp
)

install(TARGETS container
  DESTINATION lib/${PROJECT_NAME})

# Finally install the firmware to be executed with ros2 run

install(PROGRAMS firmware/cf2
  DESTINATION lib/${PROJECT_NAME} 
)

# Copy config file
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
