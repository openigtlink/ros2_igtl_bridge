cmake_minimum_required(VERSION 3.5)
project(ros2_igtl_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenIGTLink REQUIRED)

#find_package(ros2_igtl_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/String.msg"
  "msg/Transform.msg"
  "msg/PointArray.msg"
  "msg/Image.msg"
  "msg/PoseArray.msg"
  "msg/PolyData.msg"
  "msg/Vector.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs
  )

ament_export_dependencies(rosidl_default_runtime)


#include_directories(include ${OpenIGTLink_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/testing/include ${VTK_INCLUDE_DIRS})
include_directories(include ${OpenIGTLink_INCLUDE_DIRS})

add_executable(igtl_node
  src/main.cpp
  src/igtl_node.cpp
  src/rib_converter_manager.cpp
  src/rib_converter_base.cpp
  src/rib_converter_string.cpp
  src/rib_converter_transform.cpp
  src/rib_converter_image.cpp
  src/rib_converter_pointarray.cpp
  src/rib_converter_posearray.cpp
  #src/rib_converter_polydata.cpp
  )

add_executable(igtl_test_listener src/igtl_test_listener.cpp)
add_executable(igtl_test_publisher src/igtl_test_publisher.cpp)
add_executable(tf_listener src/rib_tf2Transform.cpp)

ament_target_dependencies(igtl_node rclcpp std_msgs sensor_msgs OpenIGTLink)
ament_target_dependencies(igtl_test_listener rclcpp std_msgs sensor_msgs)
ament_target_dependencies(igtl_test_publisher rclcpp std_msgs sensor_msgs)
ament_target_dependencies(tf_listener rclcpp std_msgs sensor_msgs tf2 tf2_ros)

rosidl_target_interfaces(igtl_node ${PROJECT_NAME} rosidl_typesupport_cpp)
rosidl_target_interfaces(igtl_test_listener ${PROJECT_NAME} rosidl_typesupport_cpp)
rosidl_target_interfaces(igtl_test_publisher ${PROJECT_NAME} rosidl_typesupport_cpp)
rosidl_target_interfaces(tf_listener ${PROJECT_NAME} rosidl_typesupport_cpp)

target_include_directories(igtl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(igtl_test_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(igtl_test_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(tf_listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS
  igtl_node
  igtl_test_listener
  igtl_test_publisher
  tf_listener
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
