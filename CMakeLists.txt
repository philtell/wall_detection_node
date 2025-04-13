cmake_minimum_required(VERSION 3.0.2)
project(wall_detection_node)

# Set C++ standard to C++17
add_compile_options(-std=c++17)

# Find necessary packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  gps_common
)

# Find PCL
find_package(PCL REQUIRED)

# Find Paho MQTT
# find_package(PahoMqttCpp REQUIRED)

catkin_package()

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include                    # for json.hpp
  /usr/local/include                               # for Paho MQTT (ensure it's correct)
)

# Link directories (if needed, but generally catkin package will handle this)
link_directories(
  /usr/local/lib                                   # for Paho MQTT libs
)

# Add your cloud_process.cpp file to the target
add_executable(wall_detector_node src/ros_mqtt_wall_detector.cpp src/cloud_process.cpp)

# Link libraries
target_link_libraries(wall_detector_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  paho-mqttpp3
  paho-mqtt3as
  # PahoMqttCpp::PahoMqttCpp  # This is the Paho MQTT C++ library if you use `find_package`
)
