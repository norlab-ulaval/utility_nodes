cmake_minimum_required(VERSION 3.8)
project(utility_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(waiting_node src/waiting_node.cpp)
ament_target_dependencies(waiting_node
  rclcpp
  std_msgs
)

add_executable(topic_hz src/topic_hz_node.cpp)
ament_target_dependencies(topic_hz
  rclcpp
  std_msgs
)

add_library(topic_hz_component SHARED src/topic_hz_component.cpp)
ament_target_dependencies(topic_hz_component
  rclcpp
  std_msgs
  rclcpp_components
)
rclcpp_components_register_node(
        topic_hz_component
        PLUGIN "TopicHzNode"
        EXECUTABLE topic_hz_component_exec
)

install(TARGETS waiting_node topic_hz
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_topic_hz_component)
install(TARGETS topic_hz_component
        EXPORT export_topic_hz_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_package()
