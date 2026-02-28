cmake_minimum_required(VERSION 3.8)
project(late_fusion_for_yolos_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(late_fusion_node src/late_fusion_node.cpp)

ament_target_dependencies(late_fusion_node
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
  image_transport
)

target_link_libraries(late_fusion_node ${OpenCV_LIBRARIES})

install(TARGETS late_fusion_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
