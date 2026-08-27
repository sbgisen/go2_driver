cmake_minimum_required(VERSION 3.8)
project(go2_driver)

if(
  CMAKE_COMPILER_IS_GNUCXX
  OR
    CMAKE_CXX_COMPILER_ID
    MATCHES
    "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(
  dependencies
  rclcpp
  rclcpp_components
  builtin_interfaces
  sensor_msgs
  geometry_msgs
  nav_msgs
  unitree_go
  unitree_api
  go2_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED src/go2_driver/go2_driver.cpp src/go2_driver/go2_sport_bridge.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN
  "go2_driver::Go2Driver"
  EXECUTABLE
  go2_driver_node
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN
  "go2_driver::Go2SportBridge"
  EXECUTABLE
  go2_sport_bridge_node
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES LICENSE NOTICE
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
