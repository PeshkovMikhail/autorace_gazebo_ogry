cmake_minimum_required(VERSION 3.8)
project(autorace_core_gazebo_ogry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autorace_communication_gazebo_ogry REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_executable(driver src/drive.cpp)
ament_target_dependencies(driver rclcpp geometry_msgs sensor_msgs autorace_communication_gazebo_ogry nav_msgs cv_bridge)
target_link_libraries(driver ${OpenCV_LIBS})

add_library(intersection_server SHARED
  src/intersection_server.cpp)
target_include_directories(intersection_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(intersection_server
  PRIVATE "INTERSECTION_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(intersection_server
  "autorace_communication_gazebo_ogry"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs")
rclcpp_components_register_node(intersection_server PLUGIN "missions_action_cpp::IntersectionActionServer" EXECUTABLE intersection_action_server)


add_library(parking_server SHARED
  src/parking_server.cpp)
target_include_directories(parking_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(parking_server
  PRIVATE "PARKING_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(parking_server
  "autorace_communication_gazebo_ogry"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs")
rclcpp_components_register_node(parking_server PLUGIN "missions_action_cpp::ParkingActionServer" EXECUTABLE parking_action_server)



install(TARGETS
  intersection_server
  parking_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  driver
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
