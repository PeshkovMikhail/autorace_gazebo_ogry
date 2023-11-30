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
ament_target_dependencies(driver rclcpp geometry_msgs sensor_msgs autorace_communication_gazebo_ogry nav_msgs)

add_executable(recognizer src/recognizer.cpp)
ament_target_dependencies(recognizer rclcpp geometry_msgs sensor_msgs ament_cmake autorace_communication_gazebo_ogry nav_msgs)
target_link_libraries(recognizer ${OpenCV_LIBS})


add_library(action_server SHARED
  src/intersection_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "INTERSECTION_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "autorace_communication_gazebo_ogry"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs")
rclcpp_components_register_node(action_server PLUGIN "intersection_action_cpp::IntersectionActionServer" EXECUTABLE intersection_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  driver
  recognizer
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
