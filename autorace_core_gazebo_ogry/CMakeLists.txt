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
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(driver src/drive.cpp)
ament_target_dependencies(driver rclcpp geometry_msgs sensor_msgs autorace_communication_gazebo_ogry nav_msgs cv_bridge)
target_link_libraries(driver ${OpenCV_LIBS})

add_executable(test_driver src/test_driver.cpp)
ament_target_dependencies(test_driver rclcpp geometry_msgs sensor_msgs autorace_communication_gazebo_ogry nav_msgs cv_bridge)
target_link_libraries(test_driver ${OpenCV_LIBS})
target_link_libraries (test_driver ${PCL_LIBRARIES})

add_executable(sign_detector src/sign_detector.cpp)
ament_target_dependencies(sign_detector rclcpp geometry_msgs sensor_msgs autorace_communication_gazebo_ogry nav_msgs cv_bridge)
target_link_libraries(sign_detector ${OpenCV_LIBS})

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

add_library(construction_server SHARED
  src/construction_server.cpp)
target_include_directories(construction_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(construction_server
  PRIVATE "CONSTRUCTION_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(construction_server
  "autorace_communication_gazebo_ogry"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs")
rclcpp_components_register_node(construction_server PLUGIN "missions_action_cpp::ConstructionServer" EXECUTABLE construction_action_server)


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

add_library(crosswalk_server SHARED
  src/crosswalk_server.cpp)
target_include_directories(crosswalk_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(crosswalk_server
  PRIVATE "CROSSWALK_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(crosswalk_server
  "autorace_communication_gazebo_ogry"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs")
rclcpp_components_register_node(crosswalk_server PLUGIN "missions_action_cpp::CrosswalkServer" EXECUTABLE crosswalk_action_server)

add_library(tunnel_server SHARED
  src/tunnel_server.cpp)
target_include_directories(tunnel_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(tunnel_server
  PRIVATE "TUNNEL_ACTION_CPP_BUILDING_DLL")
ament_target_dependencies(tunnel_server
  "autorace_communication_gazebo_ogry"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs")
rclcpp_components_register_node(tunnel_server PLUGIN "missions_action_cpp::TunnelServer" EXECUTABLE tunnel_action_server)

install(TARGETS
  intersection_server
  construction_server
  parking_server
  crosswalk_server
  tunnel_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  driver
  sign_detector
  test_driver
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    map/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
