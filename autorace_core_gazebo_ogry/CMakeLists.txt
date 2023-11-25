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

add_executable(driver src/drive.cpp)
ament_target_dependencies(driver rclcpp geometry_msgs sensor_msgs)

add_executable(recognizer src/recognizer.cpp)
ament_target_dependencies(recognizer rclcpp geometry_msgs sensor_msgs ament_cmake)
target_link_libraries(recognizer ${OpenCV_LIBS})

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
