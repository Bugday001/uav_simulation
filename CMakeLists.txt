cmake_minimum_required(VERSION 3.8)
project(uav_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)    

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(uav_sim_node src/uav_sim/uav_sim.cpp src/uav_sim/uav_sim_node.cpp src/uav_controller/uav_controller.cpp)
ament_target_dependencies(
  uav_sim_node
  std_msgs
  nav_msgs
  tf2_ros
  tf2
  rclcpp
  Eigen3
)
target_include_directories(uav_sim_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# controller
add_executable(uav_ctrl_node src/uav_controller/uav_controller.cpp src/uav_controller/uav_controller_node.cpp)
ament_target_dependencies(
  uav_ctrl_node
  std_msgs
  nav_msgs
  tf2_ros
  tf2
  rclcpp
  Eigen3
)
target_include_directories(uav_ctrl_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS uav_sim_node uav_ctrl_node
  DESTINATION lib/${PROJECT_NAME})
# launch
install(DIRECTORY launch models config
        DESTINATION share/${PROJECT_NAME})

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
