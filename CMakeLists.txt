cmake_minimum_required(VERSION 3.8)
project(timings_ros2_conversion_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_interface_mpc REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(cpp_state src/cpp_state.cpp src/utils.cpp)
target_include_directories(cpp_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(cpp_state PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(cpp_state rclcpp std_msgs ros_interface_mpc)
target_link_libraries(
  cpp_state
  ${EIGEN3_LIBRARIES}
)

add_executable(cpp_trajectory src/cpp_trajectory.cpp src/utils.cpp)
target_include_directories(cpp_trajectory PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS})
target_compile_features(cpp_trajectory PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(cpp_trajectory rclcpp std_msgs ros_interface_mpc)
target_link_libraries(
  cpp_trajectory
  ${EIGEN3_LIBRARIES}
)

install(TARGETS cpp_state cpp_trajectory
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(scripts)
# Install Python executables
install(PROGRAMS
  scripts/python_state.py
  scripts/python_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
