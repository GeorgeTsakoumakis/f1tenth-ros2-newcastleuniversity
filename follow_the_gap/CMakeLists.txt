cmake_minimum_required(VERSION 3.5)
project(follow_the_gap)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  rclcpp
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()


# Include directories
include_directories(include)

# Create executable
add_executable(follow_the_gap src/follow_the_gap.cpp)

# Add dependencies to the executable
ament_target_dependencies(follow_the_gap 
		rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs
)


# Install executable
install(TARGETS
  follow_the_gap
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


# Export package information
ament_package()

