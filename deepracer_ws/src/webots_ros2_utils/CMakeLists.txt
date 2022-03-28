# Adapated from the following sources
#     https://github.com/cyberbotics/webots_ros2/blob/develop/webots_ros2_driver/CMakeLists.txt
#     https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/

cmake_minimum_required(VERSION 3.5)
project(webots_ros2_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(webots_ros2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(
  ${PYTHON_INCLUDE_DIRS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python Executables
install(PROGRAMS ${PROJECT_NAME}/ros2_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Build the msg and srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UrdfRobot.msg"
  "srv/SpawnUrdfRobot.srv"
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

ament_package()
