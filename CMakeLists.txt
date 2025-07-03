cmake_minimum_required(VERSION 3.16)
project(ros_synapse)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Qt6
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Network)

# Find ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

# Find tomlplusplus
find_package(PkgConfig REQUIRED)
pkg_check_modules(TOMLPLUSPLUS REQUIRED tomlplusplus)

# Qt6 setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/main.cpp
  src/mainwindow.cpp
  src/commandwidget.cpp
  src/sshtabwidget.cpp
  src/sshpassworddialog.cpp
  src/sshmanager.cpp
  src/ros2executor.cpp
  src/processmonitor.cpp
  src/configmanager.cpp
)

# Header files
set(HEADERS
  include/mainwindow.h
  include/commandwidget.h
  include/sshtabwidget.h
  include/sshpassworddialog.h
  include/sshmanager.h
  include/ros2executor.h
  include/processmonitor.h
  include/configmanager.h
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# Link libraries
target_link_libraries(${PROJECT_NAME}
  Qt6::Core
  Qt6::Widgets
  Qt6::Network
  ${TOMLPLUSPLUS_LIBRARIES}
)

# Add ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcl_interfaces
)

# Include TOML headers
target_include_directories(${PROJECT_NAME} PRIVATE ${TOMLPLUSPLUS_INCLUDE_DIRS})

# Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION bin
)

ament_package()