cmake_minimum_required(VERSION 3.8)
project(ik_solver_moveit_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ================================
# Dependencies
# ================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ik_solver_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDS
  rclcpp
  moveit_msgs
  geometry_msgs
  ik_solver_msgs
)

# ================================
# Executable Node
# ================================
add_executable(ik_adapter_node
  src/ik_adapter_node.cpp
)

ament_target_dependencies(ik_adapter_node ${THIS_PACKAGE_DEPENDS})

# ================================
# Installation
# ================================

# Install the node binary
install(TARGETS ik_adapter_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Package export
ament_package()
