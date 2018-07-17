cmake_minimum_required(VERSION 3.5)
project(rosbag2)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# [sriram] Check if, we need to create local factory.
find_package(ros1_bridge REQUIRED)

# find ROS 1 packages
include(cmake/find_ros1_package.cmake)

find_package(PkgConfig)
if(NOT PKG_CONFIG_FOUND)
  message(STATUS "Failed to find PkgConfig, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_ros1_package(roscpp)
find_ros1_package(rosbag_storage)
if(NOT ros1_roscpp_FOUND)
  message(STATUS "Failed to find ROS 1 roscpp, skipping...")
  # call ament_package() to avoid ament_tools treating this as a plain CMake pkg
  ament_package()
  return()
endif()

find_ros1_package(std_msgs REQUIRED)
find_ros1_package(sensor_msgs REQUIRED)

# find ROS 1 packages with messages / services
include(cmake/find_ros1_interface_packages.cmake)
find_ros1_interface_packages(ros1_message_packages)

set(prefixed_ros1_message_packages "")
foreach(ros1_message_package ${ros1_message_packages})
  find_ros1_package(${ros1_message_package} REQUIRED)
  list(APPEND prefixed_ros1_message_packages "ros1_${ros1_message_package}")
endforeach()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})
ament_package()

set(generated_path "${CMAKE_BINARY_DIR}/generated")
set(generated_files "${generated_path}/get_factory.cpp")
list(APPEND generated_files "${generated_path}/get_mappings.cpp")

ament_index_get_resources(ros2_message_packages "rosidl_interfaces")
foreach(message_package ${ros2_message_packages})
  find_package(${message_package} REQUIRED)
  if(NOT "${message_package}" STREQUAL "builtin_interfaces")
    list(APPEND generated_files "${generated_path}/${message_package}_factories.cpp")
  endif()
endforeach()

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND ${PYTHON_EXECUTABLE} bin/ros1_bridge_generate_factories
    --output-path "${generated_path}" --template-dir resource
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

if(NOT WIN32)
  # ignore warning in ROS 1 message headers
  set_source_files_properties(${generated_files}
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter")
endif()

include_directories(include ${generated_path})

function(custom_executable target)
  cmake_parse_arguments(
    ARG "ROS1_DEPENDENCIES" "" "TARGET_DEPENDENCIES" ${ARGN})

  add_executable(${target}
    ${ARG_UNPARSED_ARGUMENTS})
  ament_target_dependencies(${target}
    "Boost"
    "BZIP2"
    "rclcpp"
    "rosbag_factory"
    "rosbag"
    ${ros2_message_packages})
  if(ARG_ROS1_DEPENDENCIES)
    ament_target_dependencies(${target}
      "ros1_roscpp"
      "ros1_sensor_msgs"
      "ros1_rosbag_storage")
  endif()
  if(ARG_DEPENDENCIES)
    add_dependencies(${target} ${ARG_DEPENDENCIES})
  endif()

  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

add_library(librosbag_factory SHARED
  "src/rosbag2/plugins/builtin_interfaces_factories.cpp"
  "src/rosbag2/plugins/convert_builtin_interfaces.cpp"
  ${generated_files})

ament_target_dependencies(librosbag_factory
  ${prefixed_ros1_message_packages}
  ${ros2_message_packages}
  "rclcpp")

custom_executable(${PROJECT_NAME}_play
  "src/rosbag2/ros1play.cpp"
  ROS1_DEPENDENCIES
  TARGET_DEPENDENCIES ${ros2_message_packages})
target_link_libraries(${PROJECT_NAME}_play
  librosbag_factory
  rosbag)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
