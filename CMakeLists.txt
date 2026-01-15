cmake_minimum_required(VERSION 3.8)
project(rzv_model_utils_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# create target
add_library(${PROJECT_NAME} SHARED
  src/model_utils.cpp
)

# include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
  $<BUILD_INTERFACE:/usr/include/renesas/opencv4>
)

# ROS2 dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  diagnostic_msgs
  ament_index_cpp
  yaml-cpp
)

# Non-ROS2 dependencies
if(NOT DEFINED RENESAS_LIB_PATH) # Allow overriding via cross compilation
  set(RENESAS_LIB_PATH "/usr/lib/aarch64-linux-gnu/renesas")
endif()

target_link_libraries(${PROJECT_NAME}
  ${RENESAS_LIB_PATH}/libopencv_core.so
  ${RENESAS_LIB_PATH}/libopencv_imgproc.so
  yaml-cpp
)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  ament_index_cpp
  geometry_msgs
  diagnostic_msgs
  yaml-cpp
)

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
