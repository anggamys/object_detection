cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# # Try to find ONNX Runtime
# find_package(ONNXRuntime REQUIRED)  # Note: Package name might be different

# # If find_package doesn't work, try alternative methods
# if(NOT ONNXRuntime_FOUND)
#   # Option 1: Use pkg-config if available
#   find_package(PkgConfig REQUIRED)
#   pkg_check_modules(ONNXRuntime onnxruntime)
  
#   # Option 2: Manual paths if needed
#   if(NOT ONNXRuntime_FOUND)
    set(ONNXRUNTIME_INCLUDE_DIRS "/opt/onnxruntime-linux-x64-1.21.0/include")
    set(ONNXRUNTIME_LIBRARIES "/opt/onnxruntime-linux-x64-1.21.0/lib/libonnxruntime.so")
#   endif()
# endif()

# Kamera Node (publisher)
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

add_executable(detection_node src/detection_node.cpp)
target_include_directories(detection_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
target_link_libraries(detection_node ${ONNXRUNTIME_LIBRARIES})
ament_target_dependencies(detection_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install targets
install(TARGETS
  camera_node
  detection_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()