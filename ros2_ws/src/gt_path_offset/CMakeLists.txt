cmake_minimum_required(VERSION 3.8)

cmake_policy(SET CMP0074 NEW)
project(gt_path_offset)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED CONFIG)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common io)

# Declare the service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SavePath.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs
)

# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(${PROJECT_NAME}_node src/gt_path_offset.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
$<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
add_dependencies(${PROJECT_NAME}_node ${cpp_typesupport_target})

ament_target_dependencies(
  ${PROJECT_NAME}_node
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "pcl_conversions"
  "std_msgs"
  # gt_path_offset__rosidl_typesupport_cpp # For the custom service
)

# Link PCL libraries
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
  PRIVATE Eigen3::Eigen
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install service files
install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
  std_msgs
  PCL
  rosidl_default_runtime
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
