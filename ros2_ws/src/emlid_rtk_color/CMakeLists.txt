cmake_minimum_required(VERSION 3.8)
project(emlid_rtk_color)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(emlid_gps REQUIRED)
# find_package(GeographicLib REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(std_srvs REQUIRED)

add_executable(emlid_rtk_color src/emlid_rtk_color.cpp)

target_link_libraries(${PROJECT_NAME}
  Geographic
  ${PCL_LIBRARIES}
)

target_include_directories(emlid_rtk_color PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_features(emlid_rtk_color PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  emlid_rtk_color
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "visualization_msgs"
  # "emlid_gps"
  "std_srvs"
  "nav_msgs"
)

install(TARGETS emlid_rtk_color
  DESTINATION lib/${PROJECT_NAME})

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
