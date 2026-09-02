cmake_minimum_required(VERSION 3.8)
project(gimbal_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)

# BulletPlugin eklentisini derle
add_library(BulletPlugin SHARED src/BulletPlugin.cc)
target_include_directories(BulletPlugin PUBLIC ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(BulletPlugin ${GAZEBO_LIBRARIES})

ament_target_dependencies(BulletPlugin
  rclcpp
  std_msgs
  gazebo_ros
)
# Eklentiyi Gazebo'nun bulabileceği klasöre kur
install(TARGETS BulletPlugin DESTINATION lib)

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
