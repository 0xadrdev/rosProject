cmake_minimum_required(VERSION 3.8)
project(pong_ros_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pong_ros_interfaces REQUIRED) 
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# package executables 
add_executable(pong_ros_core
    src/PongControllerNode.cpp
    src/PongController.cpp
    src/BallPhysics.cpp
)

ament_target_dependencies(pong_ros_core rclcpp std_msgs pong_ros_interfaces)

# add_executable(physics 
#     src/BallPhysicsNode.cpp
#     src/BallPhysics.cpp
# )

# ament_target_dependencies(physics rclcpp std_msgs pong_ros_interfaces)

# target_include_directories(physics PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

# target_compile_features(physics PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_include_directories(pong_ros_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pong_ros_core PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#install(TARGETS suppub_logic
install(TARGETS 
  # logic
  pong_ros_core
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
