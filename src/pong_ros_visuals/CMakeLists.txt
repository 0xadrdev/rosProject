cmake_minimum_required(VERSION 3.8)
project(pong_ros_visuals)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Discover where rclcpp is
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pong_ros_interfaces REQUIRED)                      # For custom interface.

# Discover where SDL2 is
find_package(SDL2 REQUIRED)
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
set(SDL2_TTF_LIBRARIES SDL2_ttf)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# add_executable(my_node src/my_node.cpp)
add_executable(${PROJECT_NAME}
    src/PongVisualizationNode.cpp
    src/PongVisualization.cpp
    src/PongRosSdl.cpp
)
#ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs pong_ros_interfaces)             # For custom interface.

# Include the SDL2 libraries (as found above) for this executable
target_link_directories(${PROJECT_NAME} PUBLIC ${SDL2_LINK_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${SDL2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARIES})

#target_include_directories(my_node PUBLIC
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#install(TARGETS my_node
install(TARGETS ${PROJECT_NAME}
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
