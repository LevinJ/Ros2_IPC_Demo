cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)
set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(GSCAM_NODE_DEPS
  class_loader
  rclcpp
  rclcpp_components
  sensor_msgs
  OpenCV)



add_library(talker SHARED src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_library(listener SHARED src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

rclcpp_components_register_nodes(talker "MinimalPublisher")
rclcpp_components_register_nodes(listener "MinimalSubscriber")

ament_target_dependencies(talker
${GSCAM_NODE_DEPS})
ament_target_dependencies(listener
${GSCAM_NODE_DEPS})


  install(TARGETS
  talker
  listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()


