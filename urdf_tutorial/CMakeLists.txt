cmake_minimum_required(VERSION 3.5)
project(urdf_tutorial LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(controller_manager 4.0.0)
find_package(ros2_control_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
add_library(
  ros2_control_demo_example_1
  SHARED
  hardware/rrbot.cpp
)
target_compile_features(ros2_control_demo_example_1 PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_example_1 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_example_1>
)
target_link_libraries(ros2_control_demo_example_1 PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_example_1.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_example_1
)
install(
  DIRECTORY launch/
  DESTINATION share/urdf_tutorial
)
install(
  DIRECTORY urdf/
  DESTINATION share/urdf_tutorial/urdf/
)
install(
  DIRECTORY meshes/
  DESTINATION share/urdf_tutorial/meshes/
)
install(TARGETS ros2_control_demo_example_1
  EXPORT export_ros2_control_demo_example_1
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_ros2_control_demo_example_1 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
