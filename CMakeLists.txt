cmake_minimum_required(VERSION 3.8)
project(areumii_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


# Add a compile directive for a shared library providing the <robot_hardware_interface_name>.cpp file as the source.
add_library(${PROJECT_NAME} SHARED
  src/areumii_hardware_interface.cpp
)

# Add targeted include directories for the library. This is usually only include.
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)


# Add ament dependencies needed by the library.
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


# Export for pluginlib description file using the following command
pluginlib_export_plugin_description_file(hardware_interface areumii_hardware_interface.xml)



# Add install directives for targets and include directory.
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
DESTINATION include
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
