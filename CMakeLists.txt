cmake_minimum_required(VERSION 3.16)
project(triggering_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

add_compile_definitions(TEST_GAZEBO)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  builtin_interfaces
  control_msgs
  controller_interface
  generate_parameter_library
  pluginlib
  rclcpp_lifecycle
  rcutils
  realtime_tools
  sensor_msgs
  kdl_parser
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(Boost REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(triggering_controller_parameters
  src/triggering_controller_parameters.yaml
)

add_library(triggering_controller SHARED
  src/triggering_controller.cpp
)
target_compile_features(triggering_controller PUBLIC cxx_std_17)
target_include_directories(triggering_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/triggering_controller>
)
target_link_libraries(triggering_controller PUBLIC
  triggering_controller_parameters
)
ament_target_dependencies(triggering_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(triggering_controller PRIVATE "TRIGGERING_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface triggering_controller_plugin.xml)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(hardware_interface REQUIRED)
#   find_package(rclcpp REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)

#   ament_add_gmock(test_load_triggering_controller
#     test/test_load_triggering_controller.cpp
#   )
#   target_link_libraries(test_load_triggering_controller
#     triggering_controller
#   )
#   ament_target_dependencies(test_load_triggering_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   ament_add_gmock(test_triggering_controller
#     test/test_triggering_controller.cpp
#   )
#   target_link_libraries(test_triggering_controller
#     triggering_controller
#   )
#   ament_target_dependencies(test_triggering_controller
#     hardware_interface
#   )
# endif()

install(
  DIRECTORY include/
  DESTINATION include/triggering_controller
)
install(
  TARGETS
    triggering_controller
    triggering_controller_parameters
  EXPORT export_triggering_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_triggering_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
