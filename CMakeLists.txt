cmake_minimum_required(VERSION 3.14)
project(play_launch_container)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition_interfaces REQUIRED)
find_package(play_launch_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rmw REQUIRED)

# Shared library (all three component manager classes)
add_library(observable_component_manager SHARED
  src/observable_component_manager.cpp
  src/clone_isolated_component_manager.cpp
  src/clone_vm_component_manager.cpp
  src/control_channel.cpp
  src/control_json.cpp)
target_include_directories(observable_component_manager PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# Modern CMake targets instead of ament_target_dependencies, which Rolling
# removed. Message packages export ${pkg_TARGETS}; libraries export pkg::lib.
target_link_libraries(observable_component_manager
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  ${composition_interfaces_TARGETS}
  ${play_launch_msgs_TARGETS})
# _dl_allocate_tls and __ctype_init are reached through dlsym at runtime, and
# rmw_get_implementation_identifier decides whether clone-vm may run at all.
target_link_libraries(observable_component_manager rmw::rmw ${CMAKE_DL_LIBS})
# The clone-vm child bootstrap writes into struct pthread at an offset found at
# runtime, immediately after crossing the clone boundary. Do not let the
# optimiser move those stores.
set_source_files_properties(src/clone_vm_component_manager.cpp
  PROPERTIES COMPILE_OPTIONS "-O1")

# Phase 64: control-channel unit tests (JSON codec + framing invariants).
if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_control_json test/test_control_json.cpp)
    if(TARGET test_control_json)
      target_link_libraries(test_control_json observable_component_manager)
    endif()
  endif()
endif()

# SlowLoader test component (shared library for composition)
add_library(slow_loader SHARED src/slow_loader.cpp)
target_link_libraries(slow_loader rclcpp::rclcpp rclcpp_components::component)
rclcpp_components_register_node(slow_loader
  PLUGIN "play_launch_container::SlowLoader"
  EXECUTABLE slow_loader_node)

# Container executable (single binary, configurable via CLI flags)
add_executable(component_container src/component_container.cpp)
target_link_libraries(component_container observable_component_manager rclcpp::rclcpp)

# Standalone node binary (fork+exec'd by CloneIsolatedComponentManager)
add_executable(component_node src/component_node.cpp)
target_link_libraries(component_node
  rclcpp::rclcpp
  rclcpp_components::component
  rclcpp_components::component_manager
  class_loader::class_loader
  ament_index_cpp::ament_index_cpp)

install(TARGETS observable_component_manager slow_loader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS component_container component_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_package()
