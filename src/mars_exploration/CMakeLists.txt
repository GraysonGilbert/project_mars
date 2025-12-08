cmake_minimum_required(VERSION 3.8)
project(mars_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Doxygen REQUIRED)

###################################################
# Compile commands for clangd
###################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

###################################################
# Doxygen Rules
###################################################
if (DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_HAVE_DOT YES)
  set(DOXYGEN_UML_LOOK YES)
  set(DOXYGEN_CLASS_DIAGRAMS YES)
  set(DOXYGEN_CALL_GRAPH YES)
  set(DOXYGEN_CALLER_GRAPH YES)
  set(DOXYGEN_DOT_IMAGE_FORMAT svg)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES 50)
  set(DOXYGEN_DOT_TRANSPARENT YES)
  set(DOXYGEN_DOT_MULTI_TARGETS YES)
else()
  message(FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation")
endif()

###################################
# Doxygen target
###################################
doxygen_add_docs(docs
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test)

###################################
# MarsExploration core library
###################################
add_library(mars_exploration
  src/mars_exploration.cpp
)
ament_target_dependencies(mars_exploration
  nav2_msgs
)
target_include_directories(
  mars_exploration
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
install(TARGETS mars_exploration
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

###################################
# Node implementation library (NO main here)
###################################
add_library(mars_exploration_node_lib
  src/mars_exploration_node.cpp
  src/mars_exploration.cpp
)
target_include_directories(mars_exploration_node_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(mars_exploration_node_lib
  rclcpp
  nav2_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  rclcpp_action
)

###################################
# Main executable for MarsExplorationNode
###################################
add_executable(mars_exploration_node
  src/mars_exploration_node_main.cpp
)
target_link_libraries(mars_exploration_node
  mars_exploration_node_lib
)
ament_target_dependencies(mars_exploration_node
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  sensor_msgs
  nav2_msgs
  nav2_bt_navigator
  slam_toolbox
)
install(TARGETS mars_exploration_node
  DESTINATION lib/${PROJECT_NAME}
)

###################################
# Scan frame rewriter
###################################
add_executable(scan_frame_rewriter
  src/scan_frame_rewriter.cpp
)
ament_target_dependencies(scan_frame_rewriter
  rclcpp
  sensor_msgs
)
install(
  TARGETS
    scan_frame_rewriter
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(catch_ros2 REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(nav2_msgs REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(tf2_ros REQUIRED)

  # GTest on the core exploration library
  ament_add_gtest(mars_exploration_tests
    test/mars_exploration_tests.cpp
  )
  target_link_libraries(mars_exploration_tests mars_exploration)
  ament_target_dependencies(mars_exploration_tests nav2_msgs)

  # --- Catch2 integration test node ---
  add_executable(mars_exploration_integration_test_node
    test/mars_exploration_node_tests.cpp
  )

  target_link_libraries(mars_exploration_integration_test_node
    catch_ros2::catch_ros2_with_node_main
    mars_exploration_node_lib
  )
  ament_target_dependencies(mars_exploration_integration_test_node
    rclcpp
    std_msgs
    std_srvs
    nav2_msgs
    rclcpp_action
    tf2_ros
  )
  install(TARGETS
    mars_exploration_integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )
  catch_ros2_add_integration_test(MarsExplorationNode_IntegrationTest
    LAUNCH_FILE mars_exploration_node_tests.launch.yaml
  )
endif()


###################################
# Install headers and resources
###################################
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
