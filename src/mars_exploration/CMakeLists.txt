cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(mars_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# below sets up code coverage flags
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

############################################
# Set up unit test and code coverage generation.
############################################
# Must compile with debug, e.g,
#   cmake -S ./ -B build/ -D CMAKE_BUILD_TYPE=Debug
#
# below sets up code coverage flags
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

option(WANT_COVERAGE "this option enable coverage" ON)
if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  set(WANT_COVERAGE OFF)
endif()

if(WANT_COVERAGE)
  message("Enabling coverage")
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")
  include(CodeCoverage)
  # The following line come from cmake-modules/CodeCoverage.cmake
  append_coverage_compiler_flags() # will add "--coverage" compilation flag
  # Exclude certain directories from coverage analysis
  set(COVERAGE_EXCLUDES
    ${PROJECT_BINARY_DIR}
    )
endif()

# create targets for building code coverage reports
if(WANT_COVERAGE)
  if(UNIT_TEST_ALREADY_RUN)
    unset(EXECUTABLE)          # don't run unit test again, just generate the coverage report
  else()
    set(EXECUTABLE ctest test/)   # what to run insdie the build directory?
  endif()
  # unit test coverage:
  setup_target_for_coverage_lcov ( # use lcov to generate coverage report
    NAME test_coverage             # target name
    EXECUTABLE ${EXECUTABLE}
    EXCLUDE
      "/usr/include/*"          # Don't analyze system headers
      "/opt/*"                  # Don't analyze system headers
    )
endif()

# Enables testing for this directory and below
enable_testing()
include(GoogleTest)

# Sanity check:
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
message(STATUS "WANT_COVERAGE    = ${WANT_COVERAGE}")

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

install(PROGRAMS
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})

ament_package()
