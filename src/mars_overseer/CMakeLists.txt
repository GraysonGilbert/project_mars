cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(mars_overseer)

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
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs)
find_package(Doxygen REQUIRED)    # add doxygen support
find_package(catch_ros2 REQUIRED)

# Create Executables and Libraries for package

add_library(overseer_lib
    src/map_merger.cpp
    src/overseer.cpp
)
target_include_directories(overseer_lib PUBLIC
    include
)

# Link dependencies
ament_target_dependencies(overseer_lib
    rclcpp
    nav_msgs
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)
add_executable(overseer_node src/overseer.cpp)

# Link the executable against the library
target_link_libraries(overseer_node overseer_lib)


ament_target_dependencies(overseer_node
    rclcpp
    nav_msgs
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
)

# Install the executable
install(TARGETS
  overseer_node
  overseer_lib
  DESTINATION lib/${PROJECT_NAME}
)

###################################################
# Create the compilation database for clangd and move it out of the build dir.
###################################################
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

###################################################
# Doxygen Rules
# ref: https://cmake.org/cmake/help/latest/module/FindDoxygen.html
###################################################
if (DOXYGEN_FOUND)
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

###################################
# Specify Targets
###################################
# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/test)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

   ament_add_gtest(overseer_unit_tests
    test/overseer_unit_tests.cpp
  )

  ament_target_dependencies(overseer_unit_tests
  rclcpp std_msgs nav_msgs)

  # Link any dependencies your tests need:
  target_link_libraries(overseer_unit_tests
    gtest
    overseer_lib
    #${PROJECT_NAME}
  )

  # Uncomment to run unit tests, these tests will fail in CI because Webots cannot be launched.
  # add_executable(overseer_integration_tests
  # test/overseer_integration_tests.cpp)

  # target_link_libraries(overseer_integration_tests
  # catch_ros2::catch_ros2_with_node_main)

  # ament_target_dependencies(overseer_integration_tests
  # rclcpp std_msgs nav_msgs)

  # install(TARGETS
  # overseer_integration_tests
  # DESTINATION lib/${PROJECT_NAME}
  # )

  # This CMake function allows the integration test to be run
  # when running "colcon test".
  # catch_ros2_add_integration_test (integration_test
  #   LAUNCH_FILE integration_tests.launch.py
  # )

endif()

install(DIRECTORY
  launch  
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME})

ament_package()
