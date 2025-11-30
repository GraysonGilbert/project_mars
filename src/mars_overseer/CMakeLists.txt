cmake_minimum_required(VERSION 3.8)
project(mars_overseer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Doxygen REQUIRED)    # add doxygen support
find_package(catch_ros2 REQUIRED)

# Create Executables and Libraries for package

add_library(overseer_lib
    src/map_merger.cpp
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
)

# Install the executable
install(TARGETS
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

# add_executable(overseer_node src/overseer_node.cpp)
# target_include_directories(overseer_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(overseer_node
#   rclcpp
#   nav_msgs
#   geometry_msgs
#   std_msgs
#   tf2
#   tf2_ros)
#
# install(TARGETS overseer_node
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

   ament_add_gtest(test_build_passes
    test/test_build_passes.cpp
  )

  add_executable(overseer_integration_tests
  test/overseer_integration_tests.cpp)

  target_link_libraries(overseer_integration_tests
  catch_ros2::catch_ros2_with_node_main)

  ament_target_dependencies(overseer_integration_tests
  rclcpp std_msgs nav_msgs)

  install(TARGETS
  overseer_integration_tests
  DESTINATION lib/${PROJECT_NAME}
  )

  # This CMake function allows the integration test to be run
  # when running "colcon test".
  catch_ros2_add_integration_test (integration_test
    LAUNCH_FILE integration_tests.launch.yaml

    )

endif()

install(DIRECTORY
  launch
  
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
