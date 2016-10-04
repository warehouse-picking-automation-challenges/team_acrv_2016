cmake_minimum_required(VERSION 2.8.3)
project(apc_grasping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DBT_USE_DOUBLE_PRECISION -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  roscpp
  image_transport
  sensor_msgs
  message_filters
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf
  tf_conversions
  eigen_conversions
  apc_msgs
  std_srvs
  pcl_conversions
  pcl_ros
  cv_bridge
  apc_msgs
)

find_package(OpenMP)


if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomPCLConfig.cmake")
    set(PCL_CONFIGURED true)
    message(STATUS "pcl: cmake/CustomPCLConfig.cmake found, project will be built.")
else()
    set(PCL_CONFIGURED false)
    message(WARNING "pcl: cmake/CustomPCLConfig.cmake not found, project won't be built.")
endif()

if (${PCL_CONFIGURED})
    include(cmake/CustomPCLConfig.cmake)
    message(STATUS "pcl: Looking for PCL configuration file at " ${PCL_DIR})
    find_package(PCL CONFIG REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

## Generate added messages and services with any dependencies listed here
#  add_service_files(
#    FILES
#    grasp_pose.srv
#  )
#
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   apc_grasping
#   # message_generation
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS baxter_core_msgs roscpp image_transport message_filters
    moveit_core moveit_ros_planning moveit_ros_planning_interface
  INCLUDE_DIRS include
  #  LIBRARIES fruit_picking_test
  # DEPENDS system_lib
)

###########
## Build ##
###########
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
    ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(grasp_selection_service_node src/grasp_selection.cpp )
add_executable(object_publisher src/object_publisher.cpp src/Item.cpp)
add_executable(test_shelf_position src/test_shelf_position.cpp src/Item.cpp)
add_executable(update_endpoint_weight src/update_endpoint_weight.cpp)
add_executable(attach_object_to_robot src/attach_object_to_robot.cpp)
add_executable(baxter_torque_subscriber src/baxter_torque_subscriber.cpp)
add_executable(create_items src/create_items.cpp src/Item.cpp)
add_executable(fill_bins_and_tote src/fill_bins_and_tote.cpp)
add_executable(grasp_selection_test src/grasp_selection_test.cpp src/pcl_filters.cpp)
add_executable(test_rotational_weighting src/test_rotational_weighting.cpp)


add_dependencies(grasp_selection_service_node apc_msgs_generate_messages_cpp moveit_lib_generate_messages_cpp)
add_dependencies(object_publisher apc_msgs_generate_messages_cpp)
add_dependencies(test_shelf_position apc_msgs_generate_messages_cpp)
add_dependencies(update_endpoint_weight apc_msgs_generate_messages_cpp)
add_dependencies(attach_object_to_robot apc_msgs_generate_messages_cpp)
add_dependencies(baxter_torque_subscriber apc_msgs_generate_messages_cpp)
add_dependencies(create_items moveit_lib_generate_messages_cpp)
add_dependencies(fill_bins_and_tote apc_msgs_generate_messages_cpp)
add_dependencies(grasp_selection_test apc_msgs_generate_messages_cpp)
add_dependencies(test_rotational_weighting apc_msgs_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(grasp_selection_service_node
    ${catkin_LIBRARIES}
)
target_link_libraries(test_shelf_position
    ${catkin_LIBRARIES}
)
target_link_libraries(update_endpoint_weight
    ${catkin_LIBRARIES}
)
target_link_libraries(baxter_torque_subscriber
    ${catkin_LIBRARIES}
)
target_link_libraries(object_publisher
    ${catkin_LIBRARIES}
)
target_link_libraries(create_items
    ${catkin_LIBRARIES}
)
target_link_libraries(attach_object_to_robot
    ${catkin_LIBRARIES}
)

target_link_libraries(fill_bins_and_tote
    ${catkin_LIBRARIES}
)

target_link_libraries(grasp_selection_test
    ${catkin_LIBRARIES}
)
target_link_libraries(test_rotational_weighting
    ${catkin_LIBRARIES}
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS fruit_picking_test fruit_picking_test_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   control_scratch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fruit_picking_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
