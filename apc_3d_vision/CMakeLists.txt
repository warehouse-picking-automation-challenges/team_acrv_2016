cmake_minimum_required(VERSION 2.8.3)
project(apc_3d_vision)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DBT_USE_DOUBLE_PRECISION -Wall")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    baxter_core_msgs
    roscpp
    rospy
    std_msgs
    geometry_msgs
    image_transport
    sensor_msgs
    shape_msgs
    cv_bridge
    message_filters
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf
    tf_conversions
    eigen_conversions
    message_generation
)

find_package(OpenCV 2 REQUIRED)
find_package(OpenMP)
# find_package(Eigen REQUIRED)  // Eigen is not a ROS (Catkin) package but PCL is built as one
# find_package(PCL REQUIRED)

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
find_package(PCL 1.8 REQUIRED)

# find_package(PCL 1.8 REQUIRED)

if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_service_files(
    FILES
    split_labelled_point_cloud.srv
    fit_cad_models.srv
    shelf_scan_bin.srv
)

generate_messages(
    DEPENDENCIES
    sensor_msgs
    shape_msgs
    std_msgs
    geometry_msgs
    apc_3d_vision
)

catkin_package(
    CATKIN_DEPENDS baxter_core_msgs
    cv_bridge
    roscpp
    image_transport
    message_filters
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    tf
    tf_conversions
    eigen_conversions
    INCLUDE_DIRS include
    #  LIBRARIES fruit_picking_test
    # DEPENDS system_lib
    DEPENDS Eigen
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
    ${OpenCV_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(shelf_registration_ros_node src/shelf_registration_ros_node.cpp src/apc_3d_vision.cpp)
add_executable(float_objects_ros_node src/float_objects_ros_node.cpp src/apc_3d_vision.cpp)
# add_executable(match_objects_ros_node src/match_objects_ros_node.cpp src/apc_3d_vision.cpp)
add_executable(split_labelled_point_cloud_ros_node src/split_labelled_point_cloud_ros_node.cpp src/apc_3d_vision.cpp)
add_dependencies(split_labelled_point_cloud_ros_node apc_msgs_generate_messages_cpp)
add_executable(fit_cad_models_ros_node src/fit_cad_models_ros_node.cpp src/apc_3d_vision.cpp)
add_dependencies(fit_cad_models_ros_node apc_msgs_generate_messages_cpp)
add_executable(shelf_scan src/shelf_scan_node.cpp src/shelf_scan.cpp )
add_dependencies(shelf_scan apc_3d_vision_generate_messages_cpp)
add_dependencies(shelf_scan moveit_lib_generate_messages_cpp)

add_executable(align_prerejective_test src/align_prerejective_test.cpp src/apc_3d_vision.cpp)
add_executable(align_pca_ndt_test src/align_pca_ndt_test.cpp src/apc_3d_vision.cpp)
add_executable(align_pca_icp_test src/align_pca_icp_test.cpp src/apc_3d_vision.cpp)
add_executable(test_service_nodes src/test_service_nodes.cpp src/apc_3d_vision.cpp)
add_dependencies(test_service_nodes apc_msgs_generate_messages_cpp)
add_dependencies(test_service_nodes segmentation_ros_generate_messages_cpp)
add_dependencies(test_service_nodes ${catkin_EXPORTED_TARGETS})
add_dependencies(test_service_nodes moveit_lib_generate_messages_cpp)
# add_dependencies(test_service_nodes segmentation_ros_generate_messages_cpp)


add_executable(manual_point_cloud_labeller src/manual_point_cloud_labeller.cpp src/apc_3d_vision.cpp)
add_executable(test_publish_stl src/test_publish_stl.cpp)
add_executable(pcd2ply src/pcd2ply.cpp)
add_executable(grab_points src/grab_points.cpp)
add_executable(shelf_based_image_cropper_node src/shelf_based_image_cropper_node.cpp)
add_dependencies(shelf_based_image_cropper_node apc_msgs_generate_messages_cpp)
add_executable(shelf_based_image_cropper_node_james src/shelf_based_image_cropper_node_james.cpp)
# add_executable(cropTote src/cropTote.cpp src/apc_3d_vision.cpp)
add_dependencies(shelf_based_image_cropper_node_james apc_msgs_generate_messages_cpp)



# add_dependencies(match_objects_ros_node ${catkin_EXPORTED_TARGETS})
# add_dependencies(test_service_nodes ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(shelf_registration_ros_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Eigen_Libraries}
)
target_link_libraries(float_objects_ros_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
# target_link_libraries(match_objects_ros_node
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES}
# )
target_link_libraries(split_labelled_point_cloud_ros_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(fit_cad_models_ros_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)

target_link_libraries(align_prerejective_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(align_pca_ndt_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(align_pca_icp_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(test_service_nodes
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(manual_point_cloud_labeller
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Eigen_Libraries}
    yaml-cpp
)
target_link_libraries(test_publish_stl
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(pcd2ply
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(grab_points
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
)
target_link_libraries(shelf_based_image_cropper_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Eigen_Libraries}
    -ludev
)
target_link_libraries(shelf_based_image_cropper_node_james
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    ${Eigen_Libraries}
    -ludev
)
# target_link_libraries(cropTote
#     ${catkin_LIBRARIES}
#     ${OpenCV_LIBS}
#     ${PCL_LIBRARIES}
#     ${Eigen_Libraries}
#     # -ludev
# )

target_link_libraries(shelf_scan
    ${roscpp_LIBRARIES}
    ${catkin_LIBRARIES}
)

#target_link_libraries(depth_viewer
#    ${catkin_LIBRARIES}
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#)

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

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  scripts/shelf_republisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
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
