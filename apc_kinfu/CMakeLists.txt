cmake_minimum_required(VERSION 2.8.3)
project(ros_kinfu)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  image_transport
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  pcl_ros
  eigen_conversions
  tf_conversions
  apc_msgs
)
set(OpenCV_INCLUDE_DIR "/usr/local/include/opencv2/ /usr/local/include/opencv/")
set(OpenCV_LIBS "/usr/local/lib/")
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/CustomPCLConfig.cmake")
  set(KINFU_CONFIGURED true)
  message(STATUS "kinfu: cmake/CustomPCLConfig.cmake found, kinfu will be built.")
else()
  set(KINFU_CONFIGURED false)
  message(WARNING "kinfu: cmake/CustomPCLConfig.cmake not found, kinfu won't be built.")
endif()
find_package(VTK REQUIRED)
if (${KINFU_CONFIGURED})
  include(cmake/CustomPCLConfig.cmake)
  message(STATUS "kinfu: Looking for PCL configuration file at " ${PCL_DIR})
  find_package(PCL CONFIG REQUIRED)
  include_directories(${PCL_INCLUDE_DIRS})
  message(STATUS "kinfu: Looking for PCL configuration file at " ${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  find_package(CUDA REQUIRED)

  if (CUDA_FOUND)
      message(" * CUDA ${CUDA_VERSION} was found")
      include_directories(${CUDA_INCLUDE_DIRS})
  else(CUDA_FOUND)
      message(" * CUDA is not found")
      message(FATAL_ERROR "Not all CUDA libraries are found")
  endif(CUDA_FOUND)
endif()

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs image_transport message_filters roscpp sensor_msgs std_msgs tf cv_bridge  pcl_ros  eigen_conversions tf_conversions
)

###########
## Build ##
###########

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/apc_ws
  ${OpenCV_INCLUDE_DIR}
)

if (${KINFU_CONFIGURED})
  add_executable(ros_kinfu
    src/ros_kinfu.cpp src/kinfu_viz_tools.cpp src/kinfu_tools.cpp src/ros_kinfu_publisher.cpp
    )
  add_dependencies(ros_kinfu apc_msgs_generate_messages_cpp)

  target_link_libraries(ros_kinfu
    boost_system boost_signals
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
  )
endif()

#############
## Install ##
#############

#############
## Testing ##
#############
