cmake_minimum_required(VERSION 2.8.3)
project(vision_collision_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
    geometry_msgs
    cv_bridge
    fusion_msgs
    roscpp
    dynamic_reconfigure
)

find_package(cmake_modules REQUIRED)
find_package(OpenCV 2.4 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

generate_dynamic_reconfigure_options(
  cfg/fault_detection.cfg
)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    cv_bridge
    roscpp
    dynamic_reconfigure
)

add_executable(vision_collision_detector_node
               src/vision_collision_detector_node.cpp
               src/frame_container.cpp
               src/ros_matcher.cpp
               src/ros_fault_detection.cpp
               src/ros_statistics_tools.cpp
)

add_library(${PROJECT_NAME}
            src/vision_collision_detector_node.cpp
            src/frame_container.cpp
            src/ros_matcher.cpp
            src/ros_fault_detection.cpp
            src/ros_statistics_tools.cpp
)

add_dependencies(vision_collision_detector_node ${PROJECT_NAME}_gencfg)

target_link_libraries( vision_collision_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)
