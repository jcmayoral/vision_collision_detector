cmake_minimum_required(VERSION 2.8.3)
project(vision_utils_ros)

find_package(catkin REQUIRED
    COMPONENTS
    geometry_msgs
    cv_bridge
    roscpp
)

find_package(cmake_modules REQUIRED)
find_package(OpenCV 3.2.0 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    cv_bridge
    roscpp
)

add_executable(vision_utils_ros
               src/vision_utils_ros_node.cpp
               src/FD.cpp
               src/Matcher.cpp
               src/tracker.cpp
               src/myfeaturedetection.cpp
               src/statics_tools.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
