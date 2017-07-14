
cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

find_package(catkin REQUIRED
    COMPONENTS
    geometry_msgs
    roscpp
)

find_package(cmake_modules REQUIRED)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

set(CMAKE_CXX_FLAGS "-std=c++11")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
)

add_library(${PROJECT_NAME}
  src/vision_utils_ros_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
