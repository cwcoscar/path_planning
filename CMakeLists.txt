cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  rospy
  roscpp
  std_msgs
  tf
)
set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/spot.h
    )

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(astar ${catkin_LIBRARIES})
add_dependencies(astar
  ${catkin_EXPORTED_TARGETS}
)

add_executable(tf_broadcast src/tf_broadcast.cpp ${HEADERS} ${SOURCES})
target_link_libraries(tf_broadcast ${catkin_LIBRARIES})
add_dependencies(tf_broadcast
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
    astar
    tf_broadcast
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# catkin_install_python(PROGRAMS
#    scripts/.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


