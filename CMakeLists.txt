cmake_minimum_required(VERSION 2.8.3)
project(path_planner_3d)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  pcl_conversions
  octomap_ros
  octomap_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  costmap_2d)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planner_3d
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  DEPENDS octomap PCL
)

set(LINK_LIBS
  ${OCTOMAP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

#add_executable(rrtstar src/rrtstar.cpp)
#add_dependencies(rrtstar ${catkin_EXPORTED_TARGETS})
#target_link_libraries(rrtstar ${catkin_LIBRARIES})

add_executable(path_planner_3d_node src/path_planner.cpp src/rrtstar.cpp)
add_dependencies(path_planner_3d_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner_3d_node ${catkin_LIBRARIES})

add_executable(csv2Octomap_node src/csv2Octomap.cpp)
add_dependencies(csv2Octomap_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(csv2Octomap_node ${catkin_LIBRARIES})

add_executable(fakeLocalization_node src/fake_localization_node.cpp)
add_dependencies(fakeLocalization_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(fakeLocalization_node ${catkin_LIBRARIES})

add_executable(kobuki_controller_node src/kobuki_controller.cpp)
add_dependencies(kobuki_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(kobuki_controller_node ${catkin_LIBRARIES})


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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_path_planner_3d.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
