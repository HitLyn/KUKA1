cmake_minimum_required(VERSION 2.8.3)
project(agent_ros_robot)


find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
   sensor_msgs
   geometry_msgs
   trajectory_msgs
   control_msgs
   moveit_msgs
   ros_fri_msgs
   actionlib_msgs
   actionlib
   tf
   tf_conversions
   tf2_bullet

   moveit_core
   moveit_ros_planning
   moveit_ros_planning_interface
   pluginlib
   cmake_modules
   geometric_shapes
   wsg_50_common

)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)
catkin_python_setup()


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PositionCommand.msg
   GripperCommand.msg
   SampleResult.msg
   DataRequest.msg
   num.msg
   objCommand.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agent_ros_robot
   CATKIN_DEPENDS
     message_runtime
     std_msgs
     sensor_msgs
     geometry_msgs
     trajectory_msgs
     control_msgs
     actionlib_msgs
     actionlib
     ros_fri_msgs
     moveit_ros_planning_interface
     moveit_core
     interactive_markers
#  DEPENDS system_lib
)
###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  #/opt/ReflexxesTypeII/include
  ${PROJECT_SOURCE_DIR}/kukalib
)

link_directories(
  lib
  ${catkin_LIBRARIES}
  #/opt/ReflexxesTypeII/Linux/x64/release/lib/static
  ${PROJECT_SOURCE_DIR}/kukalib
)

#add_executable(kukaplugin_yuchen src/kukaplugin_yuchen.cpp)
#add_dependencies(kukaplugin_yuchen ${catkin_EXPORTED_TARGETS} )
#target_link_libraries(kukaplugin ${catkin_LIBRARIES} FastResearchInterfaceLibrary rt  pthread) # why is not ok??
#target_link_libraries(kukaplugin_yuchen ReflexxesTypeII ${catkin_LIBRARIES} fril_test_dynamic fril_dynamic fril_irml_dynamic)

add_executable(runKUKA_fri src/runKUKA_fri.cpp)
add_dependencies(runKUKA_fri ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_fri ${catkin_LIBRARIES} ${Boost_LIBRARIES} FastResearchInterfaceLibrary rt  pthread)

add_executable(runKUKA_fri_msg src/runKUKA_fri_msg.cpp)
add_dependencies(runKUKA_fri_msg ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_fri_msg ${catkin_LIBRARIES} ${Boost_LIBRARIES} FastResearchInterfaceLibrary rt  pthread)

add_executable(runKUKA_fri_msg_V2 src/runKUKA_fri_msg_V2.cpp)
add_dependencies(runKUKA_fri_msg_V2 ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_fri_msg_V2 ${catkin_LIBRARIES} ${Boost_LIBRARIES} FastResearchInterfaceLibrary rt  pthread)

#2017.05.20
add_executable(runKUKA_fri_msg_force src/runKUKA_fri_msg_force.cpp)
add_dependencies(runKUKA_fri_msg_force ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_fri_msg_force ${catkin_LIBRARIES} ${Boost_LIBRARIES} FastResearchInterfaceLibrary rt  pthread)

# 2017.06.20
add_executable(runKUKA_fri_control src/runKUKA_fri_control.cpp)
add_dependencies(runKUKA_fri_control ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_fri_control ${catkin_LIBRARIES} ${Boost_LIBRARIES} FastResearchInterfaceLibrary rt  pthread)

add_executable(runKUKA_fri_data src/runKUKA_fri_data.cpp)
add_dependencies(runKUKA_fri_data ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_fri_data ${catkin_LIBRARIES} ${Boost_LIBRARIES} FastResearchInterfaceLibrary rt  pthread)

add_executable(runKUKA_fri_gripper src/runKUKA_fri_gripper.cpp)
add_dependencies(runKUKA_fri_gripper ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_fri_gripper ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# 2017.11.12
add_executable(spawnObj_Scene src/spawnObj_Scene.cpp)
add_dependencies(spawnObj_Scene ${catkin_EXPORTED_TARGETS} )
target_link_libraries(spawnObj_Scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#
add_executable(runKUKA_actionlib src/runKUKA_actionlib.cpp)
add_dependencies(runKUKA_actionlib ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_actionlib ${catkin_LIBRARIES} )

add_executable(runKUKA_actionlib_V2 src/runKUKA_actionlib_V2.cpp)
add_dependencies(runKUKA_actionlib_V2 ${catkin_EXPORTED_TARGETS} )
target_link_libraries(runKUKA_actionlib_V2 ${catkin_LIBRARIES} )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/agent_ros_robot.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/agent_ros_robot_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_agent_ros_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
