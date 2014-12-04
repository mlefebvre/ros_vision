cmake_minimum_required(VERSION 2.8.3)
project(ros_vision)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  pcl_ros
  cv_bridge
  OpenCV REQUIRED
)

add_message_files(
   FILES
   Filter.msg
   FilterList.msg
   FilterGroup.msg
   FilterGroupNames.msg
   IODescriptor.msg
   Parameter.msg
   PolygonListStamped.msg
   StartSignal.msg
   Workspace.msg
 )

add_service_files(
   FILES
   AddInput.srv
   AddOutput.srv
   CreateFilter.srv
   CreateFilterGroup.srv
   DeleteFilter.srv
   DeleteFilterGroup.srv
   ListFilterTypes.srv
   ListWorkspaces.srv
   LoadWorkspace.srv
   SaveWorkspace.srv
 )

generate_messages(
   DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    ros_vision
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(openni_node src/openni_node.cpp)
target_link_libraries(openni_node ${catkin_LIBRARIES})

install(TARGETS openni_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
