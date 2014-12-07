cmake_minimum_required(VERSION 2.8.3)
project(ros_vision)
 
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  image_transport
  pcl_ros
  cv_bridge
)
 
add_message_files(
   FILES
   Filter.msg
   FilterList.msg
   FilterGroup.msg
   FilterGroupNames.msg
   FilterStatistics.msg
   IODescriptor.msg
   Parameter.msg
   PolygonListStamped.msg
   StartSignal.msg
   Statistics.msg
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
   GetParameterValue.srv
   GetWorkspace.srv
   ListFilterTypes.srv
   ListWorkspaces.srv
   LoadWorkspace.srv
   SaveWorkspace.srv
   SetParameterValue.srv
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
