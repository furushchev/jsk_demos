cmake_minimum_required(VERSION 2.8.3)
project(jsk_pr2_plugs)

find_package(catkin REQUIRED COMPONENTS
  jsk_pr2_startup
  jsk_recognition_msgs
  jsk_topic_tools
  message_filter
  nodelet
  pcl_msgs
  pr2eus
  roscpp
  rospy
  tf
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS pcl
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

function(jsk_pr2_plugs_nodelet _nodelet_cpp _nodelet_cls _nodelet_exc)
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_cls} ${_nodelet_exc}
    jsk_pr2_plugs_nodelet_src jsk_pr2_plugs_excs)
endfunction()

add_library(${PROJECT_NAME} SHARED ${jsk_pr2_plugs_nodelet_src})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ${jsk_pr2_plugs_excs}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

