cmake_minimum_required(VERSION 2.8.3)
project(jsk_2017_home_butler)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  message_generation
  std_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files()

## Generate services in the 'srv' folder
add_service_files()

## Generate actions in the 'action' folder
add_action_files(FILES
  AnswerQuestion.action
  FindPerson.action
  PickObject.action
  Deliver.action
  GoTo.action
  PlaceObject.action
  FindObject.action
  Speak.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package()

