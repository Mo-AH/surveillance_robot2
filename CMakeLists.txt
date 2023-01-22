cmake_minimum_required(VERSION 3.0.2)
project(surveillance_robot2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  aruco
  aruco_msgs
  sensor_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)
find_package(aruco_ros REQUIRED)


## Uncomment this if the package has a setup.py.
catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   RoomConnection.msg
   Room.msg
 )

## Generate services in the 'srv' folder
add_service_files(
   FILES
   RoomInformation.srv
 )

## Generate actions in the 'action' folder
add_action_files(
    FILES
    ArmControl.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)


catkin_package(
  LIBRARIES aruco_ros_utils
  CATKIN_DEPENDS roscpp rospy aruco 
  DEPENDS OpenCV
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${aruco_ros_INCLUDE_DIRS}
)


add_executable(marker_server src/marker_server.cpp)
add_dependencies(marker_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_server
   ${catkin_LIBRARIES}
)

add_executable(marker_detector src/marker_detector.cpp)
add_dependencies(marker_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_detector
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  ${aruco_LIBS}
)

catkin_install_python(PROGRAMS
   scripts/arm_controller.py
   scripts/robot_state.py
   scripts/smach_robot.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
