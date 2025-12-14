cmake_minimum_required(VERSION 3.0.2)
project(franka_starter_controllers)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  franka_hw
  geometry_msgs
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  rospy
  eigen_conversions
  #dynamic_reconfigure
)

# Dynamic reconfigure is planned, urgency depends on advantages compared to topics 
#generate_dynamic_reconfigure_options(
#  cfg/joint_velocity_controller_config.cfg
#)

catkin_package(
  INCLUDE_DIRS include 

  LIBRARIES cartesian_velocity_controller 
  joint_velocity_controller 
  unified_velocity_controller 
  simple_joint_velocity_controller

  CATKIN_DEPENDS roscpp 
  rospy 
  pluginlib 
  controller_interface
  hardware_interface 
  franka_hw 
  geometry_msgs 
  realtime_tools 
  controller_manager 
  eigen_conversions
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#adding all controller under new library
#this library is going to be used by ROS to build controllers via franka_starter_controllers.xml
#franka_starter_controllers.xml is called a "plugin" file
#I dropped "plugin" from the name because HAVING 50 DIFFERENT NAMES (plugins, configs, cfg...) is too much for beginners
add_library(franka_starter_controllers
  src/unified_velocity_controller.cpp
  src/simple_joint_velocity_controller.cpp
  src/joint_velocity_controller.cpp
  src/cartesian_velocity_controller.cpp)

target_link_libraries(franka_starter_controllers
   ${catkin_LIBRARIES})

add_dependencies(franka_starter_controllers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#adding plugin list
install(FILES franka_starter_controllers.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/franka_starter_controllers.xml
     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})

#installing python nodes
catkin_install_python(PROGRAMS
  scripts/gui_tools/joint_vel_control_panel.py
  scripts/gui_tools/cart_vel_control_panel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#perhaps use this in the future (not optimized for ros)
#install(PROGRAMS scripts/*.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

