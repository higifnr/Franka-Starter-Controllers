// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

namespace franka_starter_controllers {

class SimpleJointVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:

   // ROS subscriber for velocity commands
  ros::Subscriber joint_velocity_sub_;
  ros::Subscriber filter_param_sub_;


  // Callback for joint velocity commands
  void jointVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void filterParamCallback(const std_msgs::Float64::ConstPtr& msg);
  
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;
  

  // Storage for commanded velocities
  std::vector<double> commanded_velocities_;
  std::vector<double> output_velocities_;
  
  // Mutex for thread safety
  std::mutex velocity_mutex_;

  // low-pass filter parameter
  double alpha = 0.1;
};

}  // namespace franka_starter_controllers