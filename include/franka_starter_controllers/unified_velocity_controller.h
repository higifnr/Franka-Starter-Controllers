// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <algorithm>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>

namespace franka_starter_controllers {

enum class ControlMode {
  JOINT_VELOCITY,
  CARTESIAN_VELOCITY,
  IDLE
};

class UnifiedVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaModelInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  // Callback functions
  void jointVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  void cartesianVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void filterParamCallback(const std_msgs::Float64::ConstPtr& msg);
  
  // Helper functions
  void updateJointVelocityControl();
  void updateCartesianVelocityControl();
  void setAllJointsToZero();
  
  // Hardware interfaces
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  franka_hw::FrankaModelInterface* model_interface_;
  franka_hw::FrankaStateInterface* state_interface_;
  
  // Handles
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  
  // ROS subscribers
  ros::Subscriber joint_velocity_sub_;
  ros::Subscriber cartesian_velocity_sub_;
  ros::Subscriber filter_param_sub_;
  
  // Control mode management
  ControlMode current_mode_;
  ros::Time last_joint_command_time_;
  ros::Time last_cartesian_command_time_;
  double command_timeout_; // seconds
  
  // Joint velocity control data
  std::vector<double> commanded_joint_velocities_;
  std::vector<double> output_joint_velocities_;
  
  // Cartesian velocity control data
  geometry_msgs::Twist commanded_cartesian_twist_;
  
  // Parameters
  std::string arm_id_;
  std::vector<std::string> joint_names_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double damping_factor_;
  
  // Thread safety
  std::mutex control_mutex_;
  
  // Timing
  ros::Duration elapsed_time_;

  // low-pass filter parameter
  double alpha = 0.1;
};

}  // namespace franka_starter_controllers