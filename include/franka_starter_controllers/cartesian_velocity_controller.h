// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <mutex>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>

namespace franka_starter_controllers {

class CartesianVelocityController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;

 private:
  // Hardware interfaces
  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;

  // ROS
  ros::Subscriber command_sub_;

  // Command callback
  void twistCallback(const geometry_msgs::TwistConstPtr& msg);

  // Smoothing filter state
  double theta_x = 0.0;
  double theta_y = 0.0;
  double theta_z = 0.0;
  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_z = 0.0;

  // Latest commanded twist values
  double cmd_tx = 0.0;
  double cmd_ty = 0.0;
  double cmd_tz = 0.0;
  double cmd_rx = 0.0;
  double cmd_ry = 0.0;
  double cmd_rz = 0.0;
  double alpha = 0.1;

  // Previous velocity values for acceleration limiting
  double prev_delta_x, prev_delta_y, prev_delta_z;
  double prev_theta_x, prev_theta_y, prev_theta_z;
  
  // Acceleration limits
  double max_linear_accel = 1.0;
  double max_angular_accel = 1.0;
  
  // Helper method for acceleration clamping
  double clampAcceleration(double desired_vel, double current_vel, 
                          double max_accel, double dt);

  std::mutex command_mutex_;  // Protects access to cmd_* variables

  ros::Duration elapsed_time_;
};

}  // namespace franka_starter_controllers
