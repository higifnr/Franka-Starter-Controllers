// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_starter_controllers/cartesian_velocity_controller.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <mutex>
#include <cmath>

namespace franka_starter_controllers {

bool CartesianVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR("CartesianVelocityController: Could not get Cartesian velocity interface from hardware");
    return false;
  }

  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianVelocityController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_WARN_STREAM("CartesianVelocityController: Robot is not in the expected starting position.");
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianVelocityController: Exception getting state handle: " << e.what());
    return false;
  }

  // Initialize smoothed values
  delta_x = delta_y = delta_z = 0.0;
  theta_x = theta_y = theta_z = 0.0;

  // Initialize raw command values
  cmd_tx = cmd_ty = cmd_tz = 0.0;
  cmd_rx = cmd_ry = cmd_rz = 0.0;

  // Initialize previous velocities for acceleration limiting
  prev_delta_x = prev_delta_y = prev_delta_z = 0.0;
  prev_theta_x = prev_theta_y = prev_theta_z = 0.0;

  // Maximum acceleration (m/s² for linear, rad/s² for angular)
  max_linear_accel = 2.0;  // 1 m/s²
  max_angular_accel = 0.8; // 2 rad/s² (adjust as needed)

  // Subscriber for velocity commands
  command_sub_ = node_handle.subscribe("cartesian_velocity_command", 1,
      &CartesianVelocityController::twistCallback, this);

  return true;
}

void CartesianVelocityController::twistCallback(const geometry_msgs::TwistConstPtr& msg) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  cmd_tx = msg->linear.x;
  cmd_ty = msg->linear.y;
  cmd_tz = msg->linear.z;
  cmd_rx = msg->angular.x;
  cmd_ry = msg->angular.y;
  cmd_rz = msg->angular.z;
}

void CartesianVelocityController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  // Reset previous velocities
  prev_delta_x = prev_delta_y = prev_delta_z = 0.0;
  prev_theta_x = prev_theta_y = prev_theta_z = 0.0;
  delta_x = delta_y = delta_z = 0.0;
  theta_x = theta_y = theta_z = 0.0;
}

void CartesianVelocityController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;
  double dt = period.toSec();

  // Simple low-pass filtering
  alpha = 0.01;
  double filtered_tx = (1-alpha) * delta_x + alpha * cmd_tx;
  double filtered_ty = (1-alpha) * delta_y + alpha * cmd_ty;
  double filtered_tz = (1-alpha) * delta_z + alpha * cmd_tz;
  double filtered_rx = (1-alpha) * theta_x + alpha * cmd_rx;
  double filtered_ry = (1-alpha) * theta_y + alpha * cmd_ry;
  double filtered_rz = (1-alpha) * theta_z + alpha * cmd_rz;

  // Acceleration clamping for linear velocities
  delta_x = clampAcceleration(filtered_tx, prev_delta_x, max_linear_accel, dt);
  delta_y = clampAcceleration(filtered_ty, prev_delta_y, max_linear_accel, dt);
  delta_z = clampAcceleration(filtered_tz, prev_delta_z, max_linear_accel, dt);

  // Acceleration clamping for angular velocities
  theta_x = clampAcceleration(filtered_rx, prev_theta_x, max_angular_accel, dt);
  theta_y = clampAcceleration(filtered_ry, prev_theta_y, max_angular_accel, dt);
  theta_z = clampAcceleration(filtered_rz, prev_theta_z, max_angular_accel, dt);

  // Store current velocities for next iteration
  prev_delta_x = delta_x;
  prev_delta_y = delta_y;
  prev_delta_z = delta_z;
  prev_theta_x = theta_x;
  prev_theta_y = theta_y;
  prev_theta_z = theta_z;

  std::array<double, 6> command = {{delta_x, delta_y, delta_z, theta_x, theta_y, theta_z}};
  velocity_cartesian_handle_->setCommand(command);
}

double CartesianVelocityController::clampAcceleration(double desired_vel, double current_vel,
                                                       double max_accel, double dt) {
  // Calculate the change in velocity
  double dv = desired_vel - current_vel;
  
  // Calculate maximum allowed velocity change based on acceleration limit
  double max_dv = max_accel * dt;
  
  // Clamp the velocity change
  if (dv > max_dv) {
    return current_vel + max_dv;
  } else if (dv < -max_dv) {
    return current_vel - max_dv;
  } else {
    return desired_vel;
  }
}

void CartesianVelocityController::stopping(const ros::Time& /*time*/) {
  // Let built-in stopping behavior handle deceleration.
}

}  // namespace franka_starter_controllers

PLUGINLIB_EXPORT_CLASS(franka_starter_controllers::CartesianVelocityController,
                       controller_interface::ControllerBase)