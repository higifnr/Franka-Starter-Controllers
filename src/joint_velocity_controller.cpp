// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <franka_starter_controllers/joint_velocity_controller.h>

#include <cmath>
#include <mutex>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_starter_controllers {

bool JointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                  ros::NodeHandle& node_handle) {
  velocity_joint_interface_ =
      robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR("JointVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointVelocityController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] =
          velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  commanded_velocities_.assign(7, 0.0);
  output_velocities_.assign(7, 0.0);

  joint_velocity_sub_ = node_handle.subscribe(
      "joint_velocity_commands", 1,
      &JointVelocityController::jointVelocityCallback, this);

  filter_param_sub_ = node_handle.subscribe(
      "joint_velocity_filter", 1,
      &JointVelocityController::filterParamCallback, this);

  ROS_INFO("JointVelocityController: Initialized successfully");
  ROS_INFO("JointVelocityController: Listening for velocity commands on topic: joint_velocity_commands");

  return true;
}

void JointVelocityController::jointVelocityCallback(
    const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() != 7) {
    ROS_WARN_STREAM("JointVelocityController: Received velocity command with "
                    << msg->data.size() << " values, expected 7. Ignoring command.");
    return;
  }

  std::lock_guard<std::mutex> lock(velocity_mutex_);
  for (size_t i = 0; i < 7; ++i) {
    commanded_velocities_[i] = msg->data[i];
  }

  last_command_time_ = ros::Time::now();
}

void JointVelocityController::filterParamCallback(
    const std_msgs::Float64::ConstPtr& msg) {
  alpha = msg->data;
  ROS_INFO("JointVelocityController: changed joint low pass filter parameter to %.3f", alpha);
}

void JointVelocityController::starting(const ros::Time& time) {
  elapsed_time_ = ros::Duration(0.0);

  std::lock_guard<std::mutex> lock(velocity_mutex_);
  std::fill(commanded_velocities_.begin(), commanded_velocities_.end(), 0.0);
  std::fill(output_velocities_.begin(), output_velocities_.end(), 0.0);

  // Start in idle state
  last_command_time_ = time - command_timeout_;

  ROS_INFO("JointVelocityController: Controller started");
}

void JointVelocityController::update(const ros::Time& time,
                                     const ros::Duration& period) {
  elapsed_time_ += period;

  std::lock_guard<std::mutex> lock(velocity_mutex_);

  // WATCHDOG: no recent command → do nothing
  if ((time - last_command_time_) > command_timeout_) {
    return;
  }

  for (size_t i = 0; i < output_velocities_.size(); ++i) {
    output_velocities_[i] =
        (1.0 - alpha) * output_velocities_[i] +
        alpha * commanded_velocities_[i];
  }

  for (size_t i = 0; i < velocity_joint_handles_.size(); ++i) {
    velocity_joint_handles_[i].setCommand(output_velocities_[i]);
  }
}

void JointVelocityController::stopping(const ros::Time&) {
  ROS_INFO("JointVelocityController: Controller stopped");
}

}  // namespace franka_starter_controllers

PLUGINLIB_EXPORT_CLASS(franka_starter_controllers::JointVelocityController,
                       controller_interface::ControllerBase)
