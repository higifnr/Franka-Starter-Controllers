// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_starter_controllers/simple_joint_velocity_controller.h>

#include <cmath>
#include <mutex>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_starter_controllers {

bool SimpleJointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "SimpleJointVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("SimpleJointVelocityController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("SimpleJointVelocityController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("SimpleJointVelocityController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "SimpleJointVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Initialize commanded velocities to zero
  commanded_velocities_.resize(7, 0.0);
  output_velocities_.resize(7, 0.0);


    try {
      auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
      auto state_handle = state_interface->getHandle(arm_id + "_robot");
      
      std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      for (size_t i = 0; i < q_start.size(); i++) {
        if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
          ROS_WARN_STREAM(
              "SimpleJointVelocityController: Robot is not in the expected starting position for "
              "running this example. Run `roslaunch franka_starter_controllers move_to_start.launch "
              "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        }
      }
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "SimpleJointVelocityController: Exception getting state handle: " << e.what());
      return false;
    }

  // Set up subscriber for joint velocity commands
  joint_velocity_sub_ = node_handle.subscribe(
      "joint_velocity_commands", 1, 
      &SimpleJointVelocityController::jointVelocityCallback, this);

  filter_param_sub_ = node_handle.subscribe(
      "joint_velocity_filter", 1, 
      &SimpleJointVelocityController::filterParamCallback, this);

  ROS_INFO("SimpleJointVelocityController: Initialized successfully");
  ROS_INFO("SimpleJointVelocityController: Listening for velocity commands on topic: joint_velocity_commands");
  
  return true;
}

void SimpleJointVelocityController::jointVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() != 7) {
    ROS_WARN_STREAM("SimpleJointVelocityController: Received velocity command with " 
                    << msg->data.size() << " values, expected 7. Ignoring command.");
    return;
  }

  std::lock_guard<std::mutex> lock(velocity_mutex_);
  for (size_t i = 0; i < 7; ++i) {
    commanded_velocities_[i] = msg->data[i];
  }
}

void SimpleJointVelocityController::filterParamCallback(const std_msgs::Float64::ConstPtr& msg) {
  alpha = msg->data;
  ROS_INFO("SimpleJointVelocityController: changed joint low pass filter parameter to %.3f", alpha);
}

void SimpleJointVelocityController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  
  // Initialize commanded velocities to zero
  std::lock_guard<std::mutex> lock(velocity_mutex_);
  std::fill(commanded_velocities_.begin(), commanded_velocities_.end(), 0.0);
  
  ROS_INFO("SimpleJointVelocityController: Controller started");
}

void SimpleJointVelocityController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  // Apply the commanded velocities to each joint
  std::lock_guard<std::mutex> lock(velocity_mutex_);

  if (output_velocities_.size() == commanded_velocities_.size()) {
    for (size_t i = 0; i < output_velocities_.size(); ++i) {
      output_velocities_[i] = (1.0 - alpha) * output_velocities_[i] + alpha * commanded_velocities_[i];
    }
} else {
  ROS_WARN("Size mismatch: output_velocities_ has %zu elements, commanded_velocities_ has %zu elements",
           output_velocities_.size(), commanded_velocities_.size());
}

  for (size_t i = 0; i < velocity_joint_handles_.size(); ++i) {
    velocity_joint_handles_[i].setCommand(output_velocities_[i]);
  }
}

void SimpleJointVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  ROS_INFO("SimpleJointVelocityController: Controller stopped");
}

}  // namespace franka_starter_controllers

PLUGINLIB_EXPORT_CLASS(franka_starter_controllers::SimpleJointVelocityController,
                       controller_interface::ControllerBase)