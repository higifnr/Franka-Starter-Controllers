// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_starter_controllers/unified_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_starter_controllers {

bool UnifiedVelocityController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) {
  // Get velocity joint interface
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR("UnifiedVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }
  
  // Get model interface (for Cartesian control)
  model_interface_ = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR("UnifiedVelocityController: Error getting model interface from hardware!");
    return false;
  }
  
  // Get state interface
  state_interface_ = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface_ == nullptr) {
    ROS_ERROR("UnifiedVelocityController: Error getting state interface from hardware!");
    return false;
  }

  // Load parameters
  if (!node_handle.getParam("arm_id", arm_id_)) {
    ROS_ERROR("UnifiedVelocityController: Could not get parameter arm_id");
    return false;
  }

  // Load joint names
  if (!node_handle.getParam("joint_names", joint_names_)) {
    ROS_ERROR("UnifiedVelocityController: Could not parse joint names");
    return false;
  }
  if (joint_names_.size() != 7) {
    ROS_ERROR_STREAM("UnifiedVelocityController: Wrong number of joint names, got "
                     << joint_names_.size() << " instead of 7 names!");
    return false;
  }
  
  // Load Cartesian velocity limits
  node_handle.param("max_linear_velocity", max_linear_velocity_, 1.0);   // m/s
  node_handle.param("max_angular_velocity", max_angular_velocity_, 3.14); // rad/s
  node_handle.param("damping_factor", damping_factor_, 0.1);
  node_handle.param("command_timeout", command_timeout_, 0.2); // 200ms timeout
  
  // Initialize joint handles
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names_[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("UnifiedVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Initialize model and state handles
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface_->getHandle(arm_id_ + "_model"));
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface_->getHandle(arm_id_ + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("UnifiedVelocityController: Exception getting model/state handles: " << ex.what());
    return false;
  }

  // Initialize commanded velocities
  commanded_joint_velocities_.resize(7, 0.0);
  commanded_cartesian_twist_ = geometry_msgs::Twist(); // Zero twist

  // Check robot's initial position
    try {
    auto state_handle = state_interface_->getHandle(arm_id_ + "_robot");

    std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_WARN_STREAM("CartesianVelocityController: Robot is not in the standard starting position.");
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("CartesianVelocityController: Exception getting state handle: " << e.what());
    return false;
  }
  // Set up subscribers
  joint_velocity_sub_ = node_handle.subscribe(
      "joint_velocity_commands", 1, 
      &UnifiedVelocityController::jointVelocityCallback, this);
      
  cartesian_velocity_sub_ = node_handle.subscribe(
      "cartesian_velocity_commands", 1,
      &UnifiedVelocityController::cartesianVelocityCallback, this);

  filter_param_sub_ = node_handle.subscribe(
    "joint_velocity_filter", 1, 
    &UnifiedVelocityController::filterParamCallback, this);

  // Initialize control mode
  current_mode_ = ControlMode::IDLE;
  last_joint_command_time_ = ros::Time(0);
  last_cartesian_command_time_ = ros::Time(0);

  ROS_INFO("UnifiedVelocityController: Initialized successfully");
  ROS_INFO("UnifiedVelocityController: Listening for joint velocity commands on: joint_velocity_commands");
  ROS_INFO("UnifiedVelocityController: Listening for Cartesian velocity commands on: cartesian_velocity_commands");
  ROS_INFO_STREAM("UnifiedVelocityController: Command timeout set to " << command_timeout_ << " seconds");
  
  return true;
}

void UnifiedVelocityController::jointVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  if (msg->data.size() != 7) {
    ROS_WARN_STREAM("UnifiedVelocityController: Received joint velocity command with " 
                    << msg->data.size() << " values, expected 7. Ignoring command.");
    return;
  }

  std::lock_guard<std::mutex> lock(control_mutex_);
  for (size_t i = 0; i < 7; ++i) {
    commanded_joint_velocities_[i] = msg->data[i];
  }
  last_joint_command_time_ = ros::Time::now();
  
  // Switch to joint velocity mode if this is the most recent command
  if (last_joint_command_time_ > last_cartesian_command_time_) {
    if (current_mode_ != ControlMode::JOINT_VELOCITY) {
      current_mode_ = ControlMode::JOINT_VELOCITY;
      ROS_INFO("UnifiedVelocityController: Switched to JOINT_VELOCITY mode");
    }
  }
}

void UnifiedVelocityController::cartesianVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(control_mutex_);
  commanded_cartesian_twist_ = *msg;
  last_cartesian_command_time_ = ros::Time::now();
  
  // Switch to Cartesian velocity mode if this is the most recent command
  if (last_cartesian_command_time_ > last_joint_command_time_) {
    if (current_mode_ != ControlMode::CARTESIAN_VELOCITY) {
      current_mode_ = ControlMode::CARTESIAN_VELOCITY;
      ROS_INFO("UnifiedVelocityController: Switched to CARTESIAN_VELOCITY mode");
    }
  }
}


void UnifiedVelocityController::filterParamCallback(const std_msgs::Float64::ConstPtr& msg) {
  alpha = msg->data;
  ROS_INFO("UnifiedVelocityController: changed joint low pass filter parameter to %.3f", alpha);
}

void UnifiedVelocityController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  
  std::lock_guard<std::mutex> lock(control_mutex_);
  
  // Initialize commanded velocities to zero
  std::fill(commanded_joint_velocities_.begin(), commanded_joint_velocities_.end(), 0.0);
  output_joint_velocities_.resize(7, 0.0);
  commanded_cartesian_twist_ = geometry_msgs::Twist();
  
  // Start in idle mode
  current_mode_ = ControlMode::IDLE;
  last_joint_command_time_ = ros::Time(0);
  last_cartesian_command_time_ = ros::Time(0);
  
  ROS_INFO("UnifiedVelocityController: Controller started in IDLE mode");
}

void UnifiedVelocityController::update(const ros::Time& current_time,
                                      const ros::Duration& period) {
  elapsed_time_ += period;

  std::lock_guard<std::mutex> lock(control_mutex_);
  
  // Check for command timeout
  double time_since_joint_cmd = (current_time - last_joint_command_time_).toSec();
  double time_since_cartesian_cmd = (current_time - last_cartesian_command_time_).toSec();
  
  // Switch to IDLE if both commands have timed out
  if (time_since_joint_cmd > command_timeout_ && time_since_cartesian_cmd > command_timeout_) {
    if (current_mode_ != ControlMode::IDLE) {
      current_mode_ = ControlMode::IDLE;
      ROS_INFO("UnifiedVelocityController: No recent commands, switched to IDLE mode");
    }
  }
  
  // Execute control based on current mode
  switch (current_mode_) {
    case ControlMode::JOINT_VELOCITY:
      updateJointVelocityControl();
      break;
      
    case ControlMode::CARTESIAN_VELOCITY:
      updateCartesianVelocityControl();
      break;
      
    case ControlMode::IDLE:
    default:
      setAllJointsToZero();
      break;
  }
}

void UnifiedVelocityController::updateJointVelocityControl() {
for (size_t i = 0; i < velocity_joint_handles_.size(); ++i) {
    output_joint_velocities_[i] = (1.0 - alpha) * output_joint_velocities_[i] + alpha * commanded_joint_velocities_[i];
    velocity_joint_handles_[i].setCommand(output_joint_velocities_[i]);
}
}

void UnifiedVelocityController::updateCartesianVelocityControl() {
  // Clamp Cartesian velocities to limits
  geometry_msgs::Twist clamped_twist = commanded_cartesian_twist_;
  
  // Clamp linear velocities
  clamped_twist.linear.x = std::max(-max_linear_velocity_, 
                                   std::min(clamped_twist.linear.x, max_linear_velocity_));
  clamped_twist.linear.y = std::max(-max_linear_velocity_, 
                                   std::min(clamped_twist.linear.y, max_linear_velocity_));
  clamped_twist.linear.z = std::max(-max_linear_velocity_, 
                                   std::min(clamped_twist.linear.z, max_linear_velocity_));
  
  // Clamp angular velocities
  clamped_twist.angular.x = std::max(-max_angular_velocity_, 
                                    std::min(clamped_twist.angular.x, max_angular_velocity_));
  clamped_twist.angular.y = std::max(-max_angular_velocity_, 
                                    std::min(clamped_twist.angular.y, max_angular_velocity_));
  clamped_twist.angular.z = std::max(-max_angular_velocity_, 
                                    std::min(clamped_twist.angular.z, max_angular_velocity_));

  // Get Jacobian at end effector
  auto jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // Form desired spatial velocity vector
  Eigen::Matrix<double, 6, 1> desired_velocity;
  desired_velocity << clamped_twist.linear.x, clamped_twist.linear.y, clamped_twist.linear.z,
                      clamped_twist.angular.x, clamped_twist.angular.y, clamped_twist.angular.z;

  // Damped Least Squares (DLS) inverse kinematics
  // qdot = J^T (J * J^T + lambda^2 * I)^-1 * v
  Eigen::Matrix<double, 6, 6> JJt = jacobian * jacobian.transpose();
  Eigen::Matrix<double, 6, 6> damped_matrix = JJt + 
      damping_factor_ * damping_factor_ * Eigen::Matrix<double, 6, 6>::Identity();
  
  Eigen::Matrix<double, 6, 1> lambda_v = damped_matrix.ldlt().solve(desired_velocity);
  Eigen::Matrix<double, 7, 1> joint_velocities = jacobian.transpose() * lambda_v;

  // Pass to JointVelocityControl
  for (size_t i = 0; i < velocity_joint_handles_.size(); ++i) {
      commanded_joint_velocities_[i] = joint_velocities[i];
  }
  updateJointVelocityControl();

}

void UnifiedVelocityController::setAllJointsToZero() {
  for (size_t i = 0; i < velocity_joint_handles_.size(); ++i) {
      commanded_joint_velocities_[i] = 0.0;
  }
  updateJointVelocityControl();
}

void UnifiedVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
  ROS_INFO("UnifiedVelocityController: Controller stopped");
}

}  // namespace franka_starter_controllers

PLUGINLIB_EXPORT_CLASS(franka_starter_controllers::UnifiedVelocityController,
                       controller_interface::ControllerBase)