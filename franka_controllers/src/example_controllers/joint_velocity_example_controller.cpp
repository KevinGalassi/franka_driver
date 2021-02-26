// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>



namespace franka_example_controllers {

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware, 
                                            ros::NodeHandle& root_node_handle, 
                                            ros::NodeHandle& controller_node_handle){

  franka_state_interface_ = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("FrankaStateController: Could not get Franka state interface from hardware");
    return false;
  }
  if (!controller_node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("FrankaStateController: Could not get parameter arm_id");
    return false;
  }
  double publish_rate(100);

  if (!controller_node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("FrankaStateController: Did not find publish_rate. Using default "
                    << publish_rate << " [Hz].");
  }

  publish_rate = 200;
  trigger_publish_ = franka_hw::TriggerRate(publish_rate);

  if (!controller_node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != robot_state_.q.size()) {
    ROS_ERROR(
        "FrankaStateController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }
  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("FrankaStateController: Exception getting franka state handle: " << ex.what());
    return false;
  }

  // Needed to obtain the Jacobian!
  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }


  // Addition for velocity handle (Used to assign velocity command)

   velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> _;
  if (!controller_node_handle.getParam("joint_names", joint_names_)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names_.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names_.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names_[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }


  vel_cmd_sub = controller_node_handle.subscribe("/cartesian_velocity_request", 1, &JointVelocityExampleController::Velocity_callback, this);

  return true;



}

void JointVelocityExampleController::starting(const ros::Time& /* time */) {
    ROS_INFO("STARTING COMPLETED");


}

void JointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  if (trigger_publish_()) {
    robot_state_ = franka_state_handle_->getRobotState();

    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::MatrixXd jacobian_inv;

    if(joint_cmd_start)
    {
      pseudoInverse(jacobian, jacobian_inv);
      joint_velocity = jacobian_inv*cartesian_velocity;
      for(int i=0;i<7;i++)
        velocity_joint_handles_[i].setCommand(joint_velocity(i));
    }
    else
    {
      for(int i=0; i<velocity_joint_handles_.size(); i++)
        velocity_joint_handles_[i].setCommand(0.0);
    }
    
  }
}

void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void JointVelocityExampleController::Velocity_callback(const std_msgs::Float32MultiArray& msg)
{
  int k=0;

  for(int i=0; i<6; i++)
  {
    if(msg.data[i] == 0.0)
      k++;
  }

  if(k == 6)
  {
    joint_cmd_start = false;
    for(int i=0; i<7; i++)
      velocity_joint_handles_[i].setCommand(0.0);
    ROS_INFO("Command received - STOP");
  }
  else
  {
    for(int i=0; i<6;i++)
      cartesian_velocity(i) = (double)msg.data[i];
    joint_cmd_start = true;
    std::cout << cartesian_velocity << "\n";
    ROS_INFO("Command received");
  }
  
  k=0;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
