// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <franka/errors.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_msgs/Errors.h>
#include <franka_hw/trigger_rate.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>


namespace franka_robot_controllers{

bool CartesianVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                 ros::NodeHandle& robot_node_handle) {


  std::string arm_id;
  if (!robot_node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  publish_rate = 100;
  trigger_publish= franka_hw::TriggerRate(publish_rate);

  vel_cmd_sub = robot_node_handle.subscribe("/cartesian_velocity_request", 10, &CartesianVelocityController::Velocity_callback, this);


  return true;
}


void CartesianVelocityController::starting(const ros::Time& time ) {
  ROS_INFO("STARTING COMPLETED");
  for(int i=0; i<6; i++)
    cartesian_velocity(i) = 0;

    elapsed_time = ros::Duration(0.0);
}


void CartesianVelocityController::update(const ros::Time& time, const ros::Duration&  period ) {
  
  elapsed_time += period;


  if (trigger_publish())
  {
    std::array<double,6> command;

    for(int i=0;i<6;i++)
      command[i] = cartesian_velocity(i);
  
    velocity_cartesian_handle->setCommand(command);
  }

}

void CartesianVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void CartesianVelocityController::Velocity_callback(const std_msgs::Float32MultiArray& msg)
{
    for(int i=0; i<6;i++)
      cartesian_velocity(i) = (double)msg.data[i]; 
}

}  // namespace my_franka_velocity_controller


PLUGINLIB_EXPORT_CLASS(franka_robot_controllers::CartesianVelocityController, 
                      controller_interface::ControllerBase)
