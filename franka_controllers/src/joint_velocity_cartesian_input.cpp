// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_controllers/joint_velocity_controller.h>


#include <cmath>
#include <memory>
#include <string>

#include <franka/errors.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_msgs/Errors.h>

#include <hardware_interface/hardware_interface.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>


#include <ros/ros.h>


namespace franka_controllers{

bool JointVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                 ros::NodeHandle& robot_node_handle) {

    velocity_joint_interface = robot_hardware->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface == nullptr) {
        ROS_ERROR("JointVelocityController: Error getting velocity joint interface from hardware!");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!robot_node_handle.getParam("joint_names", joint_names)) {
        ROS_ERROR("JointVelocityController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
        ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "
                        << joint_names.size() << " instead of 7 names!");
        return false;
    }
    velocity_joint_handles.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        try {
        velocity_joint_handles[i] = velocity_joint_interface->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("JointVelocityController: Exception getting joint handles: " << ex.what());
        return false;
        }
    }

    // Needed to obtain the Jacobian!
    auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) { ROS_ERROR_STREAM("JointVelocityController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM("JointVelocityController: Exception getting model handle from interface: " << ex.what());
        return false;
    }


    /*
    publish_rate = 60;
    trigger_publish = franka_hw::TriggerRate(publish_rate);
    */
    
    vel_cmd_sub = robot_node_handle.subscribe("/joint_velocity_request", 10, &JointVelocityController::Velocity_callback, this);


    return true;
}


void JointVelocityController::starting(const ros::Time& time )
{
    for(int i=0; i<7; i++)
        joint_velocity(i) = 0;

    elapsed_time = ros::Duration(0.0);
    vel_cmd_timeout = 0.1;

    filter_size = 25;
    filter_index = 0;
    VelDataVector.resize(filter_size);
    for(int i=0; i<filter_size; i++)
        VelDataVector[i] << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    MeanMatrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    vel_msg.data.resize(6);

    ROS_INFO("STARTING COMPLETED");
}


void JointVelocityController::update(const ros::Time& time, const ros::Duration& period ) {
  
  elapsed_time += period;

    robot_state_ = franka_state_handle
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::MatrixXd jacobian_inv;
  

  if(ros::Time::now().toSec() - last_cmd_time > vel_cmd_timeout)
  {
    for(int i=0; i<7; i++)
      velocity_joint_handles[i].setCommand(0.00);
    // ROS_INFO("JointVelocityController: Not receiveing any command, set the velocity to 0!");
  }
  else
  {
    VelDataVector[filter_index] = cartesian_velocity;
    filter_index ++;
    if (filter_index >= filter_size)
        filter_index = 0;
    for(int i=0; i< filter_size; i++)
        MeanMatrix += VelDataVector[i];
    vel_filter = MeanMatrix/(double)filter_size;  
    MeanMatrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        
    pseudoInverse(jacobian, jacobian_inv, false);
    joint_velocity = jacobian_inv*vel_filter;

    for(int i=0;i<7;i++)
      velocity_joint_handles[i].setCommand(joint_velocity(i));  
  }
  
}

void JointVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void JointVelocityController::Velocity_callback(const std_msgs::Float32MultiArray& msg)
{
  if(msg.data.size() != 7)
  {
    ROS_ERROR("JointVelocityController: Joint velocity request not consistence! :( ");
  }
  else
  {
    for(int i=0; i<7;i++)
      joint_velocity(i) = (double)msg.data[i]; 
  }
  last_cmd_time = ros::Time::now().toSec();
}

}  // namespace my_franka_velocity_controller


PLUGINLIB_EXPORT_CLASS(franka_controllers::JointVelocityController, 
                      controller_interface::ControllerBase)
