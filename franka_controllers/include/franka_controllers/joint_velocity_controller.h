// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_hw/franka_state_interface.h>

#include <franka_msgs/FrankaState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>

namespace franka_controllers
{

  /**
 * Controller to publish the robot state as ROS topics.
 */
  class JointVelocityController : public controller_interface::MultiInterfaceController<
                                      franka_hw::FrankaStateInterface,
                                      hardware_interface::VelocityJointInterface>
  {
  public:
    bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &robot_node_handle) override;
    void starting(const ros::Time &time);
    void update(const ros::Time &time, const ros::Duration &period) override;
    void stopping(const ros::Time &) override;

  private:
    // Velocity handler
    hardware_interface::VelocityJointInterface *velocity_joint_interface;
    std::vector<hardware_interface::JointHandle> velocity_joint_handles;
    
    // Timing
    ros::Duration elapsed_time;
    double publish_rate;
    franka_hw::TriggerRate trigger_publish;
    double last_cmd_time;
    double vel_cmd_timeout;

    // Joint space limit check

    Eigen::Matrix<double, 7, 1> q_dot;
    Eigen::Matrix<double, 7, 1> q_ddot;
    Eigen::Matrix<double, 7, 1> q_dddot;
    
    Eigen::Vector3d scaling_factor;
    double max_scaling_factor;

    Eigen::Matrix<double, 7, 1> last_joint_velocity;
    Eigen::Matrix<double, 7, 1> last_joint_acceleration;
    Eigen::Matrix<double, 7, 1> joint_velocity;
    Eigen::Matrix<double, 7, 1> new_joint_velocity;
    Eigen::Matrix<double, 7, 1> joint_acceleration;
    Eigen::Matrix<double, 7, 1> joint_jerk;

    // Input command subscriber
    ros::Subscriber vel_cmd_sub;
    std_msgs::Float32MultiArray vel_msg;
    void Velocity_callback(const std_msgs::Float32MultiArray &msg);
    std::array<double, 7> command, last_command;
    
    // Filter
    Eigen::Matrix<double, 7, 1> vel_filter;
    int filter_size;
    int filter_index;
    std::vector<Eigen::Matrix<double, 7, 1>> VelDataVector;
    Eigen::Matrix<double, 7, 1> MeanMatrix;

  };

} // namespace franka_controllers
