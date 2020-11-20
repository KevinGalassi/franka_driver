// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_msgs/FrankaState.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <geometry_msgs/WrenchStamped.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <tf2_msgs/TFMessage.h>

#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>

namespace franka_robot_controllers
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
    ros::Duration elapsed_time;
    
    int publish_rate;
    franka_hw::TriggerRate trigger_publish;

    ros::Subscriber vel_cmd_sub;
    void Velocity_callback(const std_msgs::Float32MultiArray &msg);
    
    Eigen::Matrix<double, 7, 1> joint_velocity;

  };

} // namespace franka_robot_controllers
