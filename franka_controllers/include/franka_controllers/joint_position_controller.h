// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/trigger_rate.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>

namespace franka_controllers {

class JointPositionController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::PositionJointInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& robot_node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void stopping(const ros::Time &) override;

private:
    hardware_interface::PositionJointInterface* position_joint_interface;
    std::vector<hardware_interface::JointHandle> position_joint_handles;
    ros::Duration elapsed_time;

    double publish_rate;
    franka_hw::TriggerRate trigger_publish;

    ros::Subscriber pos_cmd_sub;
    void Position_callback(const std_msgs::Float32MultiArray& msg);

    Eigen::Matrix<double,7,1> joint_position;
    
};

}  // namespace franka_controllers