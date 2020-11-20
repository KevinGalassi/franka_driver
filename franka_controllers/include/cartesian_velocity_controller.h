#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Core>


namespace franka_robot_controllers {

class CartesianVelocityController : public controller_interface::MultiInterfaceController<
                                               franka_hw::FrankaVelocityCartesianInterface,
                                               franka_hw::FrankaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& robot_node_handle) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    void starting(const ros::Time&) override;
    void stopping(const ros::Time&) override;

private:
    franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface;
    std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle;
    ros::Duration elapsed_time;

    int publish_rate; // double check
    franka_hw::TriggerRate trigger_publish;

    ros::Subscriber vel_cmd_sub; 
    void Velocity_callback(const std_msgs::Float32MultiArray& msg);

    Eigen::Matrix<double, 6, 1> cartesian_velocity;
};


} // franka_robot_controlles