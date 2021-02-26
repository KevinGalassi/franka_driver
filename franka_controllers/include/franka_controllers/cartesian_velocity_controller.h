#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <franka/rate_limiting.h>


#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <std_msgs/Float32MultiArray.h>
#include <Eigen/Core>


namespace franka_controllers {

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
      std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;

    // Timing
    ros::Duration elapsed_time;
    double publish_rate; // double check
    franka_hw::TriggerRate trigger_publish;
    double last_cmd_time;
    double vel_cmd_timeout;

    /* Velocity rescaler */
    /*
    const double p_dot = 1.700;
    const double p_ddot = 13.0;
    const double p_dddot = 6500.0;
    */

    const double p_dot = 1;
    const double p_ddot = 8.0;
    const double p_dddot = 2000.0;

    const double rot_dot = 2;
    const double rot_ddot = 18.0;
    const double rot_dddot = 5000.0;

    Eigen::Vector3d scaling_factor;
    double max_scaling_factor;

    Eigen::Matrix<double, 6, 1> last_cart_velocity;
    Eigen::Matrix<double, 6, 1> last_cartesian_acc;
    Eigen::Matrix<double, 6, 1> cartesian_velocity;
    Eigen::Matrix<double, 6, 1> new_cartesian_velocity;
    Eigen::Matrix<double, 6, 1> cartesian_acc;
    Eigen::Matrix<double, 6, 1> cartesian_jerk;

    std::array<double, 6> new_cartesian_vel;

    // Input command subscriber
    ros::Subscriber vel_cmd_sub; 
    std_msgs::Float32MultiArray vel_msg;
    void Velocity_callback(const std_msgs::Float32MultiArray& msg);
    std::array<double, 6> command, last_command;


    // Filter
    Eigen::Matrix<double, 6, 1> vel_filter;
    int filter_size;
    int filter_index;
    std::vector<Eigen::Matrix<double, 6, 1>> VelDataVector;
    Eigen::Matrix<double, 6, 1> MeanMatrix;

};


} // franka_controllers