

#include <franka_controllers/joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>        // ?
#include <ros/ros.h>


namespace franka_controllers{

bool JointPositionController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& robot_node_handle)
{
    position_joint_interface = robot_hardware->get<hardware_interface::PositionJointInterface>();
    if (position_joint_interface == nullptr)
    {
        ROS_ERROR("JointPositionController: Error getting position joint interface from hardware!");
        return false;
    }

    std::vector<std::string> joint_names;
    if (!robot_node_handle.getParam("joint_names", joint_names)) 
        ROS_ERROR("JointPositionController: Could not parse joint names");

    if (joint_names.size() != 7)
    {
        ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                        << joint_names.size() << " instead of 7 names!");
        return false;
    }
    position_joint_handles.resize(7);
    for (size_t i = 0; i < 7; ++i)
    {
        try {
        position_joint_handles[i] = position_joint_interface->getHandle(joint_names[i]);
        } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM("JointPositionController: Exception getting joint handles: " << e.what());
        return false;
        }
    }

    publish_rate = 60.0;
    trigger_publish = franka_hw::TriggerRate(publish_rate);
  
    pos_cmd_sub = robot_node_handle.subscribe("/joint_position_request", 10, &JointPositionController::Position_callback, this);

    return true;
}

void JointPositionController::starting(const ros::Time& /* time */) {
    for (size_t i = 0; i < 7; ++i)
        joint_position(i) = position_joint_handles[i].getPosition();
    
    elapsed_time = ros::Duration(0.0);
}

void JointPositionController::update(const ros::Time& /*time*/, const ros::Duration& period) {
    elapsed_time += period;

    for(int i=0; i<7; i++)
        position_joint_handles[i].setCommand(joint_position(i));

}

void JointPositionController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    ROS_INFO("Controller is stopped, see you next time");
}


void JointPositionController::Position_callback(const std_msgs::Float32MultiArray& msg)
{
    if(msg.data.size() != 7)
        ROS_ERROR("HEY! you gave me a number of joints different then 7!");
    else
    {
        for(int i=0; i<7; i++)
            joint_position(i) = msg.data[i];
    }
}


}   // franka_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::JointPositionController,
                        controller_interface::ControllerBase)