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
		vel_msg.data.resize(7);

		q_dot << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;
    	q_ddot << 15, 7.5,	10,	12.5,	15,	20,	20;
    	q_dddot << 7500, 3750, 5000, 6250, 7500, 10000,	10000;


		ROS_INFO("STARTING COMPLETED");
	}


	void JointVelocityController::update(const ros::Time& time, const ros::Duration& period ) {
	
		elapsed_time += period;

		if(ros::Time::now().toSec() - last_cmd_time > vel_cmd_timeout)
		{
			for(int i=0; i<7; i++)
				new_joint_velocity(i) = 0.0;
			// ROS_INFO("JointVelocityController: Not receiveing any command, set the velocity to 0!");
		}
		
		for(int i=0; i<7; i++)
		{
			joint_acceleration(i) = (new_joint_velocity(i) - last_joint_velocity(i))/period.toSec();
			joint_jerk(i) = (joint_acceleration(i) - last_joint_acceleration(i))/period.toSec();
		}
		scaling_factor[0] = (new_joint_velocity.cwiseQuotient(q_dot)).maxCoeff();
		scaling_factor[1] = (joint_acceleration.cwiseQuotient(q_ddot)).maxCoeff();
		scaling_factor[2] = (joint_jerk.cwiseQuotient(q_dddot)).maxCoeff();

		Eigen::Index max,min;
		max_scaling_factor = scaling_factor.maxCoeff(&max);
		if( max_scaling_factor > 1.0 && max_scaling_factor != 0.0)
		{
			int index = (int)max;
			for(int i=0; i<6; i++)
					command[i] = new_joint_velocity(i)/(pow(max_scaling_factor, 1/(index+1)));
		}
		else
		{
			for(int i=0; i<6; i++)
					command[i] = new_joint_velocity(i);
		}

		for(int i=0; i<7; i++)
			velocity_joint_handles[i].setCommand(command[i]);


		for(int i=0; i<6; i++)
		{
			last_joint_velocity(i) = command[i];
			last_joint_acceleration(i) = (command[i] - last_command[i])/period.toSec();
			last_command[i] = command[i];
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
			ROS_ERROR("JointVelocityController: Joint velocity request not consistence! :( ");
		else
		{
			for(int i=0; i<7;i++)
				new_joint_velocity(i) = (double)msg.data[i]; 
		}
		last_cmd_time = ros::Time::now().toSec();
	}

	}  // namespace my_franka_velocity_controller


	PLUGINLIB_EXPORT_CLASS(franka_controllers::JointVelocityController, controller_interface::ControllerBase)
