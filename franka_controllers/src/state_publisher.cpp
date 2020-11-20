// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <state_publisher.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <franka/errors.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_msgs/Errors.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include "pseudo_inversion.h"

namespace {

tf::Transform convertArrayToTf(const std::array<double, 16>& transform) {
  tf::Matrix3x3 rotation(transform[0], transform[4], transform[8], transform[1], transform[5],
                         transform[9], transform[2], transform[6], transform[10]);
  tf::Vector3 translation(transform[12], transform[13], transform[14]);
  return tf::Transform(rotation, translation);
}

franka_msgs::Errors errorsToMessage(const franka::Errors& error) {
  franka_msgs::Errors message;
  message.joint_position_limits_violation =
      static_cast<decltype(message.joint_position_limits_violation)>(
          error.joint_position_limits_violation);
  message.cartesian_position_limits_violation =
      static_cast<decltype(message.cartesian_position_limits_violation)>(
          error.cartesian_position_limits_violation);
  message.self_collision_avoidance_violation =
      static_cast<decltype(message.self_collision_avoidance_violation)>(
          error.self_collision_avoidance_violation);
  message.joint_velocity_violation =
      static_cast<decltype(message.joint_velocity_violation)>(error.joint_velocity_violation);
  message.cartesian_velocity_violation =
      static_cast<decltype(message.cartesian_velocity_violation)>(
          error.cartesian_velocity_violation);
  message.force_control_safety_violation =
      static_cast<decltype(message.force_control_safety_violation)>(
          error.force_control_safety_violation);
  message.joint_reflex = static_cast<decltype(message.joint_reflex)>(error.joint_reflex);
  message.cartesian_reflex =
      static_cast<decltype(message.cartesian_reflex)>(error.cartesian_reflex);
  message.max_goal_pose_deviation_violation =
      static_cast<decltype(message.max_goal_pose_deviation_violation)>(
          error.max_goal_pose_deviation_violation);
  message.max_path_pose_deviation_violation =
      static_cast<decltype(message.max_path_pose_deviation_violation)>(
          error.max_path_pose_deviation_violation);
  message.cartesian_velocity_profile_safety_violation =
      static_cast<decltype(message.cartesian_velocity_profile_safety_violation)>(
          error.cartesian_velocity_profile_safety_violation);
  message.joint_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.joint_position_motion_generator_start_pose_invalid)>(
          error.joint_position_motion_generator_start_pose_invalid);
  message.joint_motion_generator_position_limits_violation =
      static_cast<decltype(message.joint_motion_generator_position_limits_violation)>(
          error.joint_motion_generator_position_limits_violation);
  message.joint_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.joint_motion_generator_velocity_limits_violation)>(
          error.joint_motion_generator_velocity_limits_violation);
  message.joint_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.joint_motion_generator_velocity_discontinuity)>(
          error.joint_motion_generator_velocity_discontinuity);
  message.joint_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.joint_motion_generator_acceleration_discontinuity)>(
          error.joint_motion_generator_acceleration_discontinuity);
  message.cartesian_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.cartesian_position_motion_generator_start_pose_invalid)>(
          error.cartesian_position_motion_generator_start_pose_invalid);
  message.cartesian_motion_generator_elbow_limit_violation =
      static_cast<decltype(message.cartesian_motion_generator_elbow_limit_violation)>(
          error.cartesian_motion_generator_elbow_limit_violation);
  message.cartesian_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_velocity_limits_violation)>(
          error.cartesian_motion_generator_velocity_limits_violation);
  message.cartesian_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_velocity_discontinuity)>(
          error.cartesian_motion_generator_velocity_discontinuity);
  message.cartesian_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_acceleration_discontinuity)>(
          error.cartesian_motion_generator_acceleration_discontinuity);
  message.cartesian_motion_generator_elbow_sign_inconsistent =
      static_cast<decltype(message.cartesian_motion_generator_elbow_sign_inconsistent)>(
          error.cartesian_motion_generator_elbow_sign_inconsistent);
  message.cartesian_motion_generator_start_elbow_invalid =
      static_cast<decltype(message.cartesian_motion_generator_start_elbow_invalid)>(
          error.cartesian_motion_generator_start_elbow_invalid);
  message.cartesian_motion_generator_joint_position_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_position_limits_violation)>(
          error.cartesian_motion_generator_joint_position_limits_violation);
  message.cartesian_motion_generator_joint_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_limits_violation)>(
          error.cartesian_motion_generator_joint_velocity_limits_violation);
  message.cartesian_motion_generator_joint_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_discontinuity)>(
          error.cartesian_motion_generator_joint_velocity_discontinuity);
  message.cartesian_motion_generator_joint_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_acceleration_discontinuity)>(
          error.cartesian_motion_generator_joint_acceleration_discontinuity);
  message.cartesian_position_motion_generator_invalid_frame =
      static_cast<decltype(message.cartesian_position_motion_generator_invalid_frame)>(
          error.cartesian_position_motion_generator_invalid_frame);
  message.force_controller_desired_force_tolerance_violation =
      static_cast<decltype(message.force_controller_desired_force_tolerance_violation)>(
          error.force_controller_desired_force_tolerance_violation);
  message.controller_torque_discontinuity =
      static_cast<decltype(message.controller_torque_discontinuity)>(
          error.controller_torque_discontinuity);
  message.start_elbow_sign_inconsistent =
      static_cast<decltype(message.start_elbow_sign_inconsistent)>(
          error.start_elbow_sign_inconsistent);
  message.communication_constraints_violation =
      static_cast<decltype(message.communication_constraints_violation)>(
          error.communication_constraints_violation);
  message.power_limit_violation =
      static_cast<decltype(message.power_limit_violation)>(error.power_limit_violation);
  message.joint_p2p_insufficient_torque_for_planning =
      static_cast<decltype(message.joint_p2p_insufficient_torque_for_planning)>(
          error.joint_p2p_insufficient_torque_for_planning);
  message.tau_j_range_violation =
      static_cast<decltype(message.tau_j_range_violation)>(error.tau_j_range_violation);
  message.instability_detected =
      static_cast<decltype(message.instability_detected)>(error.instability_detected);
  return message;
}

}  // anonymous namespace


namespace franka_robot_controllers {

bool StatePublisher::init(hardware_interface::RobotHW* robot_hardware,
                                 ros::NodeHandle& root_node_handle,
                                 ros::NodeHandle& controller_node_handle) {
  franka_state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface == nullptr) {
    ROS_ERROR("StatePublisher: Could not get Franka state interface from hardware");
    return false;
  }
  if (!controller_node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("StatePublisher: Could not get parameter arm_id");
    return false;
  }

  publish_rate = 60;
  trigger_publish = franka_hw::TriggerRate(publish_rate);

  if (!controller_node_handle.getParam("joint_names", joint_names_) ||
      joint_names_.size() != robot_state_.q.size()) {
    ROS_ERROR(
        "StatePublisher: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }
  try {
    franka_state_handle = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("StatePublisher: Exception getting franka state handle: " << ex.what());
    return false;
  }

  {   // Add For get Estimation of the forces

    auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }
  // FIne aggiunta
  }


  
  publisher_transforms_.init(root_node_handle, "/tf", 1);
  publisher_franka_states_.init(controller_node_handle, "franka_states", 1);
  publisher_joint_states_.init(controller_node_handle, "joint_states", 1);
  publisher_joint_states_desired_.init(controller_node_handle, "joint_states_desired", 1);
  publisher_external_wrench_.init(controller_node_handle, "F_ext", 1);
  publisher_external_force_.init(controller_node_handle, "External_Wrench",1);
  publisher_external_force_filter_.init(controller_node_handle, "External_Wrench_Filter",1);

  { // Publisher brackets
  {
    std::lock_guard<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> lock(
        publisher_joint_states_);
    publisher_joint_states_.msg_.name.resize(joint_names_.size());
    publisher_joint_states_.msg_.position.resize(robot_state_.q.size());
    publisher_joint_states_.msg_.velocity.resize(robot_state_.dq.size());
    publisher_joint_states_.msg_.effort.resize(robot_state_.tau_J.size());
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> lock(
        publisher_joint_states_desired_);
    publisher_joint_states_desired_.msg_.name.resize(joint_names_.size());
    publisher_joint_states_desired_.msg_.position.resize(robot_state_.q_d.size());
    publisher_joint_states_desired_.msg_.velocity.resize(robot_state_.dq_d.size());
    publisher_joint_states_desired_.msg_.effort.resize(robot_state_.tau_J_d.size());
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<tf2_msgs::TFMessage>> lock(
        publisher_transforms_);
    publisher_transforms_.msg_.transforms.resize(2);
    tf::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
    tf::Vector3 translation(0.0, 0.0, 0.05);
    tf::Transform transform(quaternion, translation);
    tf::StampedTransform stamped_transform(transform, ros::Time::now(), arm_id + "_link8",
                                           arm_id + "_EE");
    geometry_msgs::TransformStamped transform_message;
    transformStampedTFToMsg(stamped_transform, transform_message);
    publisher_transforms_.msg_.transforms[0] = transform_message;
    translation = tf::Vector3(0.0, 0.0, 0.0);
    transform = tf::Transform(quaternion, translation);
    stamped_transform =
        tf::StampedTransform(transform, ros::Time::now(), arm_id + "_EE", arm_id + "_K");
    transformStampedTFToMsg(stamped_transform, transform_message);
    publisher_transforms_.msg_.transforms[1] = transform_message;
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> lock(
        publisher_external_wrench_);
    publisher_external_wrench_.msg_.header.frame_id = arm_id + "_K";
    publisher_external_wrench_.msg_.wrench.force.x = 0.0;
    publisher_external_wrench_.msg_.wrench.force.y = 0.0;
    publisher_external_wrench_.msg_.wrench.force.z = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.x = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.y = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.z = 0.0;
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> > lock(
        publisher_external_force_);
    publisher_external_force_.msg_.header.frame_id = arm_id + "_K";
    publisher_external_force_.msg_.wrench.force.x = 0.0;
    publisher_external_force_.msg_.wrench.force.y = 0.0;
    publisher_external_force_.msg_.wrench.force.z = 0.0;
    publisher_external_force_.msg_.wrench.torque.x = 0.0;
    publisher_external_force_.msg_.wrench.torque.y = 0.0;
    publisher_external_force_.msg_.wrench.torque.z = 0.0;
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> > lock(
        publisher_external_force_);
    publisher_external_force_.msg_.header.frame_id = arm_id + "_External_Wrench";
    publisher_external_force_.msg_.wrench.force.x = 0.0;
    publisher_external_force_.msg_.wrench.force.y = 0.0;
    publisher_external_force_.msg_.wrench.force.z = 0.0;
    publisher_external_force_.msg_.wrench.torque.x = 0.0;
    publisher_external_force_.msg_.wrench.torque.y = 0.0;
    publisher_external_force_.msg_.wrench.torque.z = 0.0;
  }  
  {
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> > lock(
        publisher_external_force_filter_);
    publisher_external_force_filter_.msg_.header.frame_id = arm_id + "Wrench_Filtered100";
    publisher_external_force_filter_.msg_.wrench.force.x = 0.0;
    publisher_external_force_filter_.msg_.wrench.force.y = 0.0;
    publisher_external_force_filter_.msg_.wrench.force.z = 0.0;
    publisher_external_force_filter_.msg_.wrench.torque.x = 0.0;
    publisher_external_force_filter_.msg_.wrench.torque.y = 0.0;
    publisher_external_force_filter_.msg_.wrench.torque.z = 0.0;
  }

  }

  return true;
}


void StatePublisher::starting(const ros::Time& time ) {
  ROS_INFO("Controller Start!");

  // Bias correction for the current external torque
  robot_state_ = franka_state_handle->getRobotState();
  std::array<double, 7> gravity_array = model_handle->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state_.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  tau_ext_initial_ = tau_measured - gravity;

  // Filter init;
  filter_size = 25;
  filter_index = 0;
  ForceDataVector.resize(filter_size);
  for(int i=0; i<filter_size; i++)
    ForceDataVector[i] << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  MeanMatrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  ROS_INFO("STARTING COMPLETED");
}


void StatePublisher::update(const ros::Time& time, const ros::Duration& /* period */) {
  
  if (trigger_publish()) 
  {
    robot_state_ = franka_state_handle->getRobotState();

    std::array<double, 42> jacobian_array = model_handle->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 7> gravity_array = model_handle->getGravity();

    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state_.tau_J.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

    Eigen::MatrixXd jacobian_transpose_pinv;

    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    tau_ext = tau_measured - gravity - tau_ext_initial_;
    Forces = jacobian_transpose_pinv * tau_ext;
  

    ForceDataVector[filter_index] = Forces;
    filter_index ++;
    if (filter_index >= filter_size)
      filter_index = 0;
    for(int i=0; i< filter_size; i++)
      MeanMatrix += ForceDataVector[i];
    Forces_filter = MeanMatrix/(double)filter_size;  
    MeanMatrix << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    
    
    // Publish to all the topic
    publishFrankaStates(time);
    publishTransforms(time);
    publishExternalWrench(time);
    publishJointStates(time);
    publishExternalForce(time);
    publishExternalForceFilter(time);
    
    sequence_number++;
  }
}



  void StatePublisher::publishFrankaStates(const ros::Time& time)
  {
    if (publisher_franka_states_.trylock()) 
    {
      static_assert(
          sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.cartesian_contact),
          "Robot state Cartesian members do not have same size");
      static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.K_F_ext_hat_K),
                    "Robot state Cartesian members do not have same size");
      static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.O_F_ext_hat_K),
                    "Robot state Cartesian members do not have same size");
      static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.O_dP_EE_d),
                    "Robot state Cartesian members do not have same size");
      static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.O_dP_EE_c),
                    "Robot state Cartesian members do not have same size");
      static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.O_ddP_EE_c),
                    "Robot state Cartesian members do not have same size");
      for (size_t i = 0; i < robot_state_.cartesian_collision.size(); i++) {
        publisher_franka_states_.msg_.cartesian_collision[i] = robot_state_.cartesian_collision[i];
        publisher_franka_states_.msg_.cartesian_contact[i] = robot_state_.cartesian_contact[i];
        publisher_franka_states_.msg_.K_F_ext_hat_K[i] = robot_state_.K_F_ext_hat_K[i];
        publisher_franka_states_.msg_.O_F_ext_hat_K[i] = robot_state_.O_F_ext_hat_K[i];
        publisher_franka_states_.msg_.O_dP_EE_d[i] = robot_state_.O_dP_EE_d[i];
        publisher_franka_states_.msg_.O_dP_EE_c[i] = robot_state_.O_dP_EE_c[i];
        publisher_franka_states_.msg_.O_ddP_EE_c[i] = robot_state_.O_ddP_EE_c[i];
      }

      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.q_d),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq_d),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.ddq_d),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_J),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dtau_J),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_J_d),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.theta),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dtheta),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.joint_collision),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.joint_contact),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_ext_hat_filtered),
                    "Robot state joint members do not have same size");
      for (size_t i = 0; i < robot_state_.q.size(); i++) {
        publisher_franka_states_.msg_.q[i] = robot_state_.q[i];
        publisher_franka_states_.msg_.q_d[i] = robot_state_.q_d[i];
        publisher_franka_states_.msg_.dq[i] = robot_state_.dq[i];
        publisher_franka_states_.msg_.dq_d[i] = robot_state_.dq_d[i];
        publisher_franka_states_.msg_.ddq_d[i] = robot_state_.ddq_d[i];
        publisher_franka_states_.msg_.tau_J[i] = robot_state_.tau_J[i];
        publisher_franka_states_.msg_.dtau_J[i] = robot_state_.dtau_J[i];
        publisher_franka_states_.msg_.tau_J_d[i] = robot_state_.tau_J_d[i];
        publisher_franka_states_.msg_.theta[i] = robot_state_.theta[i];
        publisher_franka_states_.msg_.dtheta[i] = robot_state_.dtheta[i];
        publisher_franka_states_.msg_.joint_collision[i] = robot_state_.joint_collision[i];
        publisher_franka_states_.msg_.joint_contact[i] = robot_state_.joint_contact[i];
        publisher_franka_states_.msg_.tau_ext_hat_filtered[i] = robot_state_.tau_ext_hat_filtered[i];
      }

      static_assert(sizeof(robot_state_.elbow) == sizeof(robot_state_.elbow_d),
                    "Robot state elbow configuration members do not have same size");
      static_assert(sizeof(robot_state_.elbow) == sizeof(robot_state_.elbow_c),
                    "Robot state elbow configuration members do not have same size");
      static_assert(sizeof(robot_state_.elbow) == sizeof(robot_state_.delbow_c),
                    "Robot state elbow configuration members do not have same size");
      static_assert(sizeof(robot_state_.elbow) == sizeof(robot_state_.ddelbow_c),
                    "Robot state elbow configuration members do not have same size");

      for (size_t i = 0; i < robot_state_.elbow.size(); i++) {
        publisher_franka_states_.msg_.elbow[i] = robot_state_.elbow[i];
        publisher_franka_states_.msg_.elbow_d[i] = robot_state_.elbow_d[i];
        publisher_franka_states_.msg_.elbow_c[i] = robot_state_.elbow_c[i];
        publisher_franka_states_.msg_.delbow_c[i] = robot_state_.delbow_c[i];
        publisher_franka_states_.msg_.ddelbow_c[i] = robot_state_.ddelbow_c[i];
      }

      static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.F_T_EE),
                    "Robot state transforms do not have same size");
      static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.EE_T_K),
                    "Robot state transforms do not have same size");
      static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.O_T_EE_d),
                    "Robot state transforms do not have same size");
      static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.O_T_EE_c),
                    "Robot state transforms do not have same size");
      for (size_t i = 0; i < robot_state_.O_T_EE.size(); i++) {
        publisher_franka_states_.msg_.O_T_EE[i] = robot_state_.O_T_EE[i];
        publisher_franka_states_.msg_.F_T_EE[i] = robot_state_.F_T_EE[i];
        publisher_franka_states_.msg_.EE_T_K[i] = robot_state_.EE_T_K[i];
        publisher_franka_states_.msg_.O_T_EE_d[i] = robot_state_.O_T_EE_d[i];
        publisher_franka_states_.msg_.O_T_EE_c[i] = robot_state_.O_T_EE_c[i];
      }
      publisher_franka_states_.msg_.m_ee = robot_state_.m_ee;
      publisher_franka_states_.msg_.m_load = robot_state_.m_load;
      publisher_franka_states_.msg_.m_total = robot_state_.m_total;

      for (size_t i = 0; i < robot_state_.I_load.size(); i++) {
        publisher_franka_states_.msg_.I_ee[i] = robot_state_.I_ee[i];
        publisher_franka_states_.msg_.I_load[i] = robot_state_.I_load[i];
        publisher_franka_states_.msg_.I_total[i] = robot_state_.I_total[i];
      }

      for (size_t i = 0; i < robot_state_.F_x_Cload.size(); i++) {
        publisher_franka_states_.msg_.F_x_Cee[i] = robot_state_.F_x_Cee[i];
        publisher_franka_states_.msg_.F_x_Cload[i] = robot_state_.F_x_Cload[i];
        publisher_franka_states_.msg_.F_x_Ctotal[i] = robot_state_.F_x_Ctotal[i];
      }

      publisher_franka_states_.msg_.time = robot_state_.time.toSec();
      publisher_franka_states_.msg_.control_command_success_rate =
          robot_state_.control_command_success_rate;
      publisher_franka_states_.msg_.current_errors = errorsToMessage(robot_state_.current_errors);
      publisher_franka_states_.msg_.last_motion_errors =
          errorsToMessage(robot_state_.last_motion_errors);

      switch (robot_state_.robot_mode) {
        case franka::RobotMode::kOther:
          publisher_franka_states_.msg_.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_OTHER;
          break;

        case franka::RobotMode::kIdle:
          publisher_franka_states_.msg_.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_IDLE;
          break;

        case franka::RobotMode::kMove:
          publisher_franka_states_.msg_.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_MOVE;
          break;

        case franka::RobotMode::kGuiding:
          publisher_franka_states_.msg_.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_GUIDING;
          break;

        case franka::RobotMode::kReflex:
          publisher_franka_states_.msg_.robot_mode = franka_msgs::FrankaState::ROBOT_MODE_REFLEX;
          break;

        case franka::RobotMode::kUserStopped:
          publisher_franka_states_.msg_.robot_mode =
              franka_msgs::FrankaState::ROBOT_MODE_USER_STOPPED;
          break;

        case franka::RobotMode::kAutomaticErrorRecovery:
          publisher_franka_states_.msg_.robot_mode =
              franka_msgs::FrankaState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
          break;
      }

      publisher_franka_states_.msg_.header.seq = sequence_number;
      publisher_franka_states_.msg_.header.stamp = time;
      publisher_franka_states_.unlockAndPublish();
    }
  }
  void StatePublisher::publishJointStates(const ros::Time& time) {
    if (publisher_joint_states_.trylock()) {
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_J),
                    "Robot state joint members do not have same size");
      for (size_t i = 0; i < robot_state_.q.size(); i++) {
        publisher_joint_states_.msg_.name[i] = joint_names_[i];
        publisher_joint_states_.msg_.position[i] = robot_state_.q[i];
        publisher_joint_states_.msg_.velocity[i] = robot_state_.dq[i];
        publisher_joint_states_.msg_.effort[i] = robot_state_.tau_J[i];
      }
      publisher_joint_states_.msg_.header.stamp = time;
      publisher_joint_states_.msg_.header.seq = sequence_number;
      publisher_joint_states_.unlockAndPublish();
    }
    if (publisher_joint_states_desired_.trylock()) {
      static_assert(sizeof(robot_state_.q_d) == sizeof(robot_state_.dq_d),
                    "Robot state joint members do not have same size");
      static_assert(sizeof(robot_state_.q_d) == sizeof(robot_state_.tau_J_d),
                    "Robot state joint members do not have same size");
      for (size_t i = 0; i < robot_state_.q_d.size(); i++) {
        publisher_joint_states_desired_.msg_.name[i] = joint_names_[i];
        publisher_joint_states_desired_.msg_.position[i] = robot_state_.q_d[i];
        publisher_joint_states_desired_.msg_.velocity[i] = robot_state_.dq_d[i];
        publisher_joint_states_desired_.msg_.effort[i] = robot_state_.tau_J_d[i];
      }
      publisher_joint_states_desired_.msg_.header.stamp = time;
      publisher_joint_states_desired_.msg_.header.seq = sequence_number;
      publisher_joint_states_desired_.unlockAndPublish();
    }
  }
  void StatePublisher::publishTransforms(const ros::Time& time) {
    if (publisher_transforms_.trylock()) {
      tf::StampedTransform stamped_transform(convertArrayToTf(robot_state_.F_T_EE), time,
                                            arm_id + "_link8", arm_id + "_EE");
      geometry_msgs::TransformStamped transform_message;
      transformStampedTFToMsg(stamped_transform, transform_message);
      publisher_transforms_.msg_.transforms[0] = transform_message;
      stamped_transform = tf::StampedTransform(convertArrayToTf(robot_state_.EE_T_K), time,
                                              arm_id + "_EE", arm_id + "_K");
      transformStampedTFToMsg(stamped_transform, transform_message);
      publisher_transforms_.msg_.transforms[1] = transform_message;
      publisher_transforms_.unlockAndPublish();
    }
  }
  void StatePublisher::publishExternalWrench(const ros::Time& time) {
    if (publisher_external_wrench_.trylock()) {
      publisher_external_wrench_.msg_.header.frame_id = arm_id + "_K";
      publisher_external_wrench_.msg_.header.stamp = time;
      publisher_external_wrench_.msg_.wrench.force.x = robot_state_.K_F_ext_hat_K[0];
      publisher_external_wrench_.msg_.wrench.force.y = robot_state_.K_F_ext_hat_K[1];
      publisher_external_wrench_.msg_.wrench.force.z = robot_state_.K_F_ext_hat_K[2];
      publisher_external_wrench_.msg_.wrench.torque.x = robot_state_.K_F_ext_hat_K[3];
      publisher_external_wrench_.msg_.wrench.torque.y = robot_state_.K_F_ext_hat_K[4];
      publisher_external_wrench_.msg_.wrench.torque.z = robot_state_.K_F_ext_hat_K[5];
      publisher_external_wrench_.unlockAndPublish();
    }
  }
  void StatePublisher::publishExternalForce(const ros::Time& time) {
    if (publisher_external_force_.trylock()) {
      publisher_external_force_.msg_.header.frame_id = arm_id + "_external_force";
      publisher_external_force_.msg_.header.stamp = time;
      publisher_external_force_.msg_.wrench.force.x = (float) Forces(0);
      publisher_external_force_.msg_.wrench.force.y = (float)Forces(1);
      publisher_external_force_.msg_.wrench.force.z = (float)Forces(2);
      publisher_external_force_.msg_.wrench.torque.x = (float)Forces(3);
      publisher_external_force_.msg_.wrench.torque.y = (float)Forces(4);
      publisher_external_force_.msg_.wrench.torque.z = (float)Forces(5);
      publisher_external_force_.unlockAndPublish();
    }
  }
  void StatePublisher::publishExternalForceFilter(const ros::Time& time) {
  
  if (publisher_external_force_filter_.trylock()) {
    publisher_external_force_filter_.msg_.header.frame_id = arm_id + "filtered";
    publisher_external_force_filter_.msg_.header.stamp = time;
    publisher_external_force_filter_.msg_.wrench.force.x = (float)Forces_filter(0);
    publisher_external_force_filter_.msg_.wrench.force.y = (float)Forces_filter(1);
    publisher_external_force_filter_.msg_.wrench.force.z = (float)Forces_filter(2);
    publisher_external_force_filter_.msg_.wrench.torque.x = (float)Forces_filter(3);
    publisher_external_force_filter_.msg_.wrench.torque.y = (float)Forces_filter(4);
    publisher_external_force_filter_.msg_.wrench.torque.z = (float)Forces_filter(5);
    publisher_external_force_filter_.unlockAndPublish();
  }
  }



}  // namespace my_franka_control

PLUGINLIB_EXPORT_CLASS(franka_robot_controllers::StatePublisher, controller_interface::ControllerBase)
