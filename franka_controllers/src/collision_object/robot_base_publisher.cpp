
#include "ros/ros.h"

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"



int main(int argc, char** argv)
{  

    ros::init(argc, argv, "Grasptest");
    ros::NodeHandle nh;


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::CollisionObject robot_base;
    robot_base.id = "Robot_base";
    robot_base.header.frame_id = "panda_link0";

    robot_base.primitives.resize(1);
    robot_base.primitives[0].type = robot_base.primitives[0].BOX;
    robot_base.primitives[0].dimensions.resize(3);
    robot_base.primitives[0].dimensions[0] = 2;
    robot_base.primitives[0].dimensions[1] = 1;
    robot_base.primitives[0].dimensions[2] = 0.1;

    robot_base.primitive_poses.resize(1);
    robot_base.primitive_poses[0].position.x = 0.7;
    robot_base.primitive_poses[0].position.y = 0;
    robot_base.primitive_poses[0].position.z = -0.050;

    robot_base.operation = robot_base.ADD;
    planning_scene_interface.applyCollisionObject(robot_base);

    ros::spin();

    return 0;
}