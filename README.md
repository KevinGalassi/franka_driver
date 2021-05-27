# franka_driver
Franka Controllers ready to use:

This folder provides everything that is necessary to start using a panda robot. <br />
All the controllers can be founded in the franka_controllers package. After you launched or activated from the control_manager the desired controller, you just need to publish the new desired target in the proper topics.

Description of the packages contained:

franka_controllers: Package with all the available controller to launch <br />
franka_example_controllers: Contain all the example provided by Franka Emika <br />
franka_description: Meshes and robot model (xacro, urdf ..) <br />
franka_gripper: Gripper controls <br />
franka_hw: Franka hardware library <br />
franka_msg: package with all the messages<br />
panda_moveit_config: MoveIt information for the panda<br />



### Installation ###

To use the packages be sure to have correctly installed all the libraries provided by Franka Emika following the procedure: <br>
https://frankaemika.github.io/docs/installation_linux.html

Download all the folders and compile the workspace with the proper command, substitute the correct path of the libfranka folder:
```sh
$ catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
```

### Launch ###

To activate the controllers use the launch file controllers.launch as root user in a ubuntu version with a rt kernel, the launch file loads all the controllers requested in a stopped mode by chainging the arguent inside the launch file. The online control active by default is the pos_trajectory_joint_interface used by MoveIt. With the parameters in the launch file is possible to modify the end effector of the robot to.


'''sh
$ sudo su
'''
Launch the controllers:
```sh
$ roslaunch franka_controllers controllers.launch
```

### Simulation ###
The robot can be also load in a simulation environment using RVIZ, in this simulation are not covered the inertia and contacts, but can be used to test the movement of the robot. The simulation is created using MoveIt
```sh
$ roslaunch franka_controllers simulation.launch
```



### Interface ###

To control the robot, the user must publish an std_msgs::Float32MultiArray vector in the correct topic listed below.

| controller name             | Topic to publish          | Type                          |
|-----------------------------|---------------------------|-------------------------------|
|joint_velocity_controller    |/joint_velocity_request    |std_msgs/Float32MultiArray [7] |
|cartesian_velocity_controller|/cartesian_velocity_request|std_msgs/Float32MultiArray [6] |

The controller requires a **constant** publishing of velocity vector, otherwise as safety precaution the velocity will be automatically been setted to 0. <br>
As additional feature, the controller will automatically check if the input respect the joint/cartesian limits in term of velocity, acceleration and jerk, moving the robot at the heighest velocity possible.



# Under development! #


- [ ] Add [Move-RT controller](https://dei-gitlab.dei.unibo.it/lar/move_rt) as a possible high-level trajectory controller.
- [ ] Limit the velocity in the joint position controller
- [ ] Add filtering of the input in joint_velcoity_controller

