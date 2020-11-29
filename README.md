# franka_driver
Franka Controllers ready to use:

This folder provides everything that is necessary to start using a panda robot. <br />
All the controllers can be founded in the franka_controllers package. After you launched or activated from the control_manager the desired controller, you just need to publish the new desired target in the proper topics.

Description of the packages contained:

franka_control: Package containing the file for state publisher
franka_controllers: Package with all the available controller to launch <br />
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
Once you have compiled, switch to the root user:
sudo su

Launch the controllers:
```sh
$ roslaunch franka_controllers controllers.launch
```
This will launch the joint and cartesian velocity controllers with the status -stopped, instead the joint position controller will be enabled, this controller can be used to control the franka through moveit.


### Interface ###

To control the robot, the user must publish an std_msgs::Float32MultiArray vector in the correct topic listed below.

| controller name             | Topic to publish          |
|-----------------------------|---------------------------|
|joint_velocity_controller    |/joint_velocity_request    |
|cartesian_velocity_controller|/cartesian_velocity_request|

The controller requires a **constant** publishing of velocity vector, otherwise as safety precaution the velocity will be automatically been setted to 0. <br>
As additional feature, the controller will automatically check if the input respect the joint/cartesian limits in term of velocity, acceleration and jerk, moving the robot at the heighest velocity possible.



# Under development! #

For those that actually use this repo, please let me know if you have found it useful or having some issue with it! <br>
Suggestion are <ins> always </ins> welcome!


