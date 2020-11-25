# franka_driver
Franka Controllers ready to use:

This folder provide evrything that is necessary to start using a panda robot. <br />
All the controllers can be founded in the franka_controlelrs package. After you launched or activated from the control_manager the desired controller, you just need to publish the new desired target in the proper topics.

Description of the packages contained:

franka_control: Package containing the file for state publisher
franka_controllers: Package with all the available controller to launch <br />
franka_description: Meshes and robot model (xacro, urdf ..) <br />
franka_gripper: Gripper controls <br />
franka_hw: Franka hardware library <br />
franka_msg: package with all the messages<br />
panda_moveit_config: MoveIt information for the panda<br />



### How to use it? ###

To use the packages be sure to have correctly installed all the libraries provided by Franka Emika following the procedure: <br>
https://frankaemika.github.io/docs/installation_linux.html

Download all the folders and compile the workspace with the proper command, substituite the correct path of the libfranka folder:
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


Controller Interface topic:

joint_velocity_controller: /joint_velocity_request
cartesian_velocity_controller: /cartesian_velocity_controller




# Under development! #

