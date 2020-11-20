# Franka controler

The controllers are based on the FrankaExampleControllers provided by Franka Emika.


Always launch the state_publisher that provides all the structure to publish the states of the robot and the external wrench estimation as well.

To control the robot is sufficient to publish the command in the appropriate topic:


joint_velocity_controller       -> /joint_velocity_request
cartesian_velocity_controller   -> /cartesian_velocity_request 

The message to publish is ALWAYS a std_msgs::Float32MultiArray


## How can i create a new controller? ##

1) Create .h and .cpp files for the controller
2) Modify the plugin files ("franka_robot_controllers_plugin.xml") and include the new control.
3) Modify the .yaml file in the config folder.
4) Include your new .src file in the CMakeList.txt, under the voice "add_library" 
5)Enjoy your new controller and hope that it's work correctly!


