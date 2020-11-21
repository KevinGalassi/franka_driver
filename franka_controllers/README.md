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


### How can i launch my controller? ###

Use controller.launch!, this will enable the franka to be controller through several differents types of interface. Firstly, it load the state publisher and the RViz interface, then load all the controllers selected.
By default, the only controller that is active is the joint's position controller loaded by MoveIt, but calling the service /controll_manager/switch_Controlelr your are able to choose the controller you may prefer.
You can also create your own .launch file with the controller you prefer!

