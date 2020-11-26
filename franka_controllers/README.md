# Franka controllers

The controllers are based on the FrankaExampleControllers provided by Franka Emika. <br>




## How can i create a new controller? ##

1) Create .h and .cpp files for the controller <br>
2) Modify the plugin files ("franka_robot_controllers_plugin.xml") and include the new control. <br>
3) Modify the .yaml file in the config folder with the informations required to the control (arm_id, joint_id, ...) <br>
4) Include your new .src file in the CMakeList.txt, under the voice "add_library" <br>
5) Enjoy your new controller and hope that it's work correctly!


