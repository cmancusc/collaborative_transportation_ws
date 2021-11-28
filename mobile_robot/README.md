
# gazebo_simulation

Package that contains the gazebo simulations of the Mancus project, using the robots UR10_e and MIR100.

### Dependencies

 * UR10_e

 	see https://github.com/UniversalRobots/Universal_Robots_ROS_Driver 
  and follow the related instructions

 * MIR100

 	```git clone -b melodic-2.8 https://github.com/dfki-ric/mir_robot.git```

 	```sudo apt install ros-melodic-mir-robot```

 * gazebo9

## Running the tests

Launch simulation in Gazebo:

	```roslaunch mobile_robot mobile_robot.launch```

 * to move the joints you can publish a message using the following command:
	
	rosrun mobile_robot joint_publisher

## Running on the real robot

* Launch UR10e
  roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.2.30 kinematics_config:=my_robot_calibration.yaml

* Launch MiR100
  roslaunch mir_driver mir.launch 

* Parameters
  roslaunch mobile_robot parameters.launch 

* Launch control node
  rosrun mobile_robot collaborative_transportation_node 

## Version

* **ROS:** Melodic

## Authors

* **Mancus Cristian**

