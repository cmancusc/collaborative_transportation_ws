#ifndef JOINT_1_H_
#define JOINT_1_H_
#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include "nav_msgs/Odometry.h"
#include <control_msgs/JointTrajectoryControllerState.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Eigen>


class controller_functions {

		public:

			controller_functions();
			void forceCallback(const geometry_msgs::WrenchStamped& msg);


			void Ciao();
			double p, s, x, y;
			trajectory_msgs::JointTrajectory traj;
			ros::Publisher arm_pub;
			ros::Subscriber wrench_sub_;
			
		
		private:
			Eigen::Matrix<double, 6, 1> F_ext_;
        	ros::NodeHandle nh;
        	

};

#endif /* JOINT_1_H_ */


