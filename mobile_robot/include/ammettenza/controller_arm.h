#ifndef CONTROLLER_ARM_H_
#define CONTROLLER_ARM_H_
#define _USE_MATH_DEFINES
#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

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

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>

//#include <eigen3/Eigen/Core>
#include <Eigen/Eigen>

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Array<double, 6, 1> Array6d;

class Controller_arm {

		public:
			Controller_arm();
			void JointStateCallback(const sensor_msgs::JointState::ConstPtr &);
			void joystickCallback(const sensor_msgs::Joy& msg);
			Eigen::Matrix4d compute_arm_fk (double joint_position[], double joint_velocity[]);
			Eigen::MatrixXd compute_arm_jacobian (double joint_position[], double joint_velocity[]);
			Vector6d limit_joint_dynamics (Vector6d joint_velocity);
			int sign (double num);
			Matrix6d Mapping();
            void Spinner();		

			Eigen::MatrixXd Transform_ee_platform;

			Eigen::MatrixXd F_ext;
			//Eigen::Matrix<double, 6, 6> M_tot, D_tot;
			Matrix6d M_tot_prova, D_tot_prova;

			double cycle_time;

			double joint_real_position[6];
			double joint_real_velocity[6]; 

			Vector6d adm_twist_last_cycle;

			ros::Publisher arm_pub;

			trajectory_msgs::JointTrajectory arm_message;

		private:

			ros::NodeHandle nh;
			ros::Subscriber joystick_sub;
			ros::Subscriber joints_state_sub;
			
			sensor_msgs::JointState joint_state;

			// ---- MoveIt Robot Model ---- //
			robot_model_loader::RobotModelLoader robot_model_loader;
			robot_model::RobotModelPtr kinematic_model;
			robot_state::RobotStatePtr kinematic_state;
			const robot_state::JointModelGroup *joint_model_group;
			std::vector<std::string> joint_names;
			
};

#endif /*  CONTROLLER_ARM_H_ */

