#ifndef COLLABORATIVE_TRANSPORTATION_H_
#define COLLABORATIVE_TRANSPORTATION_H_
#define _USE_MATH_DEFINES

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
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

//Eigen
#include <eigen3/Eigen/Core>
#include <Eigen/Eigen>

//#include <eigen3/Eigen/Core>
#include <Eigen/Eigen>

#include "MATLAB_TANK/MATLAB_OPT_TANK.h"
//#include "MATLAB_TANK_CBF/MATLAB_OPT_TANK_CBF.h"
//#include "MATLAB_4_CBF/MATLAB_4_CBF.h"


// extern "C" {
//     #include "solver.h"
// }


typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 8, 6> Matrix86d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;



class System_Controller {

		public:

			System_Controller();
			~System_Controller();

      MATLAB_OPT_TANK MATLAB_OPT;
//      MATLAB_OPT_TANK_CBF MATLAB_OPT;
//      MATLAB_4_CBF MATLAB_OPT;

			//##################### CALLBACK'S FUNCTIONS ######################//
			void OdometryCallback(const nav_msgs::Odometry& msg);
			void RuoteCallback(const sensor_msgs::JointState& msg);
			void JointStateCallback(const sensor_msgs::JointState::ConstPtr &);
			//void joystickCallback(const sensor_msgs::Joy& msg);
			void ForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

			//###################### UR10e KINEMATICS #########################//
			Matrix4d compute_arm_fk (double joint_position[], double joint_velocity[]);
      Vector6d compute_ee_pose(double joint_position[], double joint_velocity[]);
			Eigen::MatrixXd compute_arm_jacobian (double joint_position[], double joint_velocity[]);
			Matrix62d compute_platform_jacobian(double rotation);

			//####################### MAPPING FUNCTIONS #######################//
			Matrix6d Mir_Mapping(double rotation);
			Matrix6d Arm_Mapping(double rotation);
			Matrix6d Transformata();
			Matrix6d get_ee_rotation_matrix (double joint_position[], double joint_velocity[]);
      Vector6d Force_in_Odom_Frame();

			//###################### IMPOSE JOINT'S CONSTRAINS ################//
			Vector8d limit_joints_dynamics (Vector8d joint_velocity);
			//Vector2d limit_wheels_dynamics(Vector2d wheel_velocity);
			Vector8d SatNullSpace(Vector6d cartesian_velocity, Eigen::MatrixXd system_jacobian, Vector8d Vel_LowerLimit, Vector8d Vel_UpperLimit);

			int sign (double num);
			
			void send_velocity_to_robot (Vector6d velocity);
			
      Vector8d KALMAN_8(Vector8d measure);

			//##### MAIN FUNCTION #####//
			void Spinner();

      Vector8d P_8; //initial error covariance
      Vector8d measure_hat_8; //initial estimated state
      Vector8d K_8; //kalman gain

			Matrix3d Rot_ArmBase_Platform;
      Matrix3d Rot_Platform_b;
      Matrix3d Rot_b_World;

			Vector3d ee_ArmBase_pose;
			Vector3d ArmBase_platform_pose;
			Vector3d platform_b_pose;
			Vector3d b_world_pose;
			Vector3d W_r_b_E;
			Vector3d W_r_B_E;

			Matrix6d M_tot, D_tot;
			Vector8d acc_limits;
			Vector8d vel_limits;
//      Vector8d vel_limits_CBFclose;
			Vector8d adm_qdot_last_cycle;
			Vector8d q_dot_SNS_last_cycle;
			//Vector8d q_dot_SNS;
			//Vector8d Vel_LowerLimit;
      //Vector8d Vel_UpperLimit;
			Vector8d Acc_LowerLimit;
      Vector8d Acc_UpperLimit;
			Vector8d Max_Limits;
			Vector8d Min_Limits;
			Vector8d opt_qdot_prev;
			
			Vector6d F_ext;

			Vector6d fixed_damping;
      Vector6d fixed_mass;
			Vector6d min_damping;
			Vector6d max_damping;

			//############## Tank Variables ##############//

			double sum_of_delta_;
      double tank_state_;
      double tank_energy_;
      const double TANK_INITIAL_VALUE = 40; // was 20
      const double TANK_MAX_VALUE = 500;
      const double TANK_MIN_VALUE = 5;


		private:

			ros::NodeHandle nh;

			//#### Subscribers #########//

      ros::Subscriber odometry_sub;
			//ros::Subscriber joystick_sub;
			ros::Subscriber force_sub;
			ros::Subscriber joints_state_sub;
			ros::Subscriber mir_joint_sub;

      //#### Publishers #########//

      ros::Publisher arm_pub;
      ros::Publisher mobile_pub;
      ros::Publisher tank_node;
      ros::Publisher h_1_node;
      ros::Publisher h_2_node;

			//############### Topic Messages #################//

      sensor_msgs::JointState joint_state;
			trajectory_msgs::JointTrajectory arm_message;
      geometry_msgs::Twist base_message;
      std_msgs::Float32 energia;
      std_msgs::Float32 h_1_message;
      std_msgs::Float32 h_2_message;

      //############### File ############################//

      std::ofstream q_dot_SNS_file;
			std::ofstream q_dot_file;
			std::ofstream force_file;
//      std::ofstream force_filtered_file;
			std::ofstream q_dot_opt_file;
      std::ofstream q_dot_opt_filtered_file;
			std::ofstream tank_file;
      std::ofstream parametri_ammettenza;
      double start_time_;

      //################## MIR100 #######################//

      double cycle_time;
			double roll, pitch, yaw, vel_x, omega_z, omega_l, omega_r;
			double x_platform, y_platform, z_platform;
			//define the wheel radius and the lenght of wheel's  axis
      double R, L, b_;

      //################## UR10e #######################//

      double joint_real_position[6];
      double joint_real_velocity[6];

      //############ MoveIt Robot Model ################//

      robot_model_loader::RobotModelLoader robot_model_loader;
			robot_model::RobotModelPtr kinematic_model;
			robot_state::RobotStatePtr kinematic_state;
			const robot_state::JointModelGroup *joint_model_group;
			std::vector<std::string> joint_names;

};

#endif /* COLLABORATIVE_TRANSPORTATION_H_ */

