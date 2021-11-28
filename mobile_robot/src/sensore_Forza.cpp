#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"
#include <fstream>
#include <Eigen/Eigen>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 8, 6> Matrix86d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;

//int setValeurPoint(trajectory_msgs::JointTrajectory* traiettoria,int pos_tab, int val);

class Sensor {
   public:
    Sensor(){

        force_sub = nh.subscribe("/wrench", 1, &Sensor::ForceCallback, this);
        odometry_sub = nh.subscribe("/odom_enc", 1, &Sensor::OdometryCallback, this);
        joints_state_sub= nh.subscribe("/joint_states", 1, &Sensor::JointStateCallback, this);

        std::stringstream file_path;
        file_path << "/home/cristian/catkin_ws/src/mobile_robot/debug/force_debug.txt";
        force_file.open(file_path.str());

        //######### Create the UR10e message ##############//

        robot_model_loader = robot_model_loader::RobotModelLoader ("robot_description");
        kinematic_model = robot_model_loader.getModel();
        kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();
        joint_model_group = kinematic_model->getJointModelGroup("manipulator");
        joint_names = joint_model_group->getJointModelNames();

    }


    ~Sensor(){

        force_file.close();

    }


    void ForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg){


        geometry_msgs::WrenchStamped force_sensor = *msg;

        F_ext.setZero();

        F_ext(0) = force_sensor.wrench.force.x;
        F_ext(1) = force_sensor.wrench.force.y;
        F_ext(2) = force_sensor.wrench.force.z;
        F_ext(3) = force_sensor.wrench.torque.x;
        F_ext(4) = force_sensor.wrench.torque.y;
        F_ext(5) = force_sensor.wrench.torque.z;


//        for (int i = 0; i < F_ext.rows(); i++) {
//          if(i<3){
//            if(abs(F_ext(i)) <= 5.0){
//              F_ext(i) = 0.0;
//            }
//          }
//          else {
//            if(abs(F_ext(i)) <=0.5){
//              F_ext(i) = 0.0;
//            }
//          }

//        }

//        for(int i = 0; i < 3; i++){
//            if((F_ext(i) > 0.0) && (F_ext(i) < 4.0))
//                F_ext(i) = 0.0;

//            else if((F_ext(i) < 0.0) && (F_ext(i) > - 4.0))
//                F_ext(i) = 0.0;
//        }

//        for(int i = 3; i < F_ext.rows(); i++){
//            if((F_ext(i) > 0.0) && (F_ext(i) < 0.5))
//                F_ext(i) = 0.0;

//            else if((F_ext(i) < 0.0) && (F_ext(i) > - 0.5))
//                F_ext(i) = 0.0;
//        }



    }


    void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){


        joint_state = *msg;

        // Ur10e Real Robot has Inverted Joints
        std::swap(joint_state.name[0], joint_state.name[2]);
        std::swap(joint_state.effort[0], joint_state.effort[2]);
        std::swap(joint_state.position[0], joint_state.position[2]);
        std::swap(joint_state.velocity[0], joint_state.velocity[2]);

        for (unsigned int i = 0; i < joint_state.position.size(); i++) {joint_real_position[i] = joint_state.position[i];}
        for (unsigned int i = 0; i < joint_state.velocity.size(); i++) {joint_real_velocity[i] = joint_state.velocity[i];}

    }


    void OdometryCallback(const nav_msgs::Odometry& msg){


        // the incoming message, in particular the orientation is transformed to a tf::Quaterion
        //float prova = msg.pose.pose.orientation.z;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
        double theta, phi, psi;
        // the tf::Quaternion has a method to acess roll pitch and yaw
        tf::Matrix3x3(quat).getRPY(theta, phi, psi);
        roll = theta;
        pitch = phi;
        yaw = psi;
        //ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", roll, pitch, yaw);
        //theta = msg.pose.pose.orientation.w;

        vel_x = msg.twist.twist.linear.x;
        //vel_x = pow( pow(msg.twist.twist.linear.x, 2) + pow(msg.twist.twist.linear.y, 2) , 2);
        omega_z = msg.twist.twist.angular.z;

    }



    Matrix6d get_ee_rotation_matrix(double joint_position[], double joint_velocity[]) {

        ros::spinOnce();

        //Update MoveIt! Kinematic Model
        kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
        kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
        kinematic_state->enforceBounds();

        // Computing the actual position of the end-effector using Forward Kinematic respect "ur10e_base_link"
        const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

        // Rotation Matrix 6x6
        Matrix6d rotation_matrix;
        rotation_matrix.setZero();

        Eigen::Matrix3d Rot_ee_baseUR;

        Rot_ee_baseUR = end_effector_state.rotation();

        //############# Nel frame del MIR ################
        //Rot_ee_odom = Rot_ArmBase_Platform * end_effector_state.rotation();

        rotation_matrix.topLeftCorner(3, 3) = Rot_ee_baseUR;
        rotation_matrix.bottomRightCorner(3, 3) = Rot_ee_baseUR;

        //Eigen::Vector3d euler_angles = end_effector_state.rotation().eulerAngles(0, 1, 2);
        //Eigen::Quaterniond rotation_quaternion(end_effector_state.rotation());

        return rotation_matrix;

    }



    Vector6d Force_in_odom(Vector6d force_ur_base){

      Matrix3d Rot_ArmBase_Platform;
      Matrix3d Rot_Platform_b;
      Matrix3d Rot_b_World;

      double angle = 135*M_PI/180.0;
    //----------------------------- Arm Base frame --> Platform frame  ----------------------------------//

      Rot_ArmBase_Platform <<  cos(angle),  -sin(angle),   0.0,
                               sin(angle),   cos(angle),   0.0,
                               0.0,         0.0,   1.0;


    //--------------------------- Platform Frame --> I-O SFL Frame (b_) -------------------------------//

      Rot_Platform_b = Eigen::Matrix3d::Identity(3,3);


    //------------------------------- I-O SFL Frame --> World Frame ----------------------------------//

      Rot_b_World << cos(yaw), -sin(yaw), 0.0,
                     sin(yaw),  cos(yaw), 0.0,
                        0.0,      0.0,    1.0;


     Matrix6d Transformation_in_odom;

     Transformation_in_odom << Rot_b_World * Rot_Platform_b * Rot_ArmBase_Platform,
                               Eigen::Matrix3d::Zero(3,3),
                               Eigen::Matrix3d::Zero(3,3),
                               Rot_b_World * Rot_Platform_b * Rot_ArmBase_Platform;


     Vector6d Force_vector;

     Force_vector = Transformation_in_odom * force_ur_base;

     return  Force_vector;
    }


      double joint_real_position[6];
      double joint_real_velocity[6];
      double roll, pitch, yaw, vel_x, omega_z;

      Matrix3d Rot_ArmBase_Platform;

      Vector6d F_ext;
      ros::NodeHandle nh;
      ros::Subscriber force_sub;
      ros::Subscriber joints_state_sub;
      ros::Subscriber odometry_sub;

      std::ofstream force_file;

      sensor_msgs::JointState joint_state;

      //############ MoveIt Robot Model ################//

      robot_model_loader::RobotModelLoader robot_model_loader;
      robot_model::RobotModelPtr kinematic_model;
      robot_state::RobotStatePtr kinematic_state;
      const robot_state::JointModelGroup *joint_model_group;
      std::vector<std::string> joint_names;


    };


int main(int argc, char** argv) {
	
    ros::init(argc, argv, "sensore_Forza");

    Sensor force;

    ros::Rate loop_rate(50);

    while(ros::ok()) {

            Vector6d Force_ur_base;
            Vector6d Force_odom;

            Force_ur_base = force.get_ee_rotation_matrix(force.joint_real_position, force.joint_real_velocity) * force.F_ext;

            Force_odom = force.Force_in_odom(Force_ur_base);
            ROS_INFO_STREAM("FORCE IN TOOL0 FRAME: \n" << force.F_ext);
            ROS_INFO_STREAM("FORCE IN WORLD FRAME: \n" << Force_odom);

            for(int i = 0; i < 6; i++)
                force.force_file << " " << Force_odom(i);

            force.force_file << std::endl;

            loop_rate.sleep();
            
    }
    ros::shutdown();
    return 0;
}
