#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"
#include <sensor_msgs/Joy.h>


//int setValeurPoint(trajectory_msgs::JointTrajectory* traiettoria,int pos_tab, int val);

class controller {
   public:
    controller(){
        arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("/ur10e/manipulator_joint_trajectory_controller/command",1);
        //arm_pub = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
        //joystick_sub = n.subscribe("/joy", 100, &controller::joyCallback, this);

        //traj.header.frame_id = "ur10e_base_link";
        traj.joint_names.resize(6);
        traj.points.resize(1);


        traj.joint_names[0] ="ur10e_shoulder_pan_joint";
        traj.joint_names[1] ="ur10e_shoulder_lift_joint";
        traj.joint_names[2] ="ur10e_elbow_joint";
        traj.joint_names[3] ="ur10e_wrist_1_joint";
        traj.joint_names[4] ="ur10e_wrist_2_joint";
        traj.joint_names[5] ="ur10e_wrist_3_joint";
/*
        traj.joint_names[0] ="shoulder_pan_joint";
        traj.joint_names[1] ="shoulder_lift_joint";
        traj.joint_names[2] ="elbow_joint";
        traj.joint_names[3] ="wrist_1_joint";
        traj.joint_names[4] ="wrist_2_joint";
        traj.joint_names[5] ="wrist_3_joint";
*/
        }


        void Spinner() {
            
            bool abort;
            bool vel_request;
            float time;

            double pos_joint_1, pos_joint_2, pos_joint_3, pos_joint_4, pos_joint_5, pos_joint_6;
            //double vel_joint_1, vel_joint_2, vel_joint_3, vel_joint_4, vel_joint_5, vel_joint_6;
            //double acc_joint_1, acc_joint_2, acc_joint_3, acc_joint_4, acc_joint_5, acc_joint_6;

            //traj.header.stamp = ros::Time::now();
            traj.points[0].time_from_start = ros::Duration(0.001);

            traj.points[0].positions = {0.0, -1.5, -1.5, 0.0, 1.5, 0.0};
            
            ROS_INFO("UR10e GOTO (0, -1.5, -1.5, 0.0, 1.5, 0.0)");
            //ROS_INFO("UR10 GOTO (%.2f;%.2f;%.2f;%.2f;%.2f;%.2f)", pos_joint_1, pos_joint_2, pos_joint_3, pos_joint_4, pos_joint_5, pos_joint_6);
            ROS_INFO("Movement Time (%.2f)", time);
            arm_pub.publish(traj);
       
        }


        //double pos_x, pos_y;
        ros::NodeHandle n;
        //ros::Subscriber joystick_sub;
        ros::Publisher arm_pub;

        trajectory_msgs::JointTrajectory traj;

};
