#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"

ros::Publisher arm_pub_v2;

int main(int argc, char** argv) {
	
	bool abort;
	bool vel_request;
	float time;

    ros::init(argc, argv, "joint_2");
    ros::NodeHandle n;
    //arm_pub_v2 = n.advertise<trajectory_msgs::JointTrajectory>("/ur10e/manipulator_joint_trajectory_controller/command",1);
    arm_pub_v2 = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
    ros::Rate loop_rate(10);

    trajectory_msgs::JointTrajectory traj;
   
    traj.header.frame_id = "base_link";
    traj.joint_names.resize(6);
    traj.points.resize(1);

    traj.joint_names[0] ="shoulder_pan_joint";
    traj.joint_names[1] ="shoulder_lift_joint";
    traj.joint_names[2] ="elbow_joint";
    traj.joint_names[3] ="wrist_1_joint";
    traj.joint_names[4] ="wrist_2_joint";
    traj.joint_names[5] ="wrist_3_joint";


    while(ros::ok()) {

        traj.header.stamp = ros::Time::now();
        traj.points[0].time_from_start = ros::Duration(1);

        traj.points[0].positions = {0, -2, 1.5, 0, 0, 0};
           
        arm_pub_v2.publish(traj);
        ros::spinOnce();

        loop_rate.sleep();
            
    }

    return 0;
}
