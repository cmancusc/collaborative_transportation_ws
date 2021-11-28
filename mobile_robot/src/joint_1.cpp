#include "joint_1.h"

controller_functions::controller_functions() {

    //Initialize publisher on "/prbt/manipulator_joint_trajectory_controller/command"
    arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur10e/manipulator_joint_trajectory_controller/command", 1000);
    wrench_sub_ = nh.subscribe("wrench", 1, &controller_functions::forceCallback, this);

}

//* Callback for the wrench coming from the Ati Mini 45 F/T sensor
void controller_functions::forceCallback(const geometry_msgs::WrenchStamped& msg){

    F_ext_[0] = msg.wrench.force.x;
    F_ext_[1] = msg.wrench.force.y;
    F_ext_[2] = msg.wrench.force.z;
    // F_ext_[3] = msg.wrench.torque.x;
    // F_ext_[4] = msg.wrench.torque.y;
    // F_ext_[5] = msg.wrench.torque.z;
    // F_ext_[1] = 0.0;
    // F_ext_[2] = 0.0;
    F_ext_[3] = 0.0;
    F_ext_[4] = 0.0;
    F_ext_[5] = 0.0;

    for(int i = 0; i < F_ext_.size(); i++){
        ROS_ERROR_STREAM("FORCE " << i << ": " << F_ext_[i]);
    }
}

void controller_functions::Ciao()
{
    x = 2.0;
    y = 3.0;

    s = x + y;
    p = x * y;

    ROS_INFO("LA SOMMA VALE: %.2f", s);
}

int main(int argc, char** argv) {
	
    ros::init(argc, argv, "joint_1");
    
    controller_functions prova;

    ros::Rate loop_rate(10);
   
    prova.traj.header.frame_id = "ur10e_base_link";
    prova.traj.joint_names.resize(6);
    prova.traj.points.resize(1);

    prova.traj.joint_names[0] ="ur10e_shoulder_pan_joint";
    prova.traj.joint_names[1] ="ur10e_shoulder_lift_joint";
    prova.traj.joint_names[2] ="ur10e_elbow_joint";
    prova.traj.joint_names[3] ="ur10e_wrist_1_joint";
    prova.traj.joint_names[4] ="ur10e_wrist_2_joint";
    prova.traj.joint_names[5] ="ur10e_wrist_3_joint";
    
    prova.Ciao();
    ROS_INFO("il prodotto vale: %.2f", prova.p);
    

    while(ros::ok()) {

        prova.traj.header.stamp = ros::Time::now();
        prova.traj.points[0].time_from_start = ros::Duration(1);

        prova.traj.points[0].positions = {0, 0, -1, 0, 0, 0};
           
        prova.arm_pub.publish(prova.traj);
        
        //ROS_INFO("il prodotto vale: %.2f", prova::Ciao().p);

        ros::spinOnce();

        loop_rate.sleep();
            
    }

    return 0;
}
