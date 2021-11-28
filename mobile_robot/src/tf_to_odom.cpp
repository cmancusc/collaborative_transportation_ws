#include <ros/ros.h>
#include <cmath>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <eigen3/Eigen/Core>
#include <Eigen/Eigen>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_to_odom");
  
  tf2_ros::Buffer* tfBuffer;
  tf2_ros::TransformListener* tf2_listener;

  tf2_ros::Buffer* tfBuffer_b_w;
  tf2_ros::TransformListener* tf2_listener_b_w;

  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Rate loop_rate(500);
  
  while (ros::ok())
  {
    
    tfBuffer = new tf2_ros::Buffer;
    tf2_listener = new tf2_ros::TransformListener(*tfBuffer);

    tfBuffer_b_w = new tf2_ros::Buffer;
    tf2_listener_b_w = new tf2_ros::TransformListener(*tfBuffer_b_w);

//########################### transform base manipulator to base platform ###############
    geometry_msgs::TransformStamped transform = tfBuffer->lookupTransform("ur10e_base_link", "ur10e_tool0", 
                                                ros::Time(0), ros::Duration(6.0));
    
    ROS_INFO_STREAM("transformata: \n" << transform << "\n");

    Eigen::Affine3d tfToEigen = tf2::transformToEigen(transform);
    Eigen::MatrixXd transformToPlatform = tfToEigen.matrix();
    ROS_INFO_STREAM("Matrice di trasformazione: \n" << transformToPlatform << "\n");


    Eigen::Vector4d ee_platform_pose;
    double x_ee_platform, y_ee_platform, z_ee_platform;

    ee_platform_pose = transformToPlatform.col(3); 
    x_ee_platform = ee_platform_pose[0];
    y_ee_platform = ee_platform_pose[1];
    z_ee_platform = ee_platform_pose[2];

    ROS_INFO_STREAM("X END EFFECTOR: " << x_ee_platform << "\n");
    ROS_INFO_STREAM("Y END EFFECTOR: " << y_ee_platform << "\n");
    ROS_INFO_STREAM("Z END EFFECTOR: " << z_ee_platform << "\n");


//########################## transform base platform to odom ##################
/*
    geometry_msgs::TransformStamped transform_b_w = tfBuffer_b_w->lookupTransform("mobile_odom_comb", "mobile_base_footprint", 
                                                ros::Time(0), ros::Duration(6.0));
    
    ROS_INFO_STREAM("transformata: \n" << transform_b_w << "\n");

    Eigen::Affine3d tfToEigen_b_w = tf2::transformToEigen(transform_b_w);
    Eigen::MatrixXd transformToOdom = tfToEigen_b_w.matrix();
    ROS_INFO_STREAM("Matrice di trasformazione: \n" << transformToOdom << "\n");


    Eigen::Vector4d platform_odom_pose;
    double x_platform_odom, y_platform_odom, z_platform_odom;

    platform_odom_pose = transformToOdom.col(3); 
    x_platform_odom = platform_odom_pose[0];
    y_platform_odom = platform_odom_pose[1];
    z_platform_odom = platform_odom_pose[2];

    ROS_INFO_STREAM("X END EFFECTOR: " << x_platform_odom << "\n");
    ROS_INFO_STREAM("Y END EFFECTOR: " << y_platform_odom << "\n");
    ROS_INFO_STREAM("Z END EFFECTOR: " << z_platform_odom << "\n");

*/


    ros::spinOnce();
    loop_rate.sleep();
   }
   
  ros::shutdown();
  return 0;
}
