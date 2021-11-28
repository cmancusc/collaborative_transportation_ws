#include <ros/ros.h>
#include "ros/time.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

void PlatformCallback(const nav_msgs::Odometry& msg){
   // the incoming message, in particular the orientation is transformed to a tf::Quaterion
   float ciao = msg.pose.pose.orientation.z;
   tf::Quaternion quat;
   tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);

   // the tf::Quaternion has a method to acess roll pitch and yaw
   double roll, pitch, yaw;
   tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
   double theta = roll;
   double phi = pitch;
   double psi = yaw;
   ROS_INFO("YAW angle: %.5f", ciao);
   ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", theta, phi, psi);

//ROS_INFO_STREAM("Velocità angolare: "<<"theta X= "<< w_x << "theta Y= "<< w_y <<"theta Z= "<< w_z );
}

int main(int argc, char** argv){
    
   ros::init(argc, argv, "mobile_subscriber");
   ros::NodeHandle nh;
   ros::Subscriber sub = nh.subscribe("/odom_comb", 100, &PlatformCallback);

   ros::spin();
   return 0;
}
