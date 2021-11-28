#include <ros/ros.h>
#include "ros/time.h"
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
    
   ros::init(argc, argv, "mobile_publisher");
   ros::NodeHandle n;
   ros::Publisher mobile_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
   ros::Rate loop_rate(10);

   geometry_msgs::Twist message;

   while (ros::ok())
   {
        message.linear.x = 2;
        message.angular.z = -0.8;

        mobile_pub.publish(message);

        ros::spinOnce();
        loop_rate.sleep();
   }
   
   return 0;
}
