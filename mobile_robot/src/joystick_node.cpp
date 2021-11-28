#include <ros/ros.h>
#include "ros/time.h"
#include <sensor_msgs/Joy.h>
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

class controller {
   public:
      controller(){
         sub = nh.subscribe("/joy", 100, &controller::joyCallback, this);
         pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
      }
   
      void joyCallback(const sensor_msgs::Joy& msg){

         ROS_INFO_STREAM("controller heard: " << msg.axes[1] << "\n");

         vel_x = msg.axes[1];
         }

      void moveBase(){
         geometry_msgs::Twist pubMsg;

         if(vel_x < 0.2 && vel_x > -0.2)
            pubMsg.linear.x = 0;
         else
            pubMsg.linear.x = vel_x * 2;

        // else if(vel_x == 1)
          //  pubMsg.linear.x =-2;
        
         //pubMsg.angular.z = 3.14;
         pub.publish(pubMsg);
      }

      double vel_x;
      ros::NodeHandle nh;
      ros::Subscriber sub;
      ros::Publisher pub;
};



int main(int argc, char** argv){
    
   ros::init(argc, argv, "joystick_node");
   controller ctrl;
   ros::AsyncSpinner spinner(1);
   spinner.start();
  
   ros::Rate loop_rate(100);
  
   while (ros::ok()){
      
      ctrl.moveBase();

      ros::spinOnce();
      loop_rate.sleep();
   }
   
  ros::shutdown();
  return 0;
}
