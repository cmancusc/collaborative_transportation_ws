#include "ammettenza_real/collaborative_transportation.h"
#include <signal.h>

void Shutdown_Signal_Handler (int sig) {

    ros::NodeHandle nh;
    
    std::string topic_joint_group_vel_controller_publisher;
    nh.param<std::string>("/admittance_controller_Node/topic_joint_group_vel_controller", topic_joint_group_vel_controller_publisher, "/joint_group_vel_controller/command");

    ros::Publisher joint_group_vel_controller_publisher = nh.advertise<std_msgs::Float64MultiArray>(topic_joint_group_vel_controller_publisher, 1);

    std_msgs::Float64MultiArray stop_msgs;
    std::vector<double> stop_vector;
    stop_vector.resize(6, 0.0);

    stop_msgs.layout.dim.push_back(std_msgs::MultiArrayDimension());
    stop_msgs.layout.dim[0].size = stop_vector.size();
    stop_msgs.layout.dim[0].stride = 1;
    stop_msgs.layout.dim[0].label = "velocity";

    // copy in the data
    stop_msgs.data.clear();
    stop_msgs.data.insert(stop_msgs.data.end(), stop_vector.begin(), stop_vector.end());

    joint_group_vel_controller_publisher.publish(stop_msgs);

    ros::shutdown();

}

int main(int argc, char **argv){

		ros::init(argc, argv, "collaborative_transportation_node", ros::init_options::NoSigintHandler);

		signal(SIGINT, Shutdown_Signal_Handler);
		System_Controller* solver = new System_Controller();

    int cont = 1;

    ros::Rate r(50);

		while(ros::ok()) {

      ros::spinOnce();

      if(cont > 10){
        solver->Spinner();
      }
      cont++;

			r.sleep();

		}
		
		delete solver;

//return 0;

}
