#include "ammettenza/controller_arm.h"

int main(int argc, char **argv){

		ros::init(argc, argv, "controller_arm_node");
		Controller_arm* solver = new Controller_arm();

		ros::Rate r(10);
		while(ros::ok()) {

			ros::spinOnce();
			solver->Spinner();
			r.sleep();
		}
		
		delete solver;

//return 0;

}