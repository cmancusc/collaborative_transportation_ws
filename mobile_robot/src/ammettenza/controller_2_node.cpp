#include "ammettenza/controller_2.h"

int main(int argc, char **argv){

		ros::init(argc, argv, "controller_2_node");
		Controller_2* solver = new Controller_2();

		ros::Rate r(10);
		while(ros::ok()) {

			ros::spinOnce();
			solver->Spinner();
			r.sleep();
		}
		
		delete solver;

//return 0;

}