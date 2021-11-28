#include "ammettenza/controller_mobile.h"

int main(int argc, char **argv){

		ros::init(argc, argv, "controller_node");
		Controller_mobile* solver = new Controller_mobile();

		ros::Rate r(10);
		while(ros::ok()) {

			ros::spinOnce();
			solver->Spinner();
			r.sleep();
		}
		
		delete solver;

//return 0;

}
