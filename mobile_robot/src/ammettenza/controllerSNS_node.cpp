#include "ammettenza/controllerSNS.h"

int main(int argc, char **argv){

		ros::init(argc, argv, "controllerSNS_node");
		ControllerSNS* solver = new ControllerSNS();

		ros::Rate r(10);
		while(ros::ok()) {

			ros::spinOnce();
			solver->Spinner();
			r.sleep();
		}
		
		delete solver;

//return 0;

}