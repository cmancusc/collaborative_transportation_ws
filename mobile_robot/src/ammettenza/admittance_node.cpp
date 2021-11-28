#include "ammettenza/admittance.h"

int main(int argc, char **argv){

		ros::init(argc, argv, "admittance_node");
		Admittance* solver = new Admittance();

		ros::Rate r(10);
		while(ros::ok()) {

			ros::spinOnce();
			solver->Spinner();
			r.sleep();
		}
		
		delete solver;

//return 0;

}