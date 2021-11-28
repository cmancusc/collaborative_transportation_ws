#include "ammettenza_real/mobile_real.h"


int main(int argc, char **argv){

		ros::init(argc, argv, "mobile_real_node");
		Controller_mobile_real* solver = new Controller_mobile_real();

		ros::Rate r(50);
		while(ros::ok()) {

			ros::spinOnce();
			solver->Spinner();
			r.sleep();
		}
		
		delete solver;

//return 0;

}