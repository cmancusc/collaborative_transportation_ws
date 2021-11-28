#include "ammettenza/admittance_tank.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "admittance_tank_node");
    Admittance_Tank* solver = new Admittance_Tank();

    ros::Rate r(10);
    while(ros::ok()) {

      ros::spinOnce();
      solver->Spinner();
      r.sleep();
    }

    delete solver;

//return 0;

}
