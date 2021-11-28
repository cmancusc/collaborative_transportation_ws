#include "ammettenza/admittance_tank_cbf.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "admittance_tank__cbf_node");
    Admittance_Tank_CBF* solver = new Admittance_Tank_CBF();

    int cont = 1;
    ros::Rate r(10);

    while(ros::ok()) {

      if(cont > 10){
      ros::spinOnce();
      solver->Spinner();
      }
      cont++;

      r.sleep();
    }

    delete solver;

//return 0;

}
