#include "joint_prova.cpp"

int main(int argc, char **argv){

		ros::init(argc, argv, "joint_prova_node");
		controller* solver = new controller();

		ros::Rate r(50); // Was 500
		while(ros::ok()) {

			ros::spinOnce();
			solver->Spinner();
			r.sleep();
		}
		
		delete solver;

//return 0;

}
