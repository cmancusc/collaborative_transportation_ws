#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "ros/time.h"
#include <fstream>
#include <Eigen/Eigen>


typedef Eigen::Matrix<double, 8, 8> Matrix8d;
typedef Eigen::Matrix<double, 8, 6> Matrix86d;
typedef Eigen::Matrix<double, 6, 8> Matrix68d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 2> Matrix62d;
typedef Eigen::Matrix<double, 2, 6> Matrix26d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;

typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 8, 1> Vector8d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;

//int setValeurPoint(trajectory_msgs::JointTrajectory* traiettoria,int pos_tab, int val);

class controller {
   public:
    controller(){
            measure_hat.setZero();
            P_.setZero();
            K_.setZero();

            std::vector<double> Q_dot_parameters;

            if (!nh.getParam("q_dot_noise", Q_dot_parameters)) {
              ROS_ERROR("Couldn't retrieve the desired q_dot of the robot.");
            }

            Q_dot_Noise = Eigen::Map<Eigen::Matrix<double, 207, 9> >(Q_dot_parameters.data());

            std::stringstream file_path;
            file_path << "/home/cristian/catkin_ws/src/mobile_robot/debug/q_dot_filtered.txt";
            q_dot_filtered.open(file_path.str());
    }

    ~controller(){
      q_dot_filtered.close();
    }

    Vector9d KALMAN(Vector9d measure){
    //    Vector8d R; //noise covariance
    //    Vector8d H; //measurement map scalar
    //    Vector8d Q; //initial estimated covariance
    //    Vector8d P; //initial error covariance
    //    Vector8d measure_hat; //initial estimated state
    //    Vector8d K; //kalman gain

        // a higher R reduces the Kalman Gain (K), which fileters
        // more noise, however slows down the speed of the filter
        static const double R = 0.33; // (10 .. 0.33) noise covariance
        static const double H = 1.0; //measurement map scalar
        static double Q = 15; // (15) initial estimated covariance
    //    Vector8d P; //initial error covariance
    //    Vector8d measure_hat; //initial estimated state
        // the higher K is, the faster the filter responds,
        // however the less noise it will be remove
    //    Vector8d K; //kalman gain
        Vector9d estimated;

        for(int i = 0; i < measure.rows(); i++){
            K_(i) = P_(i) * H / (H * P_(i) * H + R); // update the gain
            measure_hat(i) += K_(i) * (measure(i) - H * measure_hat(i)); //update the estimation
            P_(i) = (1 - K_(i) * H) * P_(i) + Q;
        }


        estimated = measure_hat;
        return estimated;
      }

      Vector9d P_; //initial error covariance
      Vector9d measure_hat; //initial estimated state
      Vector9d K_; //kalman gain
      Eigen::MatrixXd Q_dot_Noise;

      std::ofstream q_dot_filtered;

      ros::NodeHandle nh;
    };


int main(int argc, char** argv) {
	
    ros::init(argc, argv, "prova_Kalman");

    controller ctrl;

    ros::Rate loop_rate(50);

    int count = 0;

    while(ros::ok()) {
        
            Vector9d noise_vector;
            Vector9d estimated_vector;

            noise_vector = ctrl.Q_dot_Noise.row(count);
            ROS_INFO_STREAM("ROW %d" << count << ": \n" <<noise_vector);
            estimated_vector = ctrl.KALMAN(noise_vector);


            for(int i = 0; i < 9; i++)
                ctrl.q_dot_filtered << " " << estimated_vector(i);

            ctrl.q_dot_filtered << std::endl;

            count ++;

            if(count == 207){
              ros::shutdown();
            }

            loop_rate.sleep();
            
    }
    //ros::shutdown();
    return 0;
}
