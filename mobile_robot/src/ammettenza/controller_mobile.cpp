#include "ammettenza/controller_mobile.h"

Controller_mobile::Controller_mobile()
{
  odometry_sub = nh.subscribe("/base_pose_ground_truth", 1, &Controller_mobile::OdometryCallback, this);
  mir_joint_sub = nh.subscribe("/joint_states", 1, &Controller_mobile::RuoteCallback, this);
  joystick_sub = nh.subscribe("/joy", 1, &Controller_mobile::joystickCallback, this);
  mobile_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  R = 0.0625;
  L = 0.445208;
  cycle_time = 0.1;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  vel_x = 0.0;
  omega_z = 0.0;
  //IO-SFL lead value
  b_ = 0.6;

  F_ext.conservativeResize(6,1);
  F_ext << Eigen::MatrixXd::Zero(6,1);


  std::vector<double> M_tot_parameters;
  std::vector<double> D_tot_parameters;

/*
  if (!nh.getParam("mass_platform", M_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the robot.");
  }

  if (!nh.getParam("damping_platform", D_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
  }

  M_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_tot_parameters.data());
  D_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_tot_parameters.data());
*/



  if (!nh.getParam("mass_platform_prova", M_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the robot.");
  }

  if (!nh.getParam("damping_platform_prova", D_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
  }

  M_tot_prova = Eigen::Map<Eigen::Matrix<double, 2, 2> >(M_tot_parameters.data());
  D_tot_prova = Eigen::Map<Eigen::Matrix<double, 2, 2> >(D_tot_parameters.data());

}



void Controller_mobile::OdometryCallback(const nav_msgs::Odometry& msg){
  
  // the incoming message, in particular the orientation is transformed to a tf::Quaterion
  //float prova = msg.pose.pose.orientation.z;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
  double theta, phi, psi;
  // the tf::Quaternion has a method to acess roll pitch and yaw
  tf::Matrix3x3(quat).getRPY(theta, phi, psi);
  roll = theta;
  pitch = phi;
  yaw = psi;
  //ROS_INFO("published rpy angles: roll=%f pitch=%f yaw=%f", roll, pitch, yaw);
  //theta = msg.pose.pose.orientation.w;

  vel_x = pow( pow(msg.twist.twist.linear.x, 2) + pow(msg.twist.twist.linear.y, 2) , 2);
  omega_z = msg.twist.twist.angular.z;

  x_platform = msg.pose.pose.position.x;
  y_platform = msg.pose.pose.position.y;
  //z_platform = msg.pose.pose.position.z;
  z_platform = 0.0;
}

void Controller_mobile::RuoteCallback(const sensor_msgs::JointState& msg){
  omega_l = msg.velocity[0];
  omega_r = msg.velocity[1];

}

void Controller_mobile::joystickCallback(const sensor_msgs::Joy& msg){

/*
  Guida:  msg.axes[0]; //levetta sx orizzontale --> Fx
          msg.axes[1]; //levetta sx verticale --> Fy
          msg.axes[4]; //levetta dx verticale --> Fz
          msg.axes[3]; //levetta dx orizzontale --> Mx
          msg.axes[6]; //analogico orizzontale --> My
          msg.axes[7]; //analogico verticale --> Mz
 */

  double intensity = 100;

  if(abs(msg.axes[0]) > 0.5)
    F_ext(0,0) = msg.axes[0] * intensity; //levetta sx orizzontale --> Fx
  else 
    F_ext(0,0) = 0.0;

  if(abs(msg.axes[1]) > 0.5)
    F_ext(1,0) = msg.axes[1] * intensity; //levetta sx verticale --> Fy
  else
    F_ext(1,0) = 0.0;

  if(abs(msg.axes[4]) > 0.5)
    F_ext(2,0) = msg.axes[4] * intensity; //levetta dx verticale --> Fz
  else 
    F_ext(2,0) = 0.0; 
  
  if(abs(msg.axes[3]) > 0.5)
    F_ext(3,0) = msg.axes[3] * intensity; //levetta dx orizzontale --> Mx
  else 
    F_ext(3,0) = 0.0; 

  if(abs(msg.axes[6]) > 0.5)
    F_ext(4,0) = msg.axes[6] * intensity; //analogico orizzontale --> My
  else 
    F_ext(4,0) = 0.0; 

  if(abs(msg.axes[7]) > 0.5)
    F_ext(5,0) = msg.axes[7] * intensity; //analogico verticale --> Mz
    F_ext(5,0) = 0.0;


}


Matrix62d Controller_mobile::compute_platform_jacobian(double rotation)
{
  
  Matrix62d jacobian_base;

/*
  //twist_cartesiane = jocobian_base * twist_cmd_vel
  jacobian_base << R*cos(yaw)/2, R*cos(yaw)/2,
                   R*sin(yaw)/2, R*sin(yaw)/2,
                          0,            0, 
                          0,            0, 
                          0,            0, 
                          R/L,         -R/L;
*/
  //ROS_INFO_STREAM("JAcobiano della piattaforma: \n" << jacobian_base << "\n");


  double j11 = R*cos(rotation)/2 - b_*sin(rotation)*R/L;
  double j12 = R*cos(rotation)/2 + b_*sin(rotation)*R/L;
  double j21 = R*sin(rotation)/2 + b_*cos(rotation)*R/L;
  double j22 = R*sin(rotation)/2 - b_*cos(rotation)*R/L;


/*
  j11 = R/2;
  j12 = R/2;
  j21 = R/L;
  j22 = -R/L;
*/

  jacobian_base << j11, j12,
                   j21, j22,
                    0,   0, 
                    0,   0, 
                    0,   0, 
                   R/L, -R/L;

/*
  double b11, b12, b21, b22;
  b11 = cos(yaw);
  b12 = - b_ * sin(yaw);
  b21 = sin(yaw);
  b22 = b_ * cos(yaw);

  jacobian_base << b11, b12,
                   b21, b22,
                   0.0, 0.0, 
                   0.0, 0.0, 
                   0.0, 0.0, 
                   0.0, 1.0;
*/

    return jacobian_base;
}



Matrix6d Controller_mobile::Transformata(){


    Eigen::Matrix3d R_platform_world;
    R_platform_world << cos(yaw), -sin(yaw), 0,
                        sin(yaw),  cos(yaw), 0,
                            0,         0,    1;

    Vector3d platf_world_pose;
    platf_world_pose[0] = x_platform + b_ * cos(yaw);
    platf_world_pose[1] = y_platform + b_ * sin(yaw);
    platf_world_pose[2] = z_platform;                  

    Eigen::Matrix3d pose_platf_world;
    pose_platf_world <<       0.0,           -platf_world_pose[2], platf_world_pose[1],
                        platf_world_pose[2],          0.0,        -platf_world_pose[0],
                       -platf_world_pose[1],  platf_world_pose[0],      0.0;

    Matrix6d T_platf_world;
/*    T_platf_world << R_platform_world,
                     pose_platf_world * R_platform_world,
                     Eigen::Matrix3d::Zero(3,3),
                     R_platform_world;
*/

    T_platf_world << R_platform_world,
                     Eigen::Matrix3d::Zero(3,3),
                     Eigen::Matrix3d::Zero(3,3),
                     R_platform_world;

//--------------------------------------------------------------------------------------//

    Eigen::Matrix3d R_world_platform;
    R_world_platform = R_platform_world.inverse();

    Vector3d world_platf_pose;
    world_platf_pose = R_world_platform * platf_world_pose;

    Eigen::Matrix3d pose_world_platf;
    pose_world_platf <<       0.0,           -world_platf_pose[2], world_platf_pose[1],
                        world_platf_pose[2],          0.0,        -world_platf_pose[0],
                       -world_platf_pose[1],  world_platf_pose[0],      0.0;

    Matrix6d T_world_platf;
/*    T_world_platf << R_world_platform,
                     pose_world_platf * R_world_platform,
                     Eigen::Matrix3d::Zero(3,3),
                     R_world_platform;
*/  
    T_world_platf << R_world_platform,
                     Eigen::Matrix3d::Zero(3,3),
                     Eigen::Matrix3d::Zero(3,3),
                     R_world_platform;
  

    Matrix6d Transform;
    //Transform = T_world_platf * T_platf_world;
    Transform = T_world_platf;
    return Transform;

}



Vector6d Controller_mobile::limit_cartesian_dynamics (Vector6d joint_velocity) {

    double max_vel = 1.0;
    double max_acc = 0.7;

    // Limit Joint Velocity

    for (int i = 0; i < joint_velocity.size(); i++) {
        if (fabs(joint_velocity[i]) > max_vel) {       
          joint_velocity[i] = sign(joint_velocity[i]) * max_vel;
        }
    }

    // Limit Joint Acceleration

    for (int i = 0; i < joint_velocity.size(); i++) {
        if (fabs(joint_velocity[i] - adm_twist_last_cycle[i]) > max_acc * cycle_time) {
            joint_velocity[i] = adm_twist_last_cycle[i] + sign(joint_velocity[i] - adm_twist_last_cycle[i]) * max_acc * cycle_time;
        }
    }

    adm_twist_last_cycle = joint_velocity;
    return joint_velocity;

}



Vector2d Controller_mobile::limit_wheels_dynamics(Vector2d wheel_velocity)
{
  double max_vel = 1.0;
  double max_ang = 1.5;
  Vector2d cartesian_speed;

  cartesian_speed[0] = R / 2 * (wheel_velocity[0] + wheel_velocity[1]);
  cartesian_speed[1] = R / L * (wheel_velocity[0] - wheel_velocity[1]);


  if (fabs(cartesian_speed[0]) > max_vel) {

              cartesian_speed[0] = sign(cartesian_speed[0]) * max_vel;

          }

  if (fabs(cartesian_speed[1]) > max_ang) {

              cartesian_speed[1] = sign(cartesian_speed[1]) * max_ang;

          }

  return cartesian_speed;
}



int Controller_mobile::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}



void Controller_mobile::Spinner()
{    
/*
    Matrix6d T = Transformata();
    ROS_INFO_STREAM("Transformata: \n" << T << "\n");
    ROS_INFO_STREAM("x platform: " << x_platform << "\n");
    ROS_INFO_STREAM("y platform: " << y_platform << "\n");
    ROS_INFO_STREAM("z platform: " << z_platform << "\n");
*/
    Eigen::MatrixXd jacobian_base_mapped;
    //jacobian_base_mapped = Transformata() * compute_platform_jacobian(yaw);
    jacobian_base_mapped = compute_platform_jacobian(yaw);
    ROS_INFO_STREAM("Jacobian della Piattaforma: \n" << jacobian_base_mapped << "\n");

    Matrix26d J_base_pinv;
    J_base_pinv = jacobian_base_mapped.completeOrthogonalDecomposition().pseudoInverse();
    ROS_INFO_STREAM("Jacobian Aumentato PSEUDO-INVERSO: \n" << J_base_pinv << "\n");
    //ROS_INFO_STREAM("MASS: \n" << M_tot << "\nDAMPING: \n" << D_tot << "\n");
    ROS_INFO_STREAM("MASS: \n" << M_tot_prova << "\nDAMPING: \n" << D_tot_prova << "\n");
    ROS_INFO_STREAM("Forza applicata: \n" << F_ext << "\n");


    // **************** ADMITTANCE CONTROLLER **************** //
    Eigen::MatrixXd adm_acceleration(6,1);
    Eigen::MatrixXd adm_twist(6,1);
    Eigen::MatrixXd twist(6,1);
    Eigen::MatrixXd q_dot_base(2,1);

/*  //quando le variabili sono v e w
    q_dot_base(0,0) = (2*vel_x + omega_z * L)/(2*R);
    q_dot_base(1,0) = (2*vel_x - omega_z * L)/(2*R);
*/
    
    //quando le variabili sono w_l, w_r
    q_dot_base(0,0) = omega_l;
    q_dot_base(1,0) = omega_r;

    ROS_INFO_STREAM("Velocità dei giunti: \n" << q_dot_base << "\n");
    
    twist = jacobian_base_mapped * q_dot_base;
    ROS_INFO_STREAM("Velocità cartesiana: \n" << twist << "\n");
    
    Matrix6d M_prova = (jacobian_base_mapped * M_tot_prova.inverse() * jacobian_base_mapped.transpose()).completeOrthogonalDecomposition().pseudoInverse();
    Matrix6d D_prova = (jacobian_base_mapped * D_tot_prova.inverse() * jacobian_base_mapped.transpose()).completeOrthogonalDecomposition().pseudoInverse();

    ROS_INFO_STREAM("MASSA DI PROVA 2X2: \n" << M_prova << "\n" );
    ROS_INFO_STREAM("DAMPING DI PROVA 2X2: \n" << M_prova << "\n" );

    //adm_acceleration = M_tot.inverse() * ( - D_tot * twist + F_ext );
    adm_acceleration = M_prova.completeOrthogonalDecomposition().pseudoInverse() * ( - D_prova * twist + F_ext );


    ROS_INFO_STREAM("Admissible acceleration: \n" << adm_acceleration << "\n");
    ROS_INFO_STREAM("Twist Arm: \n" << twist << "\n");

    adm_twist = twist + adm_acceleration * cycle_time;

/*
    for(int i = 0; i < adm_twist.rows(); i++){
      if(F_ext(i,0) == 0.0){
        adm_twist(i,0) = 0.0;
      }
    }
*/
    //adm_twist = limit_cartesian_dynamics(adm_twist);

    ROS_INFO_STREAM("ADMISSIBLE TWIST: \n" << adm_twist << "\n");

    Eigen::MatrixXd adm_qdot_base(2,1);
    Eigen::MatrixXd delta_q(6,1);
    Eigen::MatrixXd adm_twist_base(6,1);

    //le variabili delle 2 ruote
    adm_qdot_base = J_base_pinv * adm_twist;
    
    ROS_INFO_STREAM("ADMISSIBLE Q DOT: \n" << adm_qdot_base << "\n");
    

//###################### BASE ######################################

    //lineare = adm_twist_base(0,0) * cos(yaw) + adm_twist_base(1,0) * sin(yaw);
    //angolare = -sin(yaw) * adm_twist_base(0,0) / b_ + cos(yaw) * adm_twist_base(1,0) / b_;

    // per rispettare i vincoli di velocità massima dei giunti 
    Vector2d vel_cartesiane = limit_wheels_dynamics(adm_qdot_base);

    base_message.linear.x = vel_cartesiane[0];
    base_message.angular.z = vel_cartesiane[1];

    ROS_INFO_STREAM("Lineare: \n" << vel_cartesiane[0] << "\n");
    ROS_INFO_STREAM("Angolare: \n" << vel_cartesiane[1] << "\n");
    ROS_INFO_STREAM("Omega sinistro: \n" << adm_qdot_base(0,0) << "\n");
    ROS_INFO_STREAM("Omega destro: \n" << adm_qdot_base(1,0) << "\n");

    mobile_pub.publish(base_message);

}

