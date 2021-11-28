#include "ammettenza/controller_arm.h"

Controller_arm::Controller_arm()
{
  joints_state_sub= nh.subscribe("/joint_states", 1, &Controller_arm::JointStateCallback, this);
  joystick_sub = nh.subscribe("/joy", 1, &Controller_arm::joystickCallback, this);
  arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur10e/manipulator_joint_trajectory_controller/command",1);

  cycle_time = 0.1;

  //inizializzo la matrice forza
  F_ext.conservativeResize(6,1);
  F_ext << Eigen::MatrixXd::Zero(6,1);

  //inizializzo i vettori posizione e volocità dei giunti
  for(int i = 0; i < 6; i++){
    joint_real_position[i] = 0.0;
    joint_real_velocity[i] = 0.0;
  }
 
  //acquisisco le matrici massa e damping
  std::vector<double> M_tot_parameters;
  std::vector<double> D_tot_parameters;
/*
  if (!nh.getParam("mass_arm", M_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the robot.");
  }

  if (!nh.getParam("damping_arm", D_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
  }

  M_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_tot_parameters.data());
  D_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_tot_parameters.data());
*/

  if (!nh.getParam("mass_arm_prova", M_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the robot.");
  }

  if (!nh.getParam("damping_arm_prova", D_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
  }

  M_tot_prova = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_tot_parameters.data());
  D_tot_prova = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_tot_parameters.data());

  //* Create the messages and publishing them
  arm_message.points.resize(1);
  arm_message.joint_names.resize(6);  
  arm_message.joint_names[0] ="ur10e_shoulder_pan_joint";
  arm_message.joint_names[1] ="ur10e_shoulder_lift_joint";
  arm_message.joint_names[2] ="ur10e_elbow_joint";
  arm_message.joint_names[3] ="ur10e_wrist_1_joint";
  arm_message.joint_names[4] ="ur10e_wrist_2_joint";
  arm_message.joint_names[5] ="ur10e_wrist_3_joint";

  arm_message.points[0].time_from_start = ros::Duration(cycle_time);


  // ---- MoveIt Robot Model ---- //
  robot_model_loader = robot_model_loader::RobotModelLoader ("robot_description");
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  joint_names = joint_model_group->getJointModelNames();

}



void Controller_arm::joystickCallback(const sensor_msgs::Joy& msg){

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



void Controller_arm::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){

  joint_state = *msg;

  // Ur10e Real Robot has Inverted Joints
  std::swap(joint_state.name[2], joint_state.name[4]);
  std::swap(joint_state.effort[2], joint_state.effort[4]);
  std::swap(joint_state.position[2], joint_state.position[4]);
  std::swap(joint_state.velocity[2], joint_state.velocity[4]);

  for (unsigned int i = 2; i < joint_state.position.size(); i++) {joint_real_position[i-2] = joint_state.position[i];}
  for (unsigned int i = 2; i < joint_state.velocity.size(); i++) {joint_real_velocity[i-2] = joint_state.velocity[i];}

}



Eigen::Matrix4d Controller_arm::compute_arm_fk(double joint_position[], double joint_velocity[]) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "world"
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ur10e_tool0");

    // Get the Translation Vector and Rotation Matrix
    // nel modello in simulazione calcola la distanza dell'end-effector da mobile_base_footprint
    // non fino alla base del manipolatore ur10e_base_link; Quindi tiene conto anche del-
    // l'altezza lungo z pari a 0.354
    Eigen::Vector3d translation_vector = end_effector_state.translation();
    Eigen::Matrix3d rotation_matrix = end_effector_state.rotation();

    //Transformation Matrix
    Eigen::Matrix4d transformation_matrix;
    transformation_matrix.setZero();

    //Set Identity to make bottom row of Matrix 0,0,0,1
    transformation_matrix.setIdentity();

    transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    transformation_matrix.block<3,1>(0,3) = translation_vector;

    return transformation_matrix;

}



Eigen::MatrixXd Controller_arm::compute_arm_jacobian (double joint_position[], double joint_velocity[]) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the Jacobian of the arm
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian;

    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian);

    return jacobian;

}




Vector6d Controller_arm::limit_joint_dynamics (Vector6d joint_velocity) {

    double max_vel = 1.0;
    double max_acc = 0.8;

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



int Controller_arm::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}


Matrix6d Controller_arm::Mapping(){
  
  Eigen::Matrix3d Rot_1;
  Eigen::Matrix3d Rot_2;
  Eigen::Matrix3d Rot_3;

  Matrix6d Trasformazione_1;
  Matrix6d Trasformazione_2;
  Matrix6d Trasformazione_3;
  Matrix6d Trasformazione;

  double angle=-45*M_PI/180;

//-------------------------------------------------------

  Rot_1 = Eigen::Matrix3d::Identity(3,3);

  Eigen::Vector3d ee_platform_pose;
  ee_platform_pose[0] = 0.0;
  ee_platform_pose[1] = 0.0;
  ee_platform_pose[2] = 0.354;


  Eigen::Matrix3d pos_ee_platform;

  pos_ee_platform <<          0.0,         -ee_platform_pose[2],  ee_platform_pose[1],
                      ee_platform_pose[2],            0.0,       -ee_platform_pose[0],
                     -ee_platform_pose[1],  ee_platform_pose[0],     0.0;

  ROS_INFO_STREAM("Matrice Antissimetrica: \n" << pos_ee_platform << "\n");


  Trasformazione_1 <<  Rot_1,
                       pos_ee_platform * Rot_1,
                       Eigen::Matrix3d::Zero(3,3),
                       Rot_1;

//-------------------------------------------------------

  Rot_2 = Eigen::Matrix3d::Identity(3,3);

  Eigen::Vector3d platform_b_pose;
  platform_b_pose[0] = -0.8;
  platform_b_pose[1] = 0.0;
  platform_b_pose[2] = 0.0;

  Eigen::Matrix3d pos_platform_b;

  pos_platform_b <<          0.0,         -platform_b_pose[2],  platform_b_pose[1],
                      platform_b_pose[2],            0.0,       -platform_b_pose[0],
                     -platform_b_pose[1],  platform_b_pose[0],     0.0;

  Trasformazione_2 <<  Rot_2,
                       pos_platform_b * Rot_2,
                       Eigen::Matrix3d::Zero(3,3),
                       Rot_2;

//-------------------------------------------------------

  Rot_3 << cos(angle), -sin(angle), 0.0,
           sin(angle),  cos(angle), 0.0,
              0.0,        0.0,      1.0;

  Eigen::Vector3d b_world_pose;
  b_world_pose[0] = 1.0 + 0.8 * cos(angle);
  b_world_pose[1] = -0.2 + 0.8 * sin(angle);
  b_world_pose[2] = 0.0;

  Eigen::Matrix3d pos_b_world;

  pos_b_world <<          0.0,         -b_world_pose[2],  b_world_pose[1],
                      b_world_pose[2],        0.0,        -b_world_pose[0],
                     -b_world_pose[1],  b_world_pose[0],         0.0;


  Trasformazione_3 <<  Rot_3,
                       pos_b_world * Rot_3,
                       Eigen::Matrix3d::Zero(3,3),
                       Rot_3;


/*
  Trasformazione_3 <<  Rot_3,
                       Eigen::Matrix3d::Zero(3,3),
                       Eigen::Matrix3d::Zero(3,3),
                       Rot_3;
*/

  Trasformazione = Trasformazione_3 * Trasformazione_2 * Trasformazione_1;

  
  return Trasformazione;
}


void Controller_arm::Spinner()
{  
    // **************** JACOBIANO AUMENTATO **************** //
    Eigen::Matrix4d Trasformation = compute_arm_fk(joint_real_position, joint_real_velocity);
    ROS_INFO_STREAM("Matrice di trasformazione: \n" << Trasformation);

    Matrix6d jacobian_augmented;
    //jacobian_augmented = Mapping() * compute_arm_jacobian(joint_real_position, joint_real_velocity);
    jacobian_augmented = compute_arm_jacobian(joint_real_position, joint_real_velocity);
    //jacobian_augmented << Transform_ee_platform * jacobian_arm_kdl;
    ROS_INFO_STREAM("Jacobian Aumentato: \n" << jacobian_augmented << "\n");

    // **************** JACOBIANO PSEUDO-INVERSO **************** //
    Eigen::MatrixXd J_pinv;

    //J_pinv = jacobian_augmented.completeOrthogonalDecomposition().pseudoInverse();
    J_pinv = jacobian_augmented.completeOrthogonalDecomposition().pseudoInverse();

    ROS_INFO_STREAM("Jacobian Aumentato PSEUDO-INVERSO: \n" << J_pinv << "\n");
    //ROS_INFO_STREAM("MASS: \n" << M_tot << "\nDAMPING: \n" << D_tot << "\n");
    ROS_INFO_STREAM("MASS: \n" << M_tot_prova << "\nDAMPING: \n" << D_tot_prova << "\n");
    ROS_INFO_STREAM("Forza applicata: \n" << F_ext << "\n");

    // **************** ADMITTANCE CONTROLLER **************** //
    Eigen::MatrixXd adm_acceleration(6,1);
    Eigen::MatrixXd adm_twist(6,1);
    Eigen::MatrixXd twist(6,1);
    Eigen::MatrixXd q_dot_arm(6,1);
    Eigen::MatrixXd q_dot(6,1);

    for(int i = 0; i < q_dot_arm.rows(); i++){
      q_dot_arm(i,0) = joint_real_velocity[i];
    }

    ROS_INFO_STREAM("Velocità dei giunti del braccio: \n" << q_dot_arm << "\n");

    q_dot << q_dot_arm;
    twist = jacobian_augmented * q_dot;
    
    
    Matrix6d M_prova = (jacobian_augmented * M_tot_prova.inverse() * jacobian_augmented.transpose()).completeOrthogonalDecomposition().pseudoInverse();
    Matrix6d D_prova = (jacobian_augmented * D_tot_prova.inverse() * jacobian_augmented.transpose()).completeOrthogonalDecomposition().pseudoInverse();

    ROS_INFO_STREAM("MASSA DI PROVA 2X2: \n" << M_prova << "\n" );
    ROS_INFO_STREAM("DAMPING DI PROVA 2X2: \n" << M_prova << "\n" );

    //adm_acceleration = M_tot.inverse() * ( - D_tot * twist + F_ext );
    adm_acceleration = M_prova.completeOrthogonalDecomposition().pseudoInverse() * ( - D_prova * twist + F_ext );

    //adm_acceleration = M_tot.inverse() * ( - D_tot * twist + F_ext);


    ROS_INFO_STREAM("Admissible acceleration: \n" << adm_acceleration << "\n");
    ROS_INFO_STREAM("Twist del Robot: \n" << twist << "\n");

    adm_twist = twist + adm_acceleration * cycle_time;

/*
    for(int i = 0; i < adm_twist.rows(); i++){
      if(F_ext(i,0) == 0.0){
        adm_twist(i,0) = 0.0;
      }
    }
*/


    ROS_INFO_STREAM("ADMISSIBLE TWIST: \n" << adm_twist << "\n");

    Eigen::MatrixXd adm_qdot(6,1);
    Eigen::MatrixXd delta_q(6,1);
    Eigen::MatrixXd adm_twist_base(6,1);

    adm_qdot = J_pinv * adm_twist;

    adm_qdot = limit_joint_dynamics(adm_qdot);
    ROS_INFO_STREAM("ADMISSIBLE Q DOT: \n" << adm_qdot << "\n");

//###################### ARM ######################################
    
    delta_q = adm_qdot * cycle_time;
    ROS_INFO_STREAM("DELTA Q: \n" << delta_q << "\n");

    Eigen::MatrixXd q_arm(6,1); 

    for(int i = 0; i < q_arm.rows(); i++){

      q_arm(i,0) = joint_real_position[i];
    }

    ROS_INFO_STREAM("Real Joint Position: \n" << q_arm << "\n");

    q_arm += delta_q;

    ROS_INFO_STREAM("Q ARM: \n" << q_arm << "\n");

   
    arm_message.points[0].positions = {q_arm(0,0),
                                             q_arm(1,0),
                                             q_arm(2,0),
                                             q_arm(3,0),
                                             q_arm(4,0),
                                             q_arm(5,0)};
  

    arm_pub.publish(arm_message);

}

