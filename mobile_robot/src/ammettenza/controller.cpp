
// Created: 2021/03/17 12:24:41
// Last modified: 2021/04/10 00:07:00


#include "ammettenza/controller.h"

Controller::Controller()
{
  odometry_sub = nh.subscribe("/base_pose_ground_truth", 1, &Controller::OdometryCallback, this);
  joints_state_sub= nh.subscribe("/joint_states", 1, &Controller::JointStateCallback, this);
  joystick_sub = nh.subscribe("/joy", 1, &Controller::joystickCallback, this);
  arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur10e/manipulator_joint_trajectory_controller/command",1);
  mobile_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  R = 0.0625;
  L = 0.445208;
  cycle_time = 0.1;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  vel_x = 0.0;
  omega_z = 0.0;
  omega_l = 0.0;
  omega_r = 0.0;
  //IO-SFL lead value
  b_ = -0.7;

  F_ext.setZero();

  for(int i = 0; i < 6; i++){
    joint_real_position[i] = 0.0;
    joint_real_velocity[i] = 0.0;
  }
 
  twist.setZero();

  std::vector<double> M_tot_parameters;
  std::vector<double> D_tot_parameters;
  std::vector<double> max_acc;
  std::vector<double> max_vel;


  if (!nh.getParam("mass_robot", M_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the robot.");
  }

  if (!nh.getParam("damping_robot", D_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
  }

  if (!nh.getParam("acc_limits", max_acc)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
  }

  if (!nh.getParam("vel_limits", max_vel)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
  }

  M_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_tot_parameters.data());
  D_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_tot_parameters.data());
  acc_limits = Eigen::Map<Eigen::Matrix<double, 8, 1> >(max_acc.data());
  vel_limits = Eigen::Map<Eigen::Matrix<double, 8, 1> >(max_vel.data());


  std::stringstream file_path;
  file_path << "/home/cristian/catkin_ws/src/mobile_robot/debug/q_dot.txt";
  q_dot_file.open(file_path.str());

  std::stringstream file_path1;
  file_path1 << "/home/cristian/catkin_ws/src/mobile_robot/debug/q_dot_SNS.txt";
  q_dot_SNS_file.open(file_path1.str());

  std::stringstream file_path2;
  file_path2 << "/home/cristian/catkin_ws/src/mobile_robot/debug/force_debug.txt";
  force_file.open(file_path2.str());
  
//  Vel_LowerLimit = - acc_limits * cycle_time;
//  Vel_UpperLimit = + acc_limits * cycle_time;

/*
  if (!nh.getParam("mass_robot_prova", M_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired mass of the robot.");
  }

  if (!nh.getParam("damping_robot_prova", D_tot_parameters)) {
    ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
  }

  M_tot_prova = Eigen::Map<Eigen::Matrix<double, 8, 8> >(M_tot_parameters.data());
  D_tot_prova = Eigen::Map<Eigen::Matrix<double, 8, 8> >(D_tot_parameters.data());
*/

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


Controller::~Controller() {

    q_dot_file.close();
    q_dot_SNS_file.close();

}


void Controller::OdometryCallback(const nav_msgs::Odometry& msg){
  
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

  //vel_x = msg.twist.twist.linear.x;
  vel_x = pow( pow(msg.twist.twist.linear.x, 2) + pow(msg.twist.twist.linear.y, 2) , 2);
  omega_z = msg.twist.twist.angular.z;
  
  x_platform = msg.pose.pose.position.x;
  y_platform = msg.pose.pose.position.y;
  z_platform = 0.0;

}



void Controller::joystickCallback(const sensor_msgs::Joy& msg){
/*
  Guida:  msg.axes[0]; //levetta sx orizzontale --> Fx
          msg.axes[1]; //levetta sx verticale --> Fy
          msg.axes[4]; //levetta dx verticale --> Fz
          msg.axes[3]; //levetta dx orizzontale --> Mx
          msg.axes[6]; //analogico orizzontale --> My
          msg.axes[7]; //analogico verticale --> Mz
 */


  double intensity = 30;

  if(abs(msg.axes[0]) > 0.0)
    F_ext(0) = msg.axes[0] * intensity; //levetta sx orizzontale --> Fx
  else 
    F_ext(0) = 0.0;

  if(abs(msg.axes[1]) > 0.0)
    F_ext(1) = msg.axes[1] * intensity; //levetta sx verticale --> Fy
  else
    F_ext(1) = 0.0;

  if(abs(msg.axes[4]) > 0.0)
    F_ext(2) = msg.axes[4] * intensity; //levetta dx verticale --> Fz
  else 
    F_ext(2) = 0.0; 
  
  if(abs(msg.axes[3]) > 0.0)
    F_ext(3) = msg.axes[3] * intensity; //levetta dx orizzontale --> Mx
  else 
    F_ext(3) = 0.0; 

  if(abs(msg.axes[6]) > 0.0)
    F_ext(4) = msg.axes[6] * intensity; //analogico orizzontale --> My
  else 
    F_ext(4) = 0.0; 

  if(abs(msg.axes[7]) > 0.0)
    F_ext(5) = msg.axes[7] * intensity; //analogico verticale --> Mz
  else
    F_ext(5) = 0.0;


/*
  F_ext(0) = 0.0;
  F_ext(1) = -intensity;
  F_ext(2) = 0.0;
  F_ext(3) = 0.0;
  F_ext(4) = 0.0;
  F_ext(5) = 0.0;
*/
//ROS_INFO_STREAM("Forza applicata: \n" << F_ext << "\n");
}



void Controller::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){

  joint_state = *msg;

  // Ur10e Real Robot has Inverted Joints
  std::swap(joint_state.name[2], joint_state.name[4]);
  std::swap(joint_state.effort[2], joint_state.effort[4]);
  std::swap(joint_state.position[2], joint_state.position[4]);
  std::swap(joint_state.velocity[2], joint_state.velocity[4]);

  for (unsigned int i = 2; i < joint_state.position.size(); i++) {joint_real_position[i-2] = joint_state.position[i];}
  for (unsigned int i = 2; i < joint_state.velocity.size(); i++) {joint_real_velocity[i-2] = joint_state.velocity[i];}

  omega_l = joint_state.velocity[0];
  omega_r = joint_state.velocity[1];

}


Matrix4d Controller::compute_arm_fk(double joint_position[], double joint_velocity[]) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "world"
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("ur10e_tool0");

    // Get the Translation Vector and Rotation Matrix
    Eigen::Vector3d translation_vector = end_effector_state.translation();
    Eigen::Matrix3d rotation_matrix = end_effector_state.rotation();

    ROS_INFO_STREAM("Distanza dell'end-effector da footprint: \n" << translation_vector);
    //Transformation Matrix
    Matrix4d transformation_matrix;
    transformation_matrix.setZero();

    //Set Identity to make bottom row of Matrix 0,0,0,1
    transformation_matrix.setIdentity();

    transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    transformation_matrix.block<3,1>(0,3) = translation_vector;

    return transformation_matrix;

}



Eigen::MatrixXd Controller::compute_arm_jacobian (double joint_position[], double joint_velocity[]) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the Jacobian of the arm
    Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
    Eigen::MatrixXd jacobian_arm;

    kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), reference_point_position, jacobian_arm);

    return jacobian_arm;

}



Matrix62d Controller::compute_platform_jacobian(double rotation)
{
  
  Matrix62d jacobian_platform;

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

/*
  double j11 = R*cos(rotation)/2 - b_*sin(rotation)*R/L;
  double j12 = R*cos(rotation)/2 + b_*sin(rotation)*R/L;
  double j21 = R*sin(rotation)/2 + b_*cos(rotation)*R/L;
  double j22 = R*sin(rotation)/2 - b_*cos(rotation)*R/L;

  double k = 100.0;
*/

  double b11, b12, b21, b22;
  b11 = cos(yaw);
  b12 = - b_ * sin(yaw);
  b21 = sin(yaw);
  b22 = b_ * cos(yaw);

  jacobian_platform << b11, b12,
                       b21, b22,
                       0.0, 0.0, 
                       0.0, 0.0, 
                       0.0, 0.0, 
                       0.0, 0.0;


    return jacobian_platform;

}



Matrix6d Controller::Arm_Mapping(double rotation){


    Matrix6d Transformation_ArmBase_Platform;
    Matrix6d Transformation_Platform_b;
    Matrix6d Transformation_b_World;
    Matrix6d Transformation;

    //double angle=-45*M_PI/180.0;

  //----------------------------- Arm Base frame --> Platform frame  ----------------------------------//

    Rot_ArmBase_Platform = Eigen::Matrix3d::Identity(3,3);

    Transformation_ArmBase_Platform <<  Rot_ArmBase_Platform,
                                        Eigen::Matrix3d::Zero(3,3),
                                        Eigen::Matrix3d::Zero(3,3),
                                        Rot_ArmBase_Platform;

  //--------------------------- Platform Frame --> I-O SFL Frame (b_) -------------------------------//

    Rot_Platform_b = Eigen::Matrix3d::Identity(3,3);

    Transformation_Platform_b <<  Rot_Platform_b,
                                  Eigen::Matrix3d::Zero(3,3),
                                  Eigen::Matrix3d::Zero(3,3),
                                  Rot_Platform_b;

  //------------------------------- I-O SFL Frame --> World Frame ----------------------------------//

    Rot_b_World << cos(rotation), -sin(rotation), 0.0,
                   sin(rotation),  cos(rotation), 0.0,
                        0.0,            0.0,      1.0;

    Transformation_b_World << Rot_b_World,
                              Eigen::Matrix3d::Zero(3,3),
                              Eigen::Matrix3d::Zero(3,3),
                              Rot_b_World;


  //---------------------------------------------------------------------------------------------------//

    // Cosi riporto la velocità dell'end-effector dal Frame della base del manipolatore al frame World
    // quindi dovrei avere velocità di EE rspetto a World espressa nel frame World
    Transformation = Transformation_b_World * Transformation_Platform_b * Transformation_ArmBase_Platform;
    //Transformation = Transformation_Platform_b * Transformation_ArmBase_Platform;
  //---------------------------------------------------------------------------------------------------//

    return Transformation;

}




Matrix6d Controller::Mir_Mapping(double rotation){
  

    Matrix6d H_platform;

    Vector3d ee_ArmBase_pose = compute_arm_fk(joint_real_position, joint_real_velocity).block<3,1>(0,3);
    ee_ArmBase_pose(0) -= 0.316;
    ee_ArmBase_pose(1) -= 0.085;
    ee_ArmBase_pose(2) -= 0.354;

    Vector3d platform_b_pose;
    platform_b_pose(0) = b_;
    platform_b_pose(1) = 0.0;
    platform_b_pose(2) = 0.0;
    
    Vector3d ArmBase_platform_pose;
    ArmBase_platform_pose(0) = 0.316;
    ArmBase_platform_pose(1) = 0.085;
    ArmBase_platform_pose(2) = 0.354;
    
    Vector3d W_r_b_E;
    //nel frame di WORLD
    W_r_b_E = Rot_b_World * (ArmBase_platform_pose + Rot_Platform_b * Rot_ArmBase_Platform * ee_ArmBase_pose - platform_b_pose);
    
    //nel frame del MIR
    //W_r_b_E = ArmBase_platform_pose + Rot_Platform_b * Rot_ArmBase_Platform * ee_ArmBase_pose - platform_b_pose;


    //W_r_B_E = ArmBase_platform_pose + Rot_Platform_b * Rot_ArmBase_Platform * ee_ArmBase_pose - platform_b_pose;
    W_r_B_E = ArmBase_platform_pose + Rot_Platform_b * Rot_ArmBase_Platform * ee_ArmBase_pose;


    Eigen::Matrix3d W_r_b_E_skewsimmetric;
    W_r_b_E_skewsimmetric <<        0.0,      -W_r_b_E(2),   W_r_b_E(1),
                                W_r_b_E(2),       0.0,      -W_r_b_E(0),
                               -W_r_b_E(1),    W_r_b_E(0),       0.0;


  
    H_platform << Eigen::Matrix3d::Identity(3,3),
                  W_r_b_E_skewsimmetric,
                  Eigen::Matrix3d::Zero(3,3),
                  Eigen::Matrix3d::Identity(3,3);


/*    H_platform << 1.0, 0.0, 0.0, 0.0, 0.0, -W_r_b_E(1),
                  0.0, 1.0, 0.0, 0.0, 0.0, W_r_b_E(0),
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
*/
    // nel modello unico in simulazione, movit misura la distanza dell'end-effector
    // dal frame della piattaforma, quidni considera già l'altezza in z pari a 0.354

    return H_platform;
}



Matrix6d Controller::Transformata(double rotation, int num){


    Eigen::Matrix3d R_platform_world;
    R_platform_world << cos(rotation), -sin(rotation), 0,
                        sin(rotation),  cos(rotation), 0,
                              0,               0,      1;

    Vector3d platf_world_pose;
    platf_world_pose(0) = x_platform + b_ * cos(yaw);
    platf_world_pose(1) = y_platform + b_ * sin(yaw);
    platf_world_pose(2) = z_platform;                  

    Eigen::Matrix3d pose_platf_world;
    pose_platf_world <<       0.0,           -platf_world_pose(2), platf_world_pose(1),
                        platf_world_pose(2),          0.0,        -platf_world_pose(0),
                       -platf_world_pose(1),  platf_world_pose(0),      0.0;

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
    pose_world_platf <<       0.0,           -world_platf_pose(2), world_platf_pose(1),
                        world_platf_pose(2),          0.0,        -world_platf_pose(0),
                       -world_platf_pose(1),  world_platf_pose(0),      0.0;

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
    
    switch (num)
    {
    case 1:
      Transform = T_platf_world;
      break;
    case 2:
      Transform = T_world_platf;
      break;
    }

    return Transform;

}




Vector8d Controller::limit_joints_dynamics (Vector8d joint_velocity) {

    // Limit Joint Velocity

    ROS_INFO_STREAM("joint velocity last cycle:\n" << adm_twist_last_cycle << "\n" );
    ROS_INFO_STREAM("joint velocity in ingresso:\n" << joint_velocity << "\n");
    ROS_INFO_STREAM("joint velocity in ingresso min Coeff:\n" << joint_velocity.minCoeff() << "\n");
    ROS_INFO_STREAM("joint velocity in ingresso max Coeff:\n" << joint_velocity.maxCoeff() << "\n");

    Vector8d scaling;
 

    for (int i = 0; i < joint_velocity.rows(); i++)
    {
      if(abs(joint_velocity(i)) > vel_limits(i))
        scaling(i) = vel_limits(i) / abs(joint_velocity(i));
      else 
        scaling(i) = 1;
    }

    
    ROS_INFO_STREAM("scaling factors:\n" << scaling << "\n");

    double s_ = scaling.minCoeff();
    ROS_INFO_STREAM("scaling factor valore di riferimento:\n" << s_ << "\n");
    
    joint_velocity = s_ * joint_velocity;
    
    ROS_INFO_STREAM("joint velocity scalati:\n" << joint_velocity << "\n");


    for (int i = 0; i < joint_velocity.rows(); i++) {
        if (abs(joint_velocity(i)) > vel_limits(i)) {       
          joint_velocity(i) = sign(joint_velocity(i)) * vel_limits(i);
        }
    }

    ROS_INFO_STREAM("Joint dopo saturazione di velocità: \n" << joint_velocity << "\n");
    // Limit Joint Acceleration

    for (int i = 0; i < joint_velocity.rows(); i++) {
        if (abs(joint_velocity(i) - adm_twist_last_cycle(i)) > acc_limits(i) * cycle_time) {
            joint_velocity(i) = adm_twist_last_cycle(i) + sign(joint_velocity(i) - adm_twist_last_cycle(i)) * acc_limits(i) * cycle_time;
        }
    }


    ROS_INFO_STREAM("joint velocity in uscita:\n" << joint_velocity << "\n");

    adm_twist_last_cycle = joint_velocity;
    
    return joint_velocity;

}




int Controller::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}



void Controller::Spinner()
{
    
    //Compute_Transformation();
    //ROS_INFO_STREAM("LA TRASFORMAZIONE SPAZIALE: \n" << Transform_ee_platform << "\n");

    Matrix6d jacobian_manipulator;
    jacobian_manipulator = Arm_Mapping(yaw) * compute_arm_jacobian(joint_real_position, joint_real_velocity);
    //jacobian_manipulator = compute_arm_jacobian(joint_real_position, joint_real_velocity);
    ROS_INFO_STREAM("Jacobian del Braccio Complessivo: \n" << jacobian_manipulator << "\n");


    Matrix62d jacobian_base_mapped;
    //jacobian_base_mapped = ee_mapping_matrix * compute_platform_jacobian(yaw);
    //jacobian_base_mapped = Mir_Mapping(yaw) * compute_platform_jacobian(yaw);
    jacobian_base_mapped = compute_platform_jacobian(yaw);
    //jacobian_base_mapped = Mir_Mapping(yaw) * Transformata(yaw, 2) * compute_platform_jacobian(yaw);    
    //jacobian_base_mapped = jacobian_base;
    ROS_INFO_STREAM("Jacobian della Piattaforma Complessivo: \n" << jacobian_base_mapped << "\n");

    // **************** JACOBIANO AUMENTATO **************** //
    Eigen::MatrixXd jacobian_augmented(6,8);
    jacobian_augmented << jacobian_manipulator, jacobian_base_mapped;
    ROS_INFO_STREAM("Jacobian Aumentato: \n" << jacobian_augmented << "\n");

    // **************** JACOBIANO PSEUDO-INVERSO **************** //
    Eigen::MatrixXd J_pinv(8,6);
    J_pinv = jacobian_augmented.completeOrthogonalDecomposition().pseudoInverse();
/*    Eigen::MatrixXd W_(8,8);
    W_.setIdentity();
    W_.topLeftCorner(6,6) = 0.3 * Eigen::MatrixXd::Identity(6,6);

    J_pinv = W_.inverse() * jacobian_augmented.transpose() * (jacobian_augmented * W_.inverse() * jacobian_augmented.transpose()).inverse();
*/
    ROS_INFO_STREAM("Jacobian Aumentato PSEUDO-INVERSO: \n" << J_pinv << "\n");
    ROS_INFO_STREAM("MASS: \n" << M_tot << "\nDAMPING: \n" << D_tot << "\n");
    //ROS_INFO_STREAM("MASS: \n" << M_tot_prova << "\nDAMPING: \n" << D_tot_prova << "\n");    
    ROS_INFO_STREAM("Forza applicata: \n" << F_ext << "\n");
    
    // **************** ADMITTANCE CONTROLLER **************** //
    Vector6d adm_acceleration;
    Vector6d adm_twist;
    //Eigen::MatrixXd twist(6,1);
    Vector2d q_dot_base;
    Vector6d q_dot_arm;
    Vector8d q_dot;

    for(int i = 0; i < q_dot_arm.rows(); i++){
      q_dot_arm(i) = joint_real_velocity[i];
    }

    ROS_INFO_STREAM("Velocità dei giunti del braccio: \n" << q_dot_arm << "\n");

/*
    q_dot_base(0) = R/2 * (omega_r + omega_l);
    q_dot_base(1) = R/L * (omega_r - omega_l);
*/

    q_dot_base(0) = vel_x;
    q_dot_base(1) = omega_z;

    ROS_INFO_STREAM("Velocità dei giunti delle ruote: \n" << q_dot_base << "\n");

    q_dot << q_dot_arm, q_dot_base;

    twist = jacobian_augmented * q_dot;
/*
    Matrix6d M_prova = (jacobian_augmented * M_tot_prova.inverse() * jacobian_augmented.transpose()).completeOrthogonalDecomposition().pseudoInverse();
    Matrix6d D_prova = (jacobian_augmented * D_tot_prova.inverse() * jacobian_augmented.transpose()).completeOrthogonalDecomposition().pseudoInverse();

    ROS_INFO_STREAM("MASSA DI PROVA 8X8: \n" << M_prova << "\n" );
    ROS_INFO_STREAM("DAMPING DI PROVA 8X8: \n" << M_prova << "\n" );

    adm_acceleration = M_prova.completeOrthogonalDecomposition().pseudoInverse() * ( - D_prova * twist + F_ext );

*/
    adm_acceleration = M_tot.inverse() * ( - D_tot * twist + F_ext);

    ROS_INFO_STREAM("Admissible acceleration: \n" << adm_acceleration << "\n");
    ROS_INFO_STREAM("Twist del Robot: \n" << twist << "\n");

    adm_twist = twist + adm_acceleration * cycle_time;

    ROS_INFO_STREAM("ADMISSIBLE TWIST: \n" << adm_twist << "\n");

    Vector8d adm_qdot;
    Vector6d adm_qdot_arm;
    Vector2d adm_qdot_base;
    Vector6d delta_q;
    Vector6d adm_twist_base;
    Vector8d adm_q_dot_scaling;

    //twist = adm_twist;
    //adm_qdot = J_pinv * adm_twist;
    adm_q_dot_scaling = J_pinv * adm_twist;
    //ROS_INFO_STREAM("ADMISSIBLE Q DOT: \n" << adm_qdot << "\n");

//###########################################################################################
//############################ SNS ALGORITHM ################################################
//###########################################################################################

    Eigen::MatrixXd W(8,8);
    Eigen::MatrixXd W_(8,8);
    Vector8d q_dot_N;
    Vector8d q_dot_N_;
    Vector2d check_max_limits;
    Vector2d check_min_limits;
    double s = 1.0;
    double s_ = 0.0;
    Vector8d S_max;
    Vector8d S_min;
    double scaling_factor;
    bool limit_exceeded = false;

    ROS_INFO("inizializzato le variabili \n");
    
    W.setIdentity();
    q_dot_N.setZero();
    ROS_INFO("inizializzato W e q_dot_N \n");
    ROS_INFO_STREAM("W: \n" << W << "\n");
    ROS_INFO_STREAM("q_dot_N: \n" << q_dot_N << "\n");

    do{
      
      limit_exceeded = false;
      
      Eigen::MatrixXd prod_1(6,8);
      prod_1 = (jacobian_augmented * W).completeOrthogonalDecomposition().pseudoInverse();
     
      ROS_INFO_STREAM("q_dot_SNS_1: \n" << q_dot_SNS << "\n");
      ROS_INFO_STREAM("prod_1: \n" << prod_1 << "\n");

      q_dot_SNS = q_dot_N + prod_1 * (s * adm_twist - jacobian_augmented * q_dot_N);
      ROS_INFO_STREAM("q_dot_SNS_2: \n" << q_dot_SNS << "\n");
      
      Vector8d scaling_find_critical;
      //int most_critical;
      Eigen::MatrixXf::Index most_critical;

      for (int i = 0; i < q_dot_SNS.rows(); i++)
      {
        check_max_limits << Vel_UpperLimit(i), Acc_UpperLimit(i); 
        check_min_limits << Vel_LowerLimit(i), Acc_LowerLimit(i);
        Max_Limits(i) = check_max_limits.maxCoeff();
        Min_Limits(i) = check_min_limits.minCoeff();

        if (q_dot_SNS(i) > Max_Limits(i))
          scaling_find_critical(i) = Max_Limits(i) / q_dot_SNS(i);
        else if(q_dot_SNS(i) < Min_Limits(i))
          scaling_find_critical(i) = Min_Limits(i) / q_dot_SNS(i);
        else
          scaling_find_critical(i) = 1.0;
        
      }  

      ROS_INFO_STREAM("MAX LIMITS:\n" << Max_Limits << "\n");
      ROS_INFO_STREAM("MIN LIMITS:\n" << Min_Limits << "\n");
      ROS_INFO_STREAM("Q_DOT_PORCO_DIO:\n" << q_dot_SNS << "\n");
      ROS_INFO_STREAM("scaling factor find critical:\n" << scaling_find_critical << "\n");
      if (scaling_find_critical.minCoeff() != 1){
       
        /*
        for (int i = 0; i < scaling_find_critical.rows(); i++)
        {
          if(scaling_find_critical(i) == scaling_find_critical.minCoeff()){
              most_critical = i;
              break;
          }
        }
        */
      
        scaling_find_critical.minCoeff(&most_critical);
        ROS_INFO_STREAM("MOST CRITICAL:\t" << most_critical << "\n");
        limit_exceeded = true;
      }

          // Task scaling factor Algorithm
          if(limit_exceeded == true){
            
              Eigen::MatrixXd prod_2(6,8);
              prod_2 = (jacobian_augmented * W).completeOrthogonalDecomposition().pseudoInverse();
              ROS_INFO_STREAM("prod_2: \n" << prod_2 << "\n");
              
              Vector8d a = prod_2 * adm_twist;
              Vector8d b = q_dot_N - (prod_2 * jacobian_augmented )* q_dot_N;

              for (int j = 0; j < 8; j++)
              {
                  S_min(j) = (Min_Limits(j) - b(j)) / a(j);
                  S_max(j) = (Max_Limits(j) - b(j)) / a(j);

                  if (S_min(j) > S_max(j))
                  {
                      double swap_min = S_min(j);
                      double swap_max = S_max(j);
                      
                      S_min(j) = swap_max;
                      S_max(j) = swap_min;
                  }            
              }

              double s_max = S_max.minCoeff();
              double s_min = S_min.maxCoeff();
              

              if (s_min > s_max || s_max < 0.0 || s_min > 1.0)
              {
                  scaling_factor = 0.0;
              }
              else
              {
                  Vector2d appoggio;
                  appoggio << s_max, 1.0;
                  scaling_factor = appoggio.minCoeff();
              }

              if (scaling_factor > s_)
              {
                s_ = scaling_factor;
                W_ = W;
                q_dot_N_ = q_dot_N;
              }
              
              //W(most_critical, most_critical) = 0.0;
              ROS_INFO_STREAM("MOST CRITICAL:\t" << most_critical << "\n");
              
              W(most_critical,most_critical) = 0.0;
              
              if (sign(q_dot_N(most_critical)) == -1)
                q_dot_N(most_critical) = Min_Limits(most_critical);
              else if (sign(q_dot_N(most_critical)) == 1) 
                q_dot_N(most_critical) = Max_Limits(most_critical);

              Eigen::MatrixXd prod_3(6,8);
              prod_3 = jacobian_augmented * W;
              Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(prod_3);
              int rank = lu_decomp.rank();

              ROS_INFO_STREAM("rank:\t" << rank << "\n");
              if (rank < 6)
              {
                s = s_;
                W = W_;
                q_dot_N = q_dot_N_;
                Eigen::MatrixXd prod_4(6,8);
                prod_4 = (jacobian_augmented * W).completeOrthogonalDecomposition().pseudoInverse();
                q_dot_SNS = q_dot_N + prod_4 * (s * adm_twist - jacobian_augmented * q_dot_N);
                limit_exceeded = false;
              }            

          }   
      
    } while (limit_exceeded == true);
    
    
    q_dot_SNS_last_cycle = q_dot_SNS;

    Vel_LowerLimit = -vel_limits;
    Vel_UpperLimit = vel_limits;
    
    Acc_LowerLimit = q_dot_SNS_last_cycle - acc_limits * cycle_time;
    Acc_UpperLimit = q_dot_SNS_last_cycle + acc_limits * cycle_time;

//###########################################################################################
//###########################################################################################
//###########################################################################################

    adm_qdot = q_dot_SNS;

    adm_q_dot_scaling = limit_joints_dynamics(adm_q_dot_scaling);
  
  //############# SAVE DATA #################################################################
    //generate_csv(adm_qdot, "Adm_qdot");
    double time_now = ros::Time::now().toSec();
    q_dot_file << time_now;
    q_dot_SNS_file << time_now;
    force_file << time_now;

    for(int i = 0; i < 8; i++){
        q_dot_file << " " << adm_q_dot_scaling(i);
    }

    for(int i = 0; i < 8; i++){
        q_dot_SNS_file << " " << adm_qdot(i);
    }

    for(int i = 0; i < 6; i++){
        force_file << " " << F_ext(i);
    }

    q_dot_file << std::endl;
    q_dot_SNS_file << std::endl;
    force_file << std::endl;
    
  //########################################################################################

    ROS_INFO_STREAM("VELOCITA DEI GIUNTI: \n" << adm_qdot);
  
    //le variabili di giunto del manipolatore sono solo le prime 6
    for(int i = 0; i < adm_qdot_arm.rows(); i++){
      adm_qdot_arm(i) = adm_qdot(i);
    }    

    //mentre le variabili di giunto della base sono le ultime 2  
    for(int i = 6; i < adm_qdot.rows(); i++){
      adm_qdot_base(i-6) = adm_qdot(i);
    }


    ROS_INFO_STREAM("ADMISSIBLE Q DOT ARM: \n" << adm_qdot_arm << "\n");
    ROS_INFO_STREAM("ADMISSIBLE Q DOT BASE: \n" << adm_qdot_base << "\n");
    

//###################### ARM ######################################
    
    delta_q = adm_qdot_arm * cycle_time;
    ROS_INFO_STREAM("DELTA Q: \n" << delta_q << "\n");

    Vector6d q_arm; 

    for(int i = 0; i < q_arm.rows(); i++)
      q_arm(i) = joint_real_position[i];

    ROS_INFO_STREAM("Real Joint Position: \n" << q_arm << "\n");

    q_arm += delta_q;  

    ROS_INFO_STREAM("Q ARM: \n" << q_arm << "\n");


    arm_message.points[0].positions = {q_arm(0),
                                       q_arm(1),
                                       q_arm(2),
                                       q_arm(3),
                                       q_arm(4),
                                       q_arm(5)};
  

    arm_pub.publish(arm_message);

//###################### BASE ######################################

    base_message.linear.x = adm_qdot_base(0);
    base_message.angular.z = adm_qdot_base(1);
    
    ROS_INFO_STREAM("Lineare: \n" << adm_qdot_base(0) << "\n");
    ROS_INFO_STREAM("Angolare: \n" << adm_qdot_base(1) << "\n");
    ROS_INFO_STREAM("POSIZIONE EE --> b:\t" << W_r_B_E(0) << "\n");
    ROS_INFO_STREAM("Compilato:\t 06 \n");
    mobile_pub.publish(base_message);
/*      
    Matrix6d w_2;
    w_2 = jacobian_augmented * jacobian_augmented.transpose();
    double w = sqrt(w_2.determinant());
    ROS_INFO_STREAM("Manipulability:\t" << w);
*/
    
}
