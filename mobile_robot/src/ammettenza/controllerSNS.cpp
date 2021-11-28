
// Created: 2021/03/17 12:24:41
// Last modified: 2021/03/21 17:39:53


#include "ammettenza/controllerSNS.h"

ControllerSNS::ControllerSNS()
{
  odometry_sub = nh.subscribe("/base_pose_ground_truth", 1, &ControllerSNS::OdometryCallback, this);
  joints_state_sub= nh.subscribe("/joint_states", 1, &ControllerSNS::JointStateCallback, this);
  joystick_sub = nh.subscribe("/joy", 1, &ControllerSNS::joystickCallback, this);
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
  b_ = -1.3;

  F_ext.conservativeResize(6,1);
  F_ext << Eigen::MatrixXd::Zero(6,1);

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



void ControllerSNS::OdometryCallback(const nav_msgs::Odometry& msg){
  
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



void ControllerSNS::joystickCallback(const sensor_msgs::Joy& msg){
/*
  Guida:  msg.axes[0]; //levetta sx orizzontale --> Fx
          msg.axes[1]; //levetta sx verticale --> Fy
          msg.axes[4]; //levetta dx verticale --> Fz
          msg.axes[3]; //levetta dx orizzontale --> Mx
          msg.axes[6]; //analogico orizzontale --> My
          msg.axes[7]; //analogico verticale --> Mz
 */

  double intensity = 30;

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
  else
    F_ext(5,0) = 0.0;

//ROS_INFO_STREAM("Forza applicata: \n" << F_ext << "\n");
}



void ControllerSNS::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){

  joint_state = *msg;

  // Ur10e Real Robot has Inverted Joints
  std::swap(joint_state.name[2], joint_state.name[4]);
  std::swap(joint_state.effort[2], joint_state.effort[4]);
  std::swap(joint_state.position[2], joint_state.position[4]);
  std::swap(joint_state.velocity[2], joint_state.velocity[4]);

  for (unsigned int i = 2; i < joint_state.position.size(); i++) {joint_real_position[i-2] = joint_state.position[i];}
  for (unsigned int i = 2; i < joint_state.velocity.size(); i++) {joint_real_velocity[i-2] = joint_state.velocity[i];}

  omega_l = joint_state.position[0];
  omega_r = joint_state.position[1];

}


Matrix4d ControllerSNS::compute_arm_fk(double joint_position[], double joint_velocity[]) {

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



Eigen::MatrixXd ControllerSNS::compute_arm_jacobian (double joint_position[], double joint_velocity[]) {

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



Matrix62d ControllerSNS::compute_platform_jacobian(double rotation)
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
                       0.0, 1.0;


    return jacobian_platform;

}



Matrix6d ControllerSNS::Arm_Mapping(double rotation){


    Matrix6d Transformation_ArmBase_Platform;
    Matrix6d Transformation_Platform_b;
    Matrix6d Transformation_b_World;
    Matrix6d Transformation;

    //double angle=-45*M_PI/180.0;

  //----------------------------- Arm Base frame --> Platform frame  ----------------------------------//

    Rot_ArmBase_Platform = Eigen::Matrix3d::Identity(3,3);
/*    Rot_ArmBase_Platform << cos(M_PI), -sin(M_PI), 0.0,
                            sin(M_PI),  cos(M_PI), 0.0,
                                0.0,        0.0,   1.0;
*/


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




Matrix6d ControllerSNS::Mir_Mapping(double rotation){
  

    Matrix6d H_platform;

    Vector3d ee_ArmBase_pose = compute_arm_fk(joint_real_position, joint_real_velocity).block<3,1>(0,3);
    ee_ArmBase_pose[2] = ee_ArmBase_pose[2] - 0.354;

    Vector3d platform_b_pose;
    platform_b_pose[0] = b_;
    platform_b_pose[1] = 0.0;
    platform_b_pose[2] = 0.0;

    Vector3d ArmBase_platform_pose;
    ArmBase_platform_pose[0] = 0.0;
    ArmBase_platform_pose[1] = 0.0;
    ArmBase_platform_pose[2] = 0.354;
    
    Vector3d W_r_b_E;
    W_r_b_E = Rot_b_World * (ArmBase_platform_pose + Rot_Platform_b * Rot_ArmBase_Platform * ee_ArmBase_pose - platform_b_pose);

    Eigen::Matrix3d W_r_b_E_skewsimmetric;
    W_r_b_E_skewsimmetric <<        0.0,      -W_r_b_E(2),   W_r_b_E(1),
                                W_r_b_E(2),       0.0,      -W_r_b_E(0),
                               -W_r_b_E(1),    W_r_b_E(0),       0.0;


    
    H_platform << Eigen::Matrix3d::Identity(3,3),
                  -W_r_b_E_skewsimmetric,
                  Eigen::Matrix3d::Zero(3,3),
                  Eigen::Matrix3d::Identity(3,3);


    // nel modello unico in simulazione, movit misura la distanza dell'end-effector
    // dal frame della piattaforma, quidni considera già l'altezza in z pari a 0.354

    return H_platform;
}



Matrix6d ControllerSNS::Transformata(double rotation){


    Eigen::Matrix3d R_platform_world;
    R_platform_world << cos(rotation), -sin(rotation), 0,
                        sin(rotation),  cos(rotation), 0,
                              0,               0,      1;

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




Matrix6d ControllerSNS::Transformata_2(double rotation){


    Eigen::Matrix3d R_platform_world;
    R_platform_world << cos(rotation), -sin(rotation), 0,
                        sin(rotation),  cos(rotation), 0,
                              0,                0,     1;

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
/*
    T_world_platf << R_world_platform,
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
    //Transform = T_world_platf;
    Transform = T_platf_world;
    
    return Transform;

}



Vector8d ControllerSNS::limit_joints_dynamics (Vector8d joint_velocity) {

    // Limit Joint Velocity

    ROS_INFO_STREAM("joint velocity last cycle:\n" << adm_twist_last_cycle << "\n" );
    ROS_INFO_STREAM("joint velocity in ingresso:\n" << joint_velocity << "\n");
    ROS_INFO_STREAM("joint velocity in ingresso min Coeff:\n" << joint_velocity.minCoeff() << "\n");
    ROS_INFO_STREAM("joint velocity in ingresso max Coeff:\n" << joint_velocity.maxCoeff() << "\n");

    Vector8d scaling;
 

    for (int i = 0; i < joint_velocity.rows(); i++)
    {
      if(fabs(joint_velocity[i]) > vel_limits[i])
        scaling[i] = vel_limits[i] / fabs(joint_velocity[i]);
      else 
        scaling[i] = 1;
    }

    
    ROS_INFO_STREAM("scaling factors:\n" << scaling << "\n");

    double s_ = scaling.minCoeff();
    ROS_INFO_STREAM("scaling factor valore di riferimento:\n" << s_ << "\n");
    
    joint_velocity = s_ * joint_velocity;
    
    ROS_INFO_STREAM("joint velocity scalati:\n" << joint_velocity << "\n");


    for (int i = 0; i < joint_velocity.rows(); i++) {
        if (fabs(joint_velocity[i]) > vel_limits[i]) {       
          joint_velocity[i] = sign(joint_velocity[i]) * vel_limits[i];
        }
    }

    ROS_INFO_STREAM("Joint dopo saturazione di velocità: \n" << joint_velocity << "\n");
    // Limit Joint Acceleration

    for (int i = 0; i < joint_velocity.rows(); i++) {
        if (fabs(joint_velocity[i] - adm_twist_last_cycle[i]) > acc_limits[i] * cycle_time) {
            joint_velocity[i] = adm_twist_last_cycle[i] + sign(joint_velocity[i] - adm_twist_last_cycle[i]) * acc_limits[i] * cycle_time;
        }
    }


    ROS_INFO_STREAM("joint velocity in uscita:\n" << joint_velocity << "\n");

    adm_twist_last_cycle = joint_velocity;
    return joint_velocity;

}


/*
Vector2d ControllerSNS::limit_wheels_dynamics(Vector2d wheel_velocity)
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
*/


int ControllerSNS::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}



bool ControllerSNS::Exceed_limits_check(Vector8d joint_velocity){

    bool check;
    check = false;

    for (int i = 0; i < joint_velocity.rows(); i++)
    {
      if(fabs(joint_velocity[i]) > vel_limits[i]){
          check = true;
          break;
      }
    }

    return check;
}


void ControllerSNS::Spinner()
{
    
    //Compute_Transformation();
    //ROS_INFO_STREAM("LA TRASFORMAZIONE SPAZIALE: \n" << Transform_ee_platform << "\n");

    Matrix6d jacobian_manipulator;
    jacobian_manipulator = Arm_Mapping(yaw) * compute_arm_jacobian(joint_real_position, joint_real_velocity);
    //jacobian_manipulator = compute_arm_jacobian(joint_real_position, joint_real_velocity);
    //jacobian_manipulator = Transform_platform_b * Transform_ee_platform * compute_arm_jacobian(joint_real_position, joint_real_velocity);
    //jacobian_manipulator = Transform_platform_odom * jacobian_arm_kdl;
    //jacobian_manipulator = jacobian_arm_kdl;
    ROS_INFO_STREAM("Jacobian del Braccio Complessivo: \n" << jacobian_manipulator << "\n");


    Matrix62d jacobian_base_mapped;
    //jacobian_base_mapped = ee_mapping_matrix * compute_platform_jacobian(yaw);
    //jacobian_base_mapped = Mir_Mapping(yaw) * compute_platform_jacobian(yaw);
    //jacobian_base_mapped = Transformata_2(yaw) * Mir_Mapping(yaw) * Transformata(yaw) * compute_platform_jacobian(yaw);
    //jacobian_base_mapped = Mir_Mapping(yaw) * Transformata(yaw) * compute_platform_jacobian(yaw);
    jacobian_base_mapped = compute_platform_jacobian(yaw);    
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
    Eigen::MatrixXd adm_acceleration(6,1);
    Eigen::MatrixXd adm_twist(6,1);
    //Eigen::MatrixXd twist(6,1);
    Eigen::MatrixXd q_dot_base(2,1);
    Eigen::MatrixXd q_dot_arm(6,1);
    Eigen::MatrixXd q_dot(8,1);

    for(int i = 0; i < q_dot_arm.rows(); i++){
      q_dot_arm(i,0) = joint_real_velocity[i];
    }

    ROS_INFO_STREAM("Velocità dei giunti del braccio: \n" << q_dot_arm << "\n");
    
    q_dot_base(0,0) = R/2 * (omega_r + omega_l);
    q_dot_base(1,0) = R/L * (omega_r - omega_l);

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

    Eigen::MatrixXd adm_qdot(8,1);
    Eigen::MatrixXd adm_qdot_arm(6,1);
    Eigen::MatrixXd adm_qdot_base(2,1);
    Eigen::MatrixXd delta_q(6,1);
    Eigen::MatrixXd adm_twist_base(6,1);

    //twist = adm_twist;
    //adm_qdot = J_pinv * adm_twist;
    
   ROS_INFO_STREAM("ADMISSIBLE Q DOT: \n" << adm_qdot << "\n");


//###########################################################################################
//############################ SNS ALGORITHM ################################################
//###########################################################################################

    Eigen::MatrixXd W (8,8);
    Eigen::MatrixXd W_(8,8);
    Vector8d q_dot_N;
    Vector8d q_dot_N_;
    double s = 1.0;
    double s_ = 0.0;
    Vector8d S_max;
    Vector8d S_min;
    double scaling_factor;
    bool limit_exceeded;

    ROS_INFO("inizializzato le variabili \n");
    
    W.setIdentity();
    q_dot_N.setZero();
    ROS_INFO("inizializzato W e q_dot_N \n");
    ROS_INFO_STREAM("W: \n" << W << "\n");
    ROS_INFO_STREAM("q_dot_N: \n" << q_dot_N << "\n");

    do{
      
        limit_exceeded = false;
      
        Eigen::MatrixXd inv_1(8,6);
        inv_1 = (jacobian_augmented * W).completeOrthogonalDecomposition().pseudoInverse();
        ROS_INFO_STREAM("inv_1: \n" << inv_1 << "\n");
        //ROS_INFO_STREAM("q_dot_SNS: \n" << q_dot_SNS << "\n");
        
        q_dot_SNS = q_dot_N + inv_1 * (s * adm_twist - jacobian_augmented * q_dot_N);
    
        ROS_INFO_STREAM("q_dot_SNS_1: \n" << q_dot_SNS << "\n");

      for (int i = 0; i < 8; i++)
      {

        limit_exceeded = Exceed_limits_check(q_dot_SNS);
        
        
        // Task scaling factor Algorithm
        if(limit_exceeded == true){

            Eigen::MatrixXd inv_2(8,6);
            inv_2 = (jacobian_augmented * W).completeOrthogonalDecomposition().pseudoInverse();
            ROS_INFO_STREAM("inv_2: \n" << inv_2 << "\n");

            Vector8d a = inv_2 * adm_twist;
            //Vector8d b = q_dot_SNS - a;
            Vector8d b = q_dot_N - inv_2 * jacobian_augmented * q_dot_N;

            for (int j = 0; j < 8; j++)
            {
                S_min[j] = (Vel_LowerLimit[j] - b[j]) / a[j];
                S_max[j] = (Vel_UpperLimit[j] - b[j]) / a[j];

                if (S_min[j] > S_max[j])
                {
                  double swap_min = S_min[j];
                  double swap_max = S_max[j];
                      
                  S_min[j] = swap_max;
                  S_max[j] = swap_min;
                }            
            }

            double s_max = S_max.minCoeff();
            double s_min = S_min.maxCoeff();
              
            
            int most_critical;
            for (int z = 0; z < 8; z++)
            {
              if (S_max[z] == s_max)
              {
                most_critical = z;
              }
            }
            

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
              
            W(most_critical, most_critical) = 0.0;
              
            if (fabs(q_dot_N[most_critical]) > vel_limits[most_critical])
            {
                q_dot_N[most_critical] = sign(q_dot_N[most_critical]) * vel_limits[most_critical];

                if (fabs(q_dot_N[most_critical] - q_dot_SNS_last_cycle[most_critical]) > acc_limits[most_critical] * cycle_time) 
                    q_dot_N[most_critical] = q_dot_SNS_last_cycle[most_critical] + sign(q_dot_N[most_critical] - q_dot_SNS_last_cycle[most_critical]) * acc_limits[most_critical] * cycle_time;
            }


            Eigen::MatrixXd inv_3(8,6);
            inv_3 = (jacobian_augmented * W).completeOrthogonalDecomposition().pseudoInverse();
            ROS_INFO_STREAM("inv_3: \n" << inv_3 << "\n");

            q_dot_SNS = q_dot_N + inv_3 * (s * adm_twist - jacobian_augmented * q_dot_N);
            ROS_INFO_STREAM("q_dot_SNS_2: \n" << q_dot_SNS << "\n");
            ROS_INFO_STREAM("W_3 : \n" << W << "\n");

            Eigen::MatrixXd prod_3(6,8);
            prod_3 = jacobian_augmented * W;
            ROS_INFO_STREAM("PROD_3:\n" << prod_3 << "\n");
            Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(prod_3);
            int rank = lu_decomp.rank();
            
            ROS_INFO_STREAM("CALCULATED RANK: \t " << rank << "\n");
            ROS_INFO_STREAM("MOST CRITICAL: \t" << most_critical);
            if (rank < 6)
            {
              s = s_;
              W = W_;
              q_dot_N = q_dot_N_;
  
              Eigen::MatrixXd inv_4(8,6);
              inv_4 = (jacobian_augmented * W).completeOrthogonalDecomposition().pseudoInverse();
              ROS_INFO_STREAM("inv_4: \n" << inv_4 << "\n");

              q_dot_SNS = q_dot_N + inv_4 * (s * adm_twist - jacobian_augmented * q_dot_N);
              ROS_INFO_STREAM("q_dot_SNS_3: \n" << q_dot_SNS << "\n");
              
              limit_exceeded = false;
            }            

          }   
      }
      
    } while (limit_exceeded == true);
    
    ROS_INFO_STREAM("Q_dot_SNS in uscita dall'algoritmo: \n" << q_dot_SNS << "\n");
    ROS_INFO("DIOOOOOOOO PORCOOOOOOOOOO");
    
    q_dot_SNS_last_cycle = q_dot_SNS;
    Vel_LowerLimit = q_dot_SNS_last_cycle - acc_limits * cycle_time;
    Vel_UpperLimit = q_dot_SNS_last_cycle + acc_limits * cycle_time;
  
//###########################################################################################
//###########################################################################################
//###########################################################################################

    adm_qdot = q_dot_SNS;

    //adm_qdot = limit_joints_dynamics(adm_qdot);

    //le variabili di giunto del manipolatore sono solo le prime 6
    for(int i = 0; i < adm_qdot_arm.rows(); i++){
      adm_qdot_arm(i,0) = adm_qdot(i,0);
    }    

    //mentre le variabili di giunto della base sono le ultime 2  
    for(int i = 6; i < adm_qdot.rows(); i++){
      adm_qdot_base(i-6, 0) = adm_qdot(i,0);
    }


    ROS_INFO_STREAM("ADMISSIBLE Q DOT ARM: \n" << adm_qdot_arm << "\n");
    ROS_INFO_STREAM("ADMISSIBLE Q DOT BASE: \n" << adm_qdot_base << "\n");
    

//###################### ARM ######################################
    
    delta_q = adm_qdot_arm * cycle_time;
    ROS_INFO_STREAM("DELTA Q: \n" << delta_q << "\n");

    Eigen::MatrixXd q_arm(6,1); 

    for(int i = 0; i < q_arm.rows(); i++)
      q_arm(i,0) = joint_real_position[i];

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


//###################### BASE ######################################
    //lineare = adm_twist_base(0,0) * cos(yaw) + adm_twist_base(1,0) * sin(yaw);
    //angolare = -sin(yaw) * adm_twist_base(0,0) / b_ + cos(yaw) * adm_twist_base(1,0) / b_;

    // per rispettare i vincoli di velocità massima dei giunti 
    //adm_qdot_base(0,0) = 5 * adm_qdot_base(0,0);
    //adm_qdot_base(0,0) = 5 * adm_qdot_base(0,0);

/*
    Vector2d vel_cartesiane = limit_wheels_dynamics(adm_qdot_base);

    base_message.linear.x = vel_cartesiane[0];
    base_message.angular.z = vel_cartesiane[1];

    ROS_INFO_STREAM("Lineare: \n" << vel_cartesiane[0] << "\n");
    ROS_INFO_STREAM("Angolare: \n" << vel_cartesiane[1] << "\n");
    ROS_INFO_STREAM("Omega sinistro: \n" << adm_qdot_base(0,0) << "\n");
    ROS_INFO_STREAM("Omega destro: \n" << adm_qdot_base(1,0) << "\n");
*/

    base_message.linear.x = adm_qdot_base(0,0);
    base_message.angular.z = adm_qdot_base(1,0);
    
    ROS_INFO_STREAM("Lineare: \n" << adm_qdot_base(0,0) << "\n");
    ROS_INFO_STREAM("Angolare: \n" << adm_qdot_base(1,0) << "\n");
    
    mobile_pub.publish(base_message);
   
}

