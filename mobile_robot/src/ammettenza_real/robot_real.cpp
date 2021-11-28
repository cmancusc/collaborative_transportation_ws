

// Created: 2021/03/17 12:24:41
// Last modified: 2021/05/07 17:42:47


#include "ammettenza_real/robot_real.h"
// #include "solver.h"
// #include "solver.c"
// #include "ldl.c"
// #include "matrix_support.c"
// #include "util.c"


Controller_robot_real::Controller_robot_real(){


    odometry_sub = nh.subscribe("/odom_enc", 1, &Controller_robot_real::OdometryCallback, this);
    joints_state_sub= nh.subscribe("/joint_states", 1, &Controller_robot_real::JointStateCallback, this);
    mir_joint_sub = nh.subscribe("/mir_joint_states", 1, &Controller_robot_real::RuoteCallback, this);
    //joystick_sub = nh.subscribe("/joy", 1, &Controller_robot_real::joystickCallback, this);
    force_sub = nh.subscribe("/wrench", 1, &Controller_robot_real::ForceCallback, this);
    //arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/ur10e/manipulator_joint_trajectory_Controller_robot_real/command",1);
    arm_pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 1);
    mobile_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    tank_node = nh.advertise<std_msgs::Float32>("/tank_node", 1);
    h_1_node = nh.advertise<std_msgs::Float32>("/h_1_node", 1);
    h_2_node = nh.advertise<std_msgs::Float32>("/h_2_node", 1);

    R = 0.0625;
    L = 0.445208;
    cycle_time = 0.02;
//    roll = 0.0;
//    pitch = 0.0;
//    yaw = 0.0;
//    vel_x = 0.0;
//    omega_z = 0.0;
//    omega_l = 0.0;
//    omega_r = 0.0;

    //IO-SFL lead value
    b_ = -0.7;

//    ArmBase_platform_pose(0) = 0.316;
//    ArmBase_platform_pose(1) = 0.085;
//    ArmBase_platform_pose(2) = 0.847;

    opt_qdot_prev.setZero();

    start_time_ = ros::Time::now().toSec();

//   for(int i = 0; i < 6; i++){
//     joint_real_position[i] = 0.0;
//     joint_real_velocity[i] = 0.0;
//   }

    //################# VEl, ACC, MASS & DAMPING RETRIEVING ###############//

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

    // fixed_damping = 20;
    // fixed_mass = 4;
    // min_damping = 10;
    // max_damping = 60;

    for(int i = 0; i < 6; i++){
      for(int j = 0; j < 6; j++){
        if(i == j){
          fixed_damping(i) = D_tot(i,j);
          fixed_mass(i) = M_tot(i,j);
        }
      }
    }

    for(int i = 0; i < 6; i++){
      if(i <3){
        max_damping(i) = 60.0;
        min_damping(i) = 10.0;
      }
      else
      {
        max_damping(i) = 30.0;
        min_damping(i) = 5.0;
      }

    }

    //########## TANK INITIALIZATION ###########//

    tank_energy_ = TANK_INITIAL_VALUE;
    tank_state_ = sqrt(2 * tank_energy_);
    sum_of_delta_ =  0.0;

    //########## FILE INITIALIZATION ###########//

    std::stringstream file_path;
    file_path << "/home/cristian/catkin_ws/src/mobile_robot/debug/q_dot_scaling_debug.txt";
    q_dot_file.open(file_path.str());

    std::stringstream file_path1;
    file_path1 << "/home/cristian/catkin_ws/src/mobile_robot/debug/q_dot_SNS_debug.txt";
    q_dot_SNS_file.open(file_path1.str());

    std::stringstream file_path2;
    file_path2 << "/home/cristian/catkin_ws/src/mobile_robot/debug/force_debug.txt";
    force_file.open(file_path2.str());

    std::stringstream file_path3;
    file_path3 << "/home/cristian/catkin_ws/src/mobile_robot/debug/q_dot_opt_debug.txt";
    q_dot_opt_file.open(file_path3.str());

    std::stringstream file_path4;
    file_path4 << "/home/cristian/catkin_ws/src/mobile_robot/debug/energy_tank_debug.txt";
    tank_file.open(file_path4.str());

    //######### Create the UR10e message ##############//

    robot_model_loader = robot_model_loader::RobotModelLoader ("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    joint_names = joint_model_group->getJointModelNames();

//    force_filter = new andrea_filters::RCFilter(6, 10, cycle_time);

}


Controller_robot_real::~Controller_robot_real(){

    q_dot_file.close();
    q_dot_SNS_file.close();
    force_file.close();
    q_dot_opt_file.close();
    tank_file.close();

}


void Controller_robot_real::OdometryCallback(const nav_msgs::Odometry& msg){


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

    vel_x = msg.twist.twist.linear.x;
    //vel_x = pow( pow(msg.twist.twist.linear.x, 2) + pow(msg.twist.twist.linear.y, 2) , 2);
    omega_z = msg.twist.twist.angular.z;

    x_platform = msg.pose.pose.position.x;
    y_platform = msg.pose.pose.position.y;
    z_platform = msg.pose.pose.position.z;

}



void Controller_robot_real::RuoteCallback(const sensor_msgs::JointState& msg){

    omega_l = msg.velocity[0];
    omega_r = msg.velocity[1];

}





//void Controller_robot_real::joystickCallback(const sensor_msgs::Joy& msg){


  /*
    Guida:  msg.axes[0]; //levetta sx orizzontale --> Fx
            msg.axes[1]; //levetta sx verticale --> Fy
            msg.axes[4]; //levetta dx verticale --> Fz
            msg.axes[3]; //levetta dx orizzontale --> Mx
            msg.axes[6]; //analogico orizzontale --> My
            msg.axes[7]; //analogico verticale --> Mz
  */

/*
  double intensity = 30;

  if(abs(msg.axes[0]) > 0.5)
    F_ext[0] = msg.axes[0] * intensity; //levetta sx orizzontale --> Fx
  else
    F_ext[0] = 0.0;

  if(abs(msg.axes[1]) > 0.5)
    F_ext[1] = msg.axes[1] * intensity; //levetta sx verticale --> Fy
  else
    F_ext[1] = 0.0;

  if(abs(msg.axes[4]) > 0.5)
    F_ext[2] = msg.axes[4] * intensity; //levetta dx verticale --> Fz
  else
    F_ext[2] = 0.0;

  if(abs(msg.axes[3]) > 0.5)
    F_ext[3] = msg.axes[3] * intensity; //levetta dx orizzontale --> Mx
  else
    F_ext[3] = 0.0;

  if(abs(msg.axes[6]) > 0.5)
    F_ext[3] = msg.axes[6] * intensity; //analogico orizzontale --> My
  else
    F_ext[4] = 0.0;

  if(abs(msg.axes[7]) > 0.5)
    F_ext[5] = msg.axes[7] * intensity; //analogico verticale --> Mz
  else
    F_ext[5] = 0.0;

}
*/



void Controller_robot_real::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){


    joint_state = *msg;

    // Ur10e Real Robot has Inverted Joints
    std::swap(joint_state.name[0], joint_state.name[2]);
    std::swap(joint_state.effort[0], joint_state.effort[2]);
    std::swap(joint_state.position[0], joint_state.position[2]);
    std::swap(joint_state.velocity[0], joint_state.velocity[2]);

    for (unsigned int i = 0; i < joint_state.position.size(); i++) {joint_real_position[i] = joint_state.position[i];}
    for (unsigned int i = 0; i < joint_state.velocity.size(); i++) {joint_real_velocity[i] = joint_state.velocity[i];}

}




void Controller_robot_real::ForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg){


    geometry_msgs::WrenchStamped force_sensor = *msg;

    F_ext.setZero();

    F_ext(0) = force_sensor.wrench.force.x;
    F_ext(1) = force_sensor.wrench.force.y;
    F_ext(2) = force_sensor.wrench.force.z;
    F_ext(3) = force_sensor.wrench.torque.x;
    F_ext(4) = force_sensor.wrench.torque.y;
    F_ext(5) = force_sensor.wrench.torque.z;

//    std::vector<double> F_filt(6);

//    for(int i=0; i < 6; i++){
//      F_filt[i] = F_ext(i);
//    }

//    F_filt = force_filter->filter(F_filt);

//    for(int i=0; i < 6; i++){
//      F_ext(i) = F_filt[i];
//    }


/*
  F_ext(0) = 0.0;
  F_ext(1) = 0.0;
  F_ext(2) = 0.0;
  F_ext(3) = 0.0;
  F_ext(4) = 0.0;
  F_ext(5) = 1.0;
*/



    for(int i = 0; i < 3; i++){
        if((F_ext(i) > 0.0) && (F_ext(i) < 4.0))
            F_ext(i) = 0.0;

        else if((F_ext(i) < 0.0) && (F_ext(i) > - 4.0))
            F_ext(i) = 0.0;
    }

    for(int i = 3; i < F_ext.rows(); i++){
        if((F_ext(i) > 0.0) && (F_ext(i) < 0.5))
            F_ext(i) = 0.0;

        else if((F_ext(i) < 0.0) && (F_ext(i) > - 0.5))
            F_ext(i) = 0.0;
    }



}




Matrix4d Controller_robot_real::compute_arm_fk(double joint_position[], double joint_velocity[]) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "ur10e_base_link"
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

    // Get the Translation Vector and Rotation Matrix
    Eigen::Vector3d translation_vector = end_effector_state.translation();
    Eigen::Matrix3d rotation_matrix = end_effector_state.rotation();

    //Transformation Matrix
    Matrix4d transformation_matrix;
    transformation_matrix.setZero();

    //Set Identity to make bottom row of Matrix 0,0,0,1
    transformation_matrix.setIdentity();

    transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    transformation_matrix.block<3,1>(0,3) = translation_vector;

    return transformation_matrix;

}



Vector6d Controller_robot_real::compute_ee_pose(double joint_position[], double joint_velocity[]) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "ur10e_base_link"
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

    // Get the Translation Vector and Rotation Matrix
    Eigen::Vector3d translation_vector = end_effector_state.translation();
    Eigen::Matrix3d ee_pose_angular = end_effector_state.rotation();

    // Computing current RPY of the EE
    Eigen::Vector3d euler_angles_ee = ee_pose_angular.eulerAngles(2, 1, 0);
    double roll_ee = euler_angles_ee[2];
    double pitch_ee = euler_angles_ee[1];
    double yaw_ee = euler_angles_ee[0];

    Matrix6d Transformation_ur_mir;
    Transformation_ur_mir << Rot_b_World * Rot_ArmBase_Platform,
                             Eigen::Matrix3d::Zero(3,3),
                             Eigen::Matrix3d::Zero(3,3),
                             Rot_b_World * Rot_ArmBase_Platform;

    //End Effector space position and orientation vector;
    Vector6d ee_pose;
    ee_pose << translation_vector, roll_ee, pitch_ee, yaw_ee;
    ee_pose = Transformation_ur_mir * ee_pose;

    return ee_pose;

}



Eigen::MatrixXd Controller_robot_real::compute_arm_jacobian (double joint_position[], double joint_velocity[]) {

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



Matrix62d Controller_robot_real::compute_platform_jacobian(double rotation){


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

    b11 = cos(rotation);
    b12 = - b_ * sin(rotation);
    b21 = sin(rotation);
    b22 = b_ * cos(rotation);


    jacobian_platform << b11, b12,
                         b21, b22,
                         0.0, 0.0,
                         0.0, 0.0,
                         0.0, 0.0,
                         0.0, 0.0;

    ROS_INFO_STREAM("JACOBIANO MIR:\n" << jacobian_platform << "\n");

    return jacobian_platform;

}



Matrix6d Controller_robot_real::Arm_Mapping(double rotation){


    Matrix6d Transformation_ArmBase_Platform;
    Matrix6d Transformation_Platform_b;
    Matrix6d Transformation_b_World;
    Matrix6d Transformation;

    //double angle = -45*M_PI/180.0;
    double angle = 135*M_PI/180.0;
  //----------------------------- Arm Base frame --> Platform frame  ----------------------------------//

    Rot_ArmBase_Platform <<  cos(angle),  -sin(angle),   0.0,
                             sin(angle),   cos(angle),   0.0,
                             0.0,         0.0,   1.0;



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

    //Nel frame di WORLd
    Transformation = Transformation_b_World * Transformation_Platform_b * Transformation_ArmBase_Platform;

    //nel frame del MIR
    //Transformation = Transformation_Platform_b * Transformation_ArmBase_Platform;
  //---------------------------------------------------------------------------------------------------//

    return Transformation;

}




Matrix6d Controller_robot_real::Mir_Mapping(double rotation){


    Matrix6d H_platform;

    ee_ArmBase_pose = compute_arm_fk(joint_real_position, joint_real_velocity).block<3,1>(0,3);

    ArmBase_platform_pose(0) = 0.316;
    ArmBase_platform_pose(1) = 0.085;
    ArmBase_platform_pose(2) = 0.847;

    platform_b_pose(0) = b_;
    platform_b_pose(1) = 0.0;
    platform_b_pose(2) = 0.0;

    b_world_pose(0) = x_platform + b_ * cos(rotation);
    b_world_pose(1) = y_platform + b_ * sin(rotation);
    b_world_pose(2) = z_platform;


    //########### Nel frame di WORLD ############

    //Il braccio fino al punto di conrtrollo
    W_r_b_E = Rot_b_World * (ArmBase_platform_pose + Rot_Platform_b * Rot_ArmBase_Platform * ee_ArmBase_pose - platform_b_pose);

    //Il braccio fino alla mezzeria delle ruote
    //W_r_b_E = Rot_b_World * (ArmBase_platform_pose + Rot_ArmBase_Platform * ee_ArmBase_pose);

    //############################################


    //######### Nel frame del MIR ###############

    //Il braccio fino al punto di conrtrollo
    //W_r_b_E = ArmBase_platform_pose + Rot_Platform_b * Rot_ArmBase_Platform * ee_ArmBase_pose - platform_b_pose;

    //Il braccio fino alla mezzeria delle ruote
    //W_r_b_E = ArmBase_platform_pose + Rot_ArmBase_Platform * ee_ArmBase_pose;

    //############################################

    W_r_B_E = ArmBase_platform_pose + Rot_ArmBase_Platform * ee_ArmBase_pose;


    Eigen::Matrix3d W_r_b_E_skewsimmetric;
    W_r_b_E_skewsimmetric <<        0.0,      -W_r_b_E(2),   W_r_b_E(1),
                                W_r_b_E(2),       0.0,      -W_r_b_E(0),
                               -W_r_b_E(1),    W_r_b_E(0),       0.0;



    H_platform << Eigen::Matrix3d::Identity(3,3),
                  -W_r_b_E_skewsimmetric,
                  Eigen::Matrix3d::Zero(3,3),
                  Eigen::Matrix3d::Identity(3,3);


/*
    H_platform << 1.0, 0.0, 0.0, 0.0, 0.0, W_r_b_E(1),
                  0.0, 1.0, 0.0, 0.0, 0.0, W_r_b_E(0),
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
*/

    return H_platform;
}



Matrix6d Controller_robot_real::Transformata(){

    Matrix6d T_world_platf;
    Eigen::Matrix3d R_world_platform;

    R_world_platform = Rot_b_World.inverse();

    T_world_platf << R_world_platform,
                     Eigen::Matrix3d::Zero(3,3),
                     Eigen::Matrix3d::Zero(3,3),
                     R_world_platform;

    return T_world_platf;
}




Matrix6d Controller_robot_real::get_ee_rotation_matrix(double joint_position[], double joint_velocity[]) {

    ros::spinOnce();

    //Update MoveIt! Kinematic Model
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position);
    kinematic_state->setJointGroupVelocities(joint_model_group, joint_velocity);
    kinematic_state->enforceBounds();

    // Computing the actual position of the end-effector using Forward Kinematic respect "ur10e_base_link"
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("tool0");

    // Rotation Matrix 6x6
    Matrix6d rotation_matrix;
    rotation_matrix.setZero();

    Eigen::Matrix3d Rot_ee_odom;

    //############# Nel frame del WORLD ############
    Rot_ee_odom = Rot_b_World * Rot_Platform_b * Rot_ArmBase_Platform * end_effector_state.rotation();

    //############# Nel frame del MIR ################
    //Rot_ee_odom = Rot_ArmBase_Platform * end_effector_state.rotation();

    rotation_matrix.topLeftCorner(3, 3) = Rot_ee_odom;
    rotation_matrix.bottomRightCorner(3, 3) = Rot_ee_odom;

    //Eigen::Vector3d euler_angles = end_effector_state.rotation().eulerAngles(0, 1, 2);
    //Eigen::Quaterniond rotation_quaternion(end_effector_state.rotation());

    return rotation_matrix;

}



Vector6d Controller_robot_real::Force_in_Odom_Frame(){

    Vector6d Force;

    Force = get_ee_rotation_matrix(joint_real_position, joint_real_velocity) * F_ext;

//    Force(2) = Force(2) + 8;

    return Force;
}



Vector8d Controller_robot_real::limit_joints_dynamics (Vector8d joint_velocity) {


    ROS_INFO_STREAM("joint velocity last cycle:\n" << adm_qdot_last_cycle << "\n" );
    ROS_INFO_STREAM("joint velocity in ingresso:\n" << joint_velocity << "\n");
    ROS_INFO_STREAM("joint velocity in ingresso min Coeff:\n" << joint_velocity.minCoeff() << "\n");
    ROS_INFO_STREAM("joint velocity in ingresso max Coeff:\n" << joint_velocity.maxCoeff() << "\n");

    Vector8d scaling;

    for (int i = 0; i < joint_velocity.rows(); i++)
    {
      if(fabs(joint_velocity(i)) > vel_limits(i))
        scaling(i) = vel_limits(i) / fabs(joint_velocity(i));
      else
        scaling(i) = 1.0;
    }


    ROS_INFO_STREAM("scaling factors:\n" << scaling << "\n");

    double s_ = scaling.minCoeff();
    ROS_INFO_STREAM("scaling factor valore di riferimento:\n" << s_ << "\n");

    joint_velocity = s_ * joint_velocity;

    ROS_INFO_STREAM("joint velocity scalati:\n" << joint_velocity << "\n");


    // Limit Joint Velocity

    for (int i = 0; i < joint_velocity.rows(); i++) {
        if (fabs(joint_velocity(i)) > vel_limits(i)) {
          joint_velocity(i) = sign(joint_velocity(i)) * vel_limits(i);
        }
    }

    ROS_INFO_STREAM("Joint dopo saturazione di velocità: \n" << joint_velocity << "\n");


    // Limit Joint Acceleration

    for (int i = 0; i < joint_velocity.rows(); i++) {
        if (fabs(joint_velocity(i) - adm_qdot_last_cycle(i)) > acc_limits(i) * cycle_time) {
            joint_velocity(i) = adm_qdot_last_cycle(i) + sign(joint_velocity(i) - adm_qdot_last_cycle(i)) * acc_limits(i) * cycle_time;
        }
    }


    ROS_INFO_STREAM("joint velocity in uscita:\n" << joint_velocity << "\n");

    adm_qdot_last_cycle = joint_velocity;

    return joint_velocity;
}




Vector8d Controller_robot_real::SatNullSpace(Vector6d cartesian_velocity, Eigen::MatrixXd system_jacobian, Vector8d Vel_LowerLimit, Vector8d Vel_UpperLimit){

    Eigen::MatrixXd W(8,8);
    Eigen::MatrixXd W_(8,8);
    Vector8d q_dot_SNS;
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
      prod_1 = (system_jacobian * W).completeOrthogonalDecomposition().pseudoInverse();

      ROS_INFO_STREAM("q_dot_SNS_1: \n" << q_dot_SNS << "\n");
      ROS_INFO_STREAM("prod_1: \n" << prod_1 << "\n");

      q_dot_SNS = q_dot_N + prod_1 * (s * cartesian_velocity - system_jacobian * q_dot_N);
      ROS_INFO_STREAM("q_dot_SNS_2: \n" << q_dot_SNS << "\n");

      Vector8d scaling_find_critical;
      //int most_critical;
      Eigen::MatrixXf::Index most_critical;

      for (int i = 0; i < q_dot_SNS.rows(); i++)
      {

        Max_Limits(i) = Vel_UpperLimit(i);
        Min_Limits(i) = Vel_LowerLimit(i);

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
              prod_2 = (system_jacobian * W).completeOrthogonalDecomposition().pseudoInverse();
              ROS_INFO_STREAM("prod_2: \n" << prod_2 << "\n");

              Vector8d a = prod_2 * cartesian_velocity;
              Vector8d b = q_dot_N - (prod_2 * system_jacobian )* q_dot_N;

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
              prod_3 = system_jacobian * W;
              Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(prod_3);
              int rank = lu_decomp.rank();

              ROS_INFO_STREAM("rank:\t" << rank << "\n");
              if (rank < 6)
              {
                s = s_;
                W = W_;
                q_dot_N = q_dot_N_;
                Eigen::MatrixXd prod_4(6,8);
                prod_4 = (system_jacobian * W).completeOrthogonalDecomposition().pseudoInverse();
                q_dot_SNS = q_dot_N + prod_4 * (s * cartesian_velocity - system_jacobian * q_dot_N);
                limit_exceeded = false;
              }

          }

    } while (limit_exceeded == true);

    return q_dot_SNS;
}



void Controller_robot_real::send_velocity_to_robot (Vector6d velocity) {

    std_msgs::Float64MultiArray msg;

    std::vector<double> velocity_vector(velocity.data(), velocity.data() + velocity.size());

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = velocity.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "velocity";

    // copy in the data
    msg.data.clear();
    msg.data.insert(msg.data.end(), velocity_vector.begin(), velocity_vector.end());

    arm_pub.publish(msg);

}



int Controller_robot_real::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}

}


//Vector8d KALMAN(Vector8d measure){

//    Vector8d R; //noise covariance
//    Vector8d H; //measurement map scalar
//    Vector8d Q; //initial estimated covariance
//    Vector8d P; //initial error covariance
//    Vector8d measure_hat; //initial estimated state
//    Vector8d K; //kalman gain

//    for(int i = 0; i < measure.rows(); i++){
//        K(i) = P(i)*H(i)/(H(i)*P(i)*H(i)+R(i)); // update the gain
//    }

//    for (int i = 0; i < measure.rows(); i++) {
//        measure_hat(i) += K(i) * (measure(i) - H(i) * measure_hat(i));
//    }


//}


void Controller_robot_real::Spinner()
{

    Matrix6d jacobian_manipulator;
    jacobian_manipulator = Arm_Mapping(yaw) * compute_arm_jacobian(joint_real_position, joint_real_velocity);
    ROS_INFO_STREAM("Jacobian del Braccio Complessivo: \n" << jacobian_manipulator << "\n");


    // Mir_Mapping serve per caricare i dati riguardo alla posizione dell UR10e nel
    // problema di ottimizzazione (CBF)
    Eigen::MatrixXd jacobian_base_mapped;
    jacobian_base_mapped = Mir_Mapping(yaw) * compute_platform_jacobian(yaw);
    //jacobian_base_mapped = compute_platform_jacobian(yaw);

/*
    Eigen::MatrixXd appoggio_mapping;
    appoggio_mapping = Mir_Mapping(yaw);
*/
/*
    //jacobian_base_mapped = compute_platform_jacobian(yaw);

    jacobian_base_mapped = Transformata() * compute_platform_jacobian(yaw);
    //jacobian_base_mapped = Mir_Mapping(yaw) * Transformata() * compute_platform_jacobian(yaw);
*/
    ROS_INFO_STREAM("Jacobian della Piattaforma Complessivo: \n" << jacobian_base_mapped << "\n");

    // **************** JACOBIANO AUMENTATO **************** //
    Eigen::MatrixXd jacobian_augmented(6,8);
    jacobian_augmented << jacobian_manipulator, jacobian_base_mapped;
    ROS_INFO_STREAM("Jacobian Aumentato: \n" << jacobian_augmented << "\n");

    // **************** JACOBIANO PSEUDO-INVERSO **************** //
    Eigen::MatrixXd J_pinv(8,6);
    J_pinv = jacobian_augmented.completeOrthogonalDecomposition().pseudoInverse();

    ROS_INFO_STREAM("Jacobian Aumentato PSEUDO-INVERSO: \n" << J_pinv << "\n");
    ROS_INFO_STREAM("MASS: \n" << M_tot << "\nDAMPING: \n" << D_tot << "\n");

    // **************** ADMITTANCE CONTROLLER **************** //
    Vector6d adm_acceleration;
    Vector6d adm_twist;
    Vector6d twist;
    Vector2d q_dot_base;
    Vector6d q_dot_arm;
    Vector8d q_dot;

    for(int i = 0; i < q_dot_arm.rows(); i++){
      q_dot_arm(i) = joint_real_velocity[i];
    }

    ROS_INFO_STREAM("Velocità dei giunti del braccio: \n" << q_dot_arm << "\n");

/*
    q_dot_base(0) = omega_l;
    q_dot_base(1) = omega_r;
*/

    q_dot_base(0) = vel_x;
    q_dot_base(1) = omega_z;

    ROS_INFO_STREAM("Velocità dei giunti delle ruote: \n" << q_dot_base << "\n");

    q_dot << q_dot_arm, q_dot_base;
    twist = jacobian_augmented * q_dot;


    Vector6d Force_world;
    Force_world = get_ee_rotation_matrix(joint_real_position, joint_real_velocity) * F_ext;

    adm_acceleration = M_tot.inverse() * ( - D_tot * twist + Force_world);

    //adm_acceleration = M_tot.inverse() * ( - D_tot * twist + F_ext);


    ROS_INFO_STREAM("Admissible acceleration: \n" << adm_acceleration << "\n");
    ROS_INFO_STREAM("Twist del Robot: \n" << twist << "\n");

    adm_twist = twist + adm_acceleration * cycle_time;

    ROS_INFO_STREAM("ADMISSIBLE TWIST: \n" << adm_twist << "\n");

    //############################ VARIABLE ADMITTANCE ################################//

    Vector6d variable_damping;
    Vector6d variable_mass;
    Vector6d max_cartesian_acc;
    Vector6d alpha_a;
    Vector6d alpha_d;
    Vector8d adm_qdot_first;
    Vector8d adm_qdotdot_first;
    Vector6d actual_cartesian_acc;
    double beta = 0.5;
    double gamma = 0.5;

    max_cartesian_acc = jacobian_augmented * acc_limits;


//    adm_qdot_first = SatNullSpace(adm_twist, jacobian_augmented, -vel_limits, vel_limits);

//     for (int i = 0; i < adm_qdot_first.rows(); i++) {
//         if (abs(adm_qdot_first(i) - opt_qdot_prev(i)) > acc_limits(i) * cycle_time) {
//             adm_qdot_first(i) = opt_qdot_prev(i) + sign(adm_qdot_first(i) - opt_qdot_prev(i)) * acc_limits(i) * cycle_time;
//         }
//     }

//     adm_qdotdot_first = (adm_qdot_first - opt_qdot_prev) / cycle_time;

//     //actual_cartesian_acc = jacobian_augmented * (adm_qdot_first - opt_qdot_prev) / cycle_time;
//     actual_cartesian_acc = jacobian_augmented * adm_qdotdot_first;


    for(int i = 0; i < 6; i++){
      alpha_a(i) = (fixed_damping(i) - min_damping(i)) / abs(max_cartesian_acc(i));
      alpha_d(i) = (max_damping(i) - fixed_damping(i)) / abs(max_cartesian_acc(i));
    }

    for (int i = 0; i < 6; i++){
      if(sign(adm_acceleration(i)) == sign(adm_twist(i))){
          variable_damping(i) = fixed_damping(i) - alpha_a(i) * abs(max_cartesian_acc(i));
          variable_mass(i) = fixed_mass(i) * variable_damping(i) / fixed_damping(i);
      }
      else{
          variable_damping(i) = fixed_damping(i) + alpha_d(i) * abs(max_cartesian_acc(i));
          variable_mass(i) = fixed_mass(i) / fixed_damping(i) * (1 - beta * (1 - pow(M_E, (-gamma*(variable_damping(i) - fixed_damping(i)))))) * variable_damping(i);
      }
    }

    Matrix6d M_tot_var;
    Matrix6d D_tot_var;
    M_tot_var.setZero();
    D_tot_var.setZero();

    for(int i = 0; i < 6; i++){
      for(int j = 0; j < 6; j++){
        if(i == j){
          M_tot_var(i,j) = variable_mass(i);
          D_tot_var(i,j) = variable_damping(i);
        }
      }
    }

    adm_acceleration = M_tot_var.inverse() * ( -D_tot_var * twist + Force_world);

    ROS_INFO_STREAM("Admissible acceleration: \n" << adm_acceleration << "\n");
    ROS_INFO_STREAM("Twist del Robot: \n" << twist << "\n");

    adm_twist = twist + adm_acceleration * cycle_time;

    ROS_INFO_STREAM("ADMISSIBLE TWIST: \n" << adm_twist << "\n");


    Vector8d adm_qdot;
    Vector6d adm_qdot_arm;
    Vector2d adm_qdot_base;
    Vector6d adm_twist_base;
    Vector8d adm_q_dot_scaling;

    //###############################################################################################

    //twist = adm_twist;
    //adm_qdot = J_pinv * adm_twist;
    adm_q_dot_scaling = J_pinv * adm_twist;
    //ROS_INFO_STREAM("ADMISSIBLE Q DOT: \n" << adm_qdot << "\n");

    //############################ SNS ALGORITHM #####################################//

    adm_qdot = SatNullSpace(adm_twist, jacobian_augmented, -vel_limits, vel_limits);

    // Acc_LowerLimit = q_dot_SNS_last_cycle - acc_limits * cycle_time;
    // Acc_UpperLimit = q_dot_SNS_last_cycle + acc_limits * cycle_time;

    // for (int i = 0; i < adm_qdot.rows(); i++) {
    //     if (abs(adm_qdot(i) - q_dot_SNS_last_cycle(i)) > acc_limits(i) * cycle_time) {
    //         adm_qdot(i) = q_dot_SNS_last_cycle(i) + sign(adm_qdot(i) - q_dot_SNS_last_cycle(i)) * acc_limits(i) * cycle_time;
    //     }
    // }

    // q_dot_SNS_last_cycle = adm_qdot;


    //###############################################################################//

    adm_q_dot_scaling = limit_joints_dynamics(adm_q_dot_scaling);
    //adm_qdot = adm_q_dot_scaling;

    ROS_WARN_STREAM("Q DOT SNS: \n" << adm_qdot << "\n");

    //############### LOADING THE DATA FOR THE OPTIMIZATION PROBLEM #################//

//    double adm_qdot_matlab[8];
//    // load the admittance joint velocities
//    for (int i = 0; i < adm_qdot.rows(); i++)
//    {
//      adm_qdot_matlab[i] = adm_qdot(i);
//    }

//    Vector6d A;
//    double A_matlab[6];
//    A = -cycle_time * (F_ext);

//    for (int i = 0; i < A.rows(); i++)
//    {
//      A_matlab[i] = A(i);
//    }

//    double B;
//    B = - TANK_MIN_VALUE - sum_of_delta_ + tank_energy_;


//    Vector6d end_effector_pose;
//    end_effector_pose = compute_ee_pose(joint_real_position, joint_real_velocity);
//    end_effector_pose(2) = 0.0;
//    end_effector_pose(3) = 0.0;
//    end_effector_pose(4) = 0.0;
//    end_effector_pose(5) = 0.0;

//    double x[6];
//    for (int i = 0; i < end_effector_pose.rows(); i++)
//        x[i] = end_effector_pose(i);


//    Vector3d ur_base_world_pose;
//    ur_base_world_pose = Rot_b_World * ArmBase_platform_pose;

//    double x_0[6];

//    for (int i = 0; i < 6; i++) {
//      if (i < 2) {
//        x_0[i] = ur_base_world_pose(i);
//      }
//      else {
//        x_0[i] = 0.0;
//      }
//    }

//    ROS_WARN_STREAM("X: \n" << end_effector_pose << "\n");
//    ROS_WARN_STREAM("X_0: \n" << ur_base_world_pose << "\n");

//    double d1 = 0.0;
//    double d1_2 = 0.0;
//    double d2 = 0.0;
//    double d2_2 = 0.0;
//    double h_1 = 0.0;
//    double h_2 = 0.0;

//    // radius of the smallest cylinder
//    double r_1 = 0.4;
//    // radius of the biggest cylinder
//    double r_2 = 0.7;

//    for(int i = 0; i < 2; i++){
//      d1_2 += pow(x[i] , 2);
//    }

//    d1 = sqrt(d1_2);

//    h_1 = d1_2 - pow(r_1 , 2);

//    for(int i = 0; i < 2; i++){
//      d2_2 += pow(x[i] + x_0[i] , 2);
//    }

//    d2 = sqrt(d2_2);

//    h_2 = -d2_2 + pow(r_2 , 2);

//    double beta_safe = 10.0;
//    double alpha_h1, alpha_h2;
//    alpha_h1 = beta_safe * h_1;
//    alpha_h2 = beta_safe * h_2;

//    h_1_message.data = float(h_1);
//    h_2_message.data = float(h_2);
//    h_1_node.publish(h_1_message);
//    h_2_node.publish(h_2_message);


    // LOAD THE CURRENT CONFIGURATION'S JACOBIAN
    // *NdR: CVXgen stores matrices as flat-arrays in column major form (namely
    // Aij = params.A[(i-1) + (j-1)*m)])

//    double Jacob[48];
//    double Jacob_limit[48];
//    int n = 0;
//    for (int j = 0; j < jacobian_augmented.cols(); j++)
//    {
//      for (int i = 0; i < jacobian_augmented.rows(); i++)
//      {
//        Jacob[n] = jacobian_augmented(i, j);
//        n++;
//      }
//    }

    //    for (int j = 0; j < 8; j++) {
    //      for (int i = 0; i < 6; i++) {
    //        Jacob[i + 6 * j] = jacobian_augmented(i,j);
    //      }
    //    }


//    // insert the Augmented Jacobian with the last 2 columns(regarding wheels velocities) equal to zero
//    Eigen::MatrixXd jacobiano_modificato(6,8);
//    jacobiano_modificato.setZero();
//    jacobiano_modificato.topLeftCorner(6,6) = jacobian_augmented.topLeftCorner(6,6);
//    //jacobiano_modificato << jacobian_augmented.topLeftCorner(6,6), jacobiano_nullo_mir;
//    ROS_WARN_STREAM("Jacobiano modificato: \n" << jacobiano_modificato << "\n");

//    int c = 0;
//    for(int j=0; j < jacobiano_modificato.cols(); j++){
//        for(int i=0; i < jacobiano_modificato.rows(); i++){
//            Jacob_limit[c] = jacobiano_modificato(i,j);
//            c++;
//        }
//    }


//    double a_max[8];
//    for (int i = 0; i < acc_limits.rows(); i++)
//      a_max[i] = acc_limits(i) * cycle_time;


//    double opt_qdot_prev_matlab[8];
//    for (int i = 0; i < opt_qdot_prev.rows(); i++){
//      opt_qdot_prev_matlab[i] = opt_qdot_prev(i);
//    }


//    double q_dot_0[8];
//    for (int i = 0; i < 8; ++i)
//    {
//      q_dot_0[i] = 0.0;
//    }

//    double costfun_val, exit_flag;
//    double opt_qdot_matlab[8];

    //######################## SOLVE THE OPTIMIZATION PROBLEM #########################//

//    MATLAB_OPT.matlab_opt_tank(adm_qdot_matlab, q_dot_0, A_matlab, Jacob, a_max,
//                               B, opt_qdot_prev_matlab, opt_qdot_matlab,
//                               &costfun_val, &exit_flag);


//    MATLAB_OPT.matlab_opt_tank_cbf(adm_qdot_matlab, q_dot_0, A_matlab, B, x, x_0, alpha_h1, alpha_h2,
//                                   Jacob, Jacob_limit, a_max, opt_qdot_prev_matlab, opt_qdot_matlab,
//                                   &costfun_val, &exit_flag);

//    ROS_WARN_STREAM("VALUE COST FUNCTION: \t" << costfun_val);
//    ROS_WARN_STREAM("VALUE EXIT FLAG: \t" << exit_flag);


//    Vector8d q_dot_opt;
//    Vector6d x_dot_opt;

//    if (fabs(costfun_val) > 1 || isnan(costfun_val) == 1)
//    {
//        for (int i = 0; i < 8; i++)
//        {
//            opt_qdot_matlab[i] = 0.0;
//            ROS_ERROR_STREAM("OUTPUT IN USCITA DALL'OTTIMIZATORE NON VALIDO AIUTO");
//        }
//    }

//    for(int i = 0; i < 8; i++){
//        if(isnan(opt_qdot_matlab[i])){
//            opt_qdot_matlab[i] = 0.0;
//        ROS_ERROR_STREAM("OUTPUT IN USCITA DALL'OTTIMIZATORE NON VALIDO AIUTO");
//        }
//    }


//    ROS_WARN_STREAM("Q_DOT_PREV:\n" << opt_qdot_prev << "\n");

//    for (int i = 0; i < q_dot_opt.rows(); i++)
//    {
//      q_dot_opt(i) = opt_qdot_matlab[i];      // from double to eigen
//      opt_qdot_prev(i) = opt_qdot_matlab[i];
//      ROS_WARN_STREAM("DOTQ_opt" << i << ": " << q_dot_opt(i));

//      if((opt_qdot_matlab[i] > 1.58) || (opt_qdot_matlab[i] < -1.58))
//        ROS_ERROR_STREAM("VAI TROPPO VELOCE OHHH CALMA!!!!!!!");
//    }

//    x_dot_opt = jacobian_augmented * q_dot_opt;

//    ROS_INFO_STREAM("X ADM POST OPT: " << x_dot_opt(0));
//    ROS_INFO_STREAM("Y ADM POST OPT: " << x_dot_opt(1));
//    ROS_INFO_STREAM("Z ADM POST OPT: " << x_dot_opt(2));








    //#################################################################################################

    set_defaults();  // Set basic algorithm parameters.
    setup_indexing();

    // load the admittance joint velocities
    for(int i = 0; i< adm_qdot.rows(); i++){
        params.adm_qdot[i] = adm_qdot(i);
    }

    Vector6d A;
    A = -cycle_time * (F_ext);

    for(int i = 0; i < A.rows(); i++){
        params.A[i] = A(i);
    }

    double B;
    B = - TANK_MIN_VALUE - sum_of_delta_ + tank_energy_;
    params.B[0] = B;


    Vector6d end_effector_pose;
    end_effector_pose = compute_ee_pose(joint_real_position, joint_real_velocity);
    end_effector_pose(2) = 0.0;
    end_effector_pose(3) = 0.0;
    end_effector_pose(4) = 0.0;
    end_effector_pose(5) = 0.0;

//    for (int i = 0; i < end_effector_pose.rows(); i++) {
//      params.x[i] = end_effector_pose(i);
//    }

    Vector3d ur_base_world_pose;
    ur_base_world_pose = Rot_b_World * ArmBase_platform_pose;

//    for (int i = 0; i < 6; i++) {
//      if (i < 2) {
//        params.x_0[i] = ur_base_world_pose(i);
//      }
//      else {
//        params.x_0[i] = 0.0;
//      }
//    }

//    ROS_WARN_STREAM("X: \n" << end_effector_pose << "\n");
//    ROS_WARN_STREAM("X_0: \n" << ur_base_world_pose << "\n");

//    double alpha_h1, alpha_h2;
    double d1 = 0.0;
    double d1_2 = 0.0;
    double h_1 = 0.0;

    // radius of the biggest cylinder
    double r_1 = 0.5;

    for(int i = 0; i < 6; i++){
      if(i<2){
        d1_2 += pow( end_effector_pose(i) + ur_base_world_pose(i), 2);
      }
      else {
        d1_2 += 0.0;
      }
    }

    d1 = sqrt(d1_2);

    h_1 = -d1_2 + pow(r_1 , 2);

//    params.alpha_h1[0] = beta_safe * h_1;
//    params.alpha_h2[0] = beta_safe * h_2;

    Eigen::MatrixXd P_limit(6,6);
    Eigen::MatrixXd P_zero(6,6);

    P_limit.setZero();
    P_zero.setZero();

    for (int i = 0; i < P_limit.rows(); ++i) {
      for (int j = 0; j < P_limit.cols(); ++j) {
        if (i==j && i!=2) {
          P_limit(i,j) = 1.0;
        }
      }
    }


    if(h_1 > -0.2 && h_1 < 0.1){
      params.l[0] = 1.0;

      int ciao = 0;
      for(int j=0; j < 6; j++){
          for(int i=0; i < 6; i++){
              params.P[ciao] = P_limit(i,j);
              ciao++;
          }
      }

    }
    else {
      params.l[0] = 0.0;

      int hello = 0;
      for(int j=0; j < 6; j++){
          for(int i=0; i < 6; i++){
              params.P[hello] = P_zero(i,j);
              hello++;
          }
      }

    }

    // insert the Augmented Jacobian
    int n = 0;
    for(int j=0; j < jacobian_augmented.cols(); j++){
        for(int i=0; i < jacobian_augmented.rows(); i++){
            params.Jacob[n] = jacobian_augmented(i,j);
            n++;
        }
    }

    // insert the Augmented Jacobian with the last 2 columns(regarding wheels velocities) equal to zero
    Eigen::MatrixXd jacobiano_modificato(6,8);
    jacobiano_modificato.setZero();
    jacobiano_modificato.topLeftCorner(6,6) = jacobian_augmented.topLeftCorner(6,6);
    //jacobiano_modificato << jacobian_augmented.topLeftCorner(6,6), jacobiano_nullo_mir;
    ROS_WARN_STREAM("Jacobiano modificato: \n" << jacobiano_modificato << "\n");

    int c = 0;
    for(int j=0; j < jacobiano_modificato.cols(); j++){
        for(int i=0; i < jacobiano_modificato.rows(); i++){
            params.Jacob_limit[c] = jacobiano_modificato(i,j);
            c++;
        }
    }

    // acceleration constrain
    for(int i = 0; i< acc_limits.rows(); i++)
        params.a_max[i] = acc_limits(i) * cycle_time;

    for(int i=0; i<opt_qdot_prev.rows() ; i++)
        params.opt_qdot_prev[i] = opt_qdot_prev(i);


    settings.verbose = 0;
    settings.max_iters = 50;
    long num_iters = solve();

    for(int i = 0; i < 6; i++){
        if(isnan(vars.opt_qdot[i])){
            vars.opt_qdot[i] = 0.0;
        ROS_ERROR_STREAM("OUTPUT IN USCITA DALL'OTTIMIZATORE NON VALIDO AIUTO");
        }
    }


    Vector8d q_dot_opt;
    Vector6d x_dot_opt;

    //for the acceleration constrain
    for(int i = 0; i< q_dot_opt.rows(); i++){
        q_dot_opt(i) = vars.opt_qdot[i];
        opt_qdot_prev(i) = vars.opt_qdot[i];
    }

    for(int i = 0; i<8; i++){
        ROS_WARN_STREAM("DOTQ_opt" << i  <<": " << vars.opt_qdot[i]);
        if((vars.opt_qdot[i] > 1.58) || (vars.opt_qdot[i] < -1.58))
            ROS_ERROR_STREAM("VAI TROPPO VELOCE OHHH CALMA!!!!!!!");
    }

    x_dot_opt = jacobian_augmented * q_dot_opt;

    ROS_INFO_STREAM("X ADM POST OPT: " << x_dot_opt(0));
    ROS_INFO_STREAM("Y ADM POST OPT: " << x_dot_opt(1));
    ROS_INFO_STREAM("Z ADM POST OPT: " << x_dot_opt(2));
    //#################################################################################################






    //######################### UPDATE THE TANK VARIABLES #############################//

    Eigen::VectorXd modulation_matrix = x_dot_opt / tank_state_;  //! VERIFY IF FUNZIONA

    double delta_update = (pow(cycle_time, 2)/ (2 * pow(tank_state_, 2))) *
                          (pow((F_ext).transpose() * x_dot_opt, 2));

    sum_of_delta_ += delta_update;
    tank_energy_ += cycle_time * (F_ext).transpose() * x_dot_opt + delta_update;// - 0.5 * x_dot.transpose() * dot_M_ * x_dot; // NOT REALLY HOW IT SHOULD BE DONE BUT STILL
    //float dot_M_tank_update = 0.5 * x_dot.transpose() * dot_M_ * modulation_matrix;
    tank_state_ += cycle_time * modulation_matrix.transpose() * (F_ext);// - dot_M_tank_update;

    if(tank_state_ >= 2 * sqrt(TANK_MAX_VALUE))
       tank_state_ = 2 * sqrt(TANK_MAX_VALUE);

    // non ci dovrebbe mai entrare qui o si ? ... può essere negativo lo stato ?
    if(tank_state_ <= -2 * sqrt(TANK_MAX_VALUE))
       tank_state_ = -2 * sqrt(TANK_MAX_VALUE);

    if(tank_energy_ >= TANK_MAX_VALUE)
       tank_energy_ = TANK_MAX_VALUE;

    if(tank_state_ <= 0.0)
       ROS_ERROR_STREAM("TANK EMPTY!");

    ROS_INFO_STREAM("TANK STATE: " << tank_state_);
    ROS_INFO_STREAM("TANK ENERGY: " << tank_energy_);


    energia.data = float(tank_energy_);
    tank_node.publish(energia);

  //############################## SAVE DATA ####################################//

    //generate_csv(adm_qdot, "Adm_qdot");
    double time_now = ros::Time::now().toSec() - start_time_;
    q_dot_file << time_now;
    q_dot_SNS_file << time_now;
    force_file << time_now;
    q_dot_opt_file << time_now;
    tank_file << time_now;

    for(int i = 0; i < 8; i++)
        q_dot_file << " " << adm_q_dot_scaling(i);


    for(int i = 0; i < 8; i++)
        q_dot_SNS_file << " " << adm_qdot(i);


    for(int i = 0; i < 6; i++)
        force_file << " " << F_ext(i);


    for(int i = 0; i < 8; i++)
        q_dot_opt_file << " " << q_dot_opt(i);



    tank_file << " " << tank_state_ << " " << tank_energy_;

//    tank_file << " " << tank_state_ << " " << tank_energy_ << " " << d2;

    q_dot_file << std::endl;
    q_dot_SNS_file << std::endl;
    force_file << std::endl;
    q_dot_opt_file << std::endl;
    tank_file << std::endl;


    //########################################################################################

    ROS_INFO_STREAM("ADMISSIBLE Q DOT Limitati: \n" << adm_qdot << "\n");

    //le variabili di giunto del manipolatore sono solo le prime 6
    for(int i = 0; i < adm_qdot_arm.rows(); i++){
      adm_qdot_arm(i) = q_dot_opt(i);
    }

    //mentre le variabili di giunto della base sono le ultime 2
    for(int i = 6; i < q_dot_opt.rows(); i++){
      adm_qdot_base(i-6) = q_dot_opt(i);
    }


    ROS_INFO_STREAM("ADMISSIBLE Q DOT ARM: \n" << adm_qdot_arm << "\n");
    ROS_INFO_STREAM("ADMISSIBLE Q DOT BASE: \n" << adm_qdot_base << "\n");


    //###################### ARM ######################################

    send_velocity_to_robot(adm_qdot_arm);

    //###################### BASE #####################################//

    base_message.linear.x = adm_qdot_base(0);
    base_message.angular.z = adm_qdot_base(1);

    ROS_INFO_STREAM("Lineare: \n" << adm_qdot_base(0) << "\n");
    ROS_INFO_STREAM("Angolare: \n" << adm_qdot_base(1) << "\n");

    mobile_pub.publish(base_message);

    //################ Manipiulability Analisy #######################//

    // Matrix6d w_2;
    // w_2 = jacobian_manipulator * jacobian_manipulator.transpose();
    // double w = sqrt(w_2.determinant());
    // ROS_WARN_STREAM("Manipulability:\t\t" << w);

    // Eigen::FullPivLU<Eigen::MatrixXd> lu_sing_prova(jacobian_manipulator);
    // double r_cond_number = lu_sing_prova.rcond();

    // Eigen::MatrixXd Jacob_man_pinv;
    // Jacob_man_pinv = jacobian_manipulator.completeOrthogonalDecomposition().pseudoInverse();
    // double cond_number = Jacob_man_pinv.norm() * jacobian_manipulator.norm();

    // ROS_WARN_STREAM("Condition Number: \t" << cond_number << "\n");
    // ROS_WARN_STREAM("Inverse of Cond_Number: \t" << 1/cond_number << "\n");
    // ROS_WARN_STREAM("R Condition Number: \t" << r_cond_number << "\n");

    //##################### Print for Debug ###########################//

//    Vector6d Prova_Forze = get_ee_rotation_matrix(joint_real_position, joint_real_velocity) * F_ext;
    ROS_INFO_STREAM("Forza applicata: \n" << Force_world << "\n");

    //ROS_INFO_STREAM("posEE: \t" << xy_norm);
    ROS_INFO_STREAM("d1: \t" << d1);
//    ROS_INFO_STREAM("d2: \t" << d2);
    ROS_INFO_STREAM("h_1: \t" << h_1);
//    ROS_INFO_STREAM("h_2: \t" << h_2);
    ROS_INFO_STREAM("Prova Compilazione \t 03");

}


