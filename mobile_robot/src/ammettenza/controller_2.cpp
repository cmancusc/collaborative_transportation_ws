#include "ammettenza/controller_2.h"

Controller_2::Controller_2(){


    odometry_sub = nh.subscribe("/base_pose_ground_truth", 1, &Controller_2::OdometryCallback, this);
    joints_state_sub= nh.subscribe("/joint_states", 1, &Controller_2::JointStateCallback, this);
    joystick_sub = nh.subscribe("/joy", 1, &Controller_2::joystickCallback, this);
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
    b_ = 0.8;

    twist.setZero();

    F_ext.conservativeResize(6,1);
    F_ext << Eigen::MatrixXd::Zero(6,1);

    for(int i = 0; i < 6; i++){
        joint_real_position[i] = 0.0;
        joint_real_velocity[i] = 0.0;
    }
    

    std::vector<double> M_tot_parameters;
    std::vector<double> D_tot_parameters;

    if (!nh.getParam("mass_robot", M_tot_parameters)) {
        ROS_ERROR("Couldn't retrieve the desired mass of the robot.");
    }

    if (!nh.getParam("damping_robot", D_tot_parameters)) {
        ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
    }

    M_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_tot_parameters.data());
    D_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_tot_parameters.data());

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



void Controller_2::OdometryCallback(const nav_msgs::Odometry& msg){
  

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
    z_platform = msg.pose.pose.position.z;

}



void Controller_2::joystickCallback(const sensor_msgs::Joy& msg){


    /*
    Guida:  msg.axes[0]; //levetta sx orizzontale --> Fx
            msg.axes[1]; //levetta sx verticale --> Fy
            msg.axes[4]; //levetta dx verticale --> Fz
            msg.axes[3]; //levetta dx orizzontale --> Mx
            msg.axes[6]; //analogico orizzontale --> My
            msg.axes[7]; //analogico verticale --> Mz
    */

    double intensity = 50;

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



void Controller_2::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){

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


Matrix4d Controller_2::compute_arm_fk(double joint_position[], double joint_velocity[]) {

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



Eigen::MatrixXd Controller_2::compute_arm_jacobian (double joint_position[], double joint_velocity[]) {

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



Matrix62d Controller_2::compute_platform_jacobian(double rotation){

  
    Matrix62d jacobian_platform;

/*
    double j11 = R*cos(rotation)/2 - b_*sin(rotation)*R/L;
    double j12 = R*cos(rotation)/2 + b_*sin(rotation)*R/L;
    double j21 = R*sin(rotation)/2 + b_*cos(rotation)*R/L;
    double j22 = R*sin(rotation)/2 - b_*cos(rotation)*R/L;

    jacobian_platform << j11, j12,
                         j21, j22,
                          0,   0, 
                          0,   0, 
                          0,   0, 
                         R/L, -R/L;
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
                        0.0, 1.0;

    return jacobian_platform;

}



void Controller_2::Kinematic_mapping(){

    
    Eigen::Matrix3d R_B_W;
    Eigen::Matrix3d R_b_B;
    Eigen::Matrix3d R_O_b;

    R_B_W << cos(yaw), -sin(yaw), 0,
             sin(yaw),  cos(yaw), 0,
                0,          0,    1;
    
    R_b_B = Eigen::Matrix3d::Identity(3,3);
    R_O_b = Eigen::Matrix3d::Identity(3,3);



    Vector3d r_O_B;
    Vector3d r_E_O;
    Vector3d r_b_B;
            
    r_O_B(0) = 0;
    r_O_B(1) = 0;
    r_O_B(2) = 0.354;

    r_E_O = compute_arm_fk(joint_real_position, joint_real_velocity).block<3,1>(0,3);
    r_E_O(2) = r_E_O(2) - 0.354;

    r_b_B(0) = b_;
    r_b_B(1) = 0;
    r_b_B(2) = 0;

    Vector3d W_r_b_E;
    W_r_b_E = R_B_W * (r_O_B + R_b_B * R_O_b * r_E_O - r_b_B);

    Eigen::Matrix3d W_r_b_E_skewsimmetric;
    W_r_b_E_skewsimmetric <<        0.0,      -W_r_b_E(2),   W_r_b_E(1),
                                W_r_b_E(2),       0.0,      -W_r_b_E(0),
                               -W_r_b_E(1),    W_r_b_E(0),       0.0;


    
    H_platform << Eigen::Matrix3d::Identity(3,3),
                  -W_r_b_E_skewsimmetric,
                  Eigen::Matrix3d::Zero(3,3),
                  Eigen::Matrix3d::Identity(3,3);

    ROS_INFO_STREAM("MAPPING PIATTAFORMA:\n" << H_platform << "\n");

/*
    Vector3d W_r_O_W;
    Vector3d r_platform_world;
    r_platform_world(0) = x_platform;
    r_platform_world(1) = y_platform;
    r_platform_world(2) = z_platform;

    W_r_O_W =  R_B_W * (r_O_B + r_b_B) + r_platform_world;

    Eigen::Matrix3d W_r_O_W_skewsimmetric;
    W_r_O_W_skewsimmetric <<        0.0,      -W_r_O_W(2),   W_r_O_W(1),
                                W_r_O_W(2),       0.0,      -W_r_O_W(0),
                               -W_r_O_W(1),    W_r_O_W(0),       0.0;
*/
    H_arm << R_B_W * R_b_B * R_O_b,
             Eigen::Matrix3d::Zero(3,3),
             Eigen::Matrix3d::Zero(3,3),
             R_B_W * R_b_B * R_O_b;

    ROS_INFO_STREAM("MAPPING MANIPOLATORE:\n" << H_arm << "\n");

}




void Controller_2::Compute_Transformation(){

        
    //----------------------------------------------------------------------------------//
    //------- FIND THE SPACIAL TRANSFORMATION MANIPULATOR BASE --> PLATFORM_BASE -------//
    //----------------------------------------------------------------------------------//

    Eigen::Matrix3d R_ee_platform;
    R_ee_platform = Eigen::MatrixXd::Identity(3,3);
        
    ROS_INFO_STREAM("Matrice di rotazione: \n" << R_ee_platform << "\n");

    Eigen::Vector3d ee_platform_pose;
    ee_platform_pose[0] = 0.0;
    ee_platform_pose[1] = 0.0;
    ee_platform_pose[2] = 0.354;

    ROS_INFO_STREAM("X END EFFECTOR: " << ee_platform_pose[0] << "\n");
    ROS_INFO_STREAM("Y END EFFECTOR: " << ee_platform_pose[1] << "\n");
    ROS_INFO_STREAM("Z END EFFECTOR: " << ee_platform_pose[2] << "\n");

    Eigen::Matrix3d pos_ee_platform;

    pos_ee_platform <<          0.0,         -ee_platform_pose[2],  ee_platform_pose[1],
                        ee_platform_pose[2],            0.0,       -ee_platform_pose[0],
                        -ee_platform_pose[1],  ee_platform_pose[0],     0.0;

    ROS_INFO_STREAM("Matrice Antissimetrica: \n" << pos_ee_platform << "\n");


    Transform_ee_platform <<  R_ee_platform,
                                R_ee_platform * pos_ee_platform,
                                Eigen::Matrix3d::Zero(3,3),
                                R_ee_platform;

    ROS_INFO_STREAM("ADJOINT MATRIX EE-->PLATFORM: \n" << Transform_ee_platform << "\n");


    //------------------------------------------------------------------------------//
    //-- FIND THE SPACIAL TRANSFORMATION PLATFORM_BASE --> IOSFL LEAD POINT (b_) ---//
    //------------------------------------------------------------------------------//
    
    Eigen::Matrix3d R_platform_b;
    R_platform_b = Eigen::MatrixXd::Identity(3,3);
    
    Eigen::Vector3d platform_b_pose;
    platform_b_pose[0] = -b_;
    platform_b_pose[1] = 0.0;
    platform_b_pose[2] = 0.0;

    ROS_INFO_STREAM("X PLATFORM: " << platform_b_pose[0] << "\n");
    ROS_INFO_STREAM("Y PLATFORM: " << platform_b_pose[1] << "\n");
    ROS_INFO_STREAM("Z PLATFORM: " << platform_b_pose[2] << "\n");

    Eigen::Matrix3d pos_platform_b;
    pos_platform_b <<         0.0,         -platform_b_pose[2],  platform_b_pose[1],
                        platform_b_pose[2],         0.0,         -platform_b_pose[0],
                        -platform_b_pose[1],  platform_b_pose[0],         0.0;


    Transform_platform_b <<  R_platform_b,
                            R_platform_b * pos_platform_b,
                            Eigen::Matrix3d::Zero(3,3),
                            R_platform_b;
    
    ROS_INFO_STREAM("ADJOINT MATRIX PLATFORM-->IOSFL (b): \n" << Transform_platform_b << "\n");


    //------------------------------------------------------------------------------//
    //---- FIND THE SPACIAL TRANSFORMATION IOSFL LEAD POINT (b_) --> ODOM_COMB -----//
    //------------------------------------------------------------------------------//

    Eigen::Matrix3d R_b_odom;
    R_b_odom << cos(yaw),  -sin(yaw),   0.0,
                sin(yaw),   cos(yaw),   0.0,
                    0.0,         0.0,   1.0;

    Eigen::Vector3d b_odom_pose;
    b_odom_pose[0] = x_platform + b_ * cos(yaw);
    b_odom_pose[1] = y_platform + b_ * sin(yaw);
    b_odom_pose[2] = z_platform;

    ROS_INFO_STREAM("X PLATFORM: " << b_odom_pose[0] << "\n");
    ROS_INFO_STREAM("Y PLATFORM: " << b_odom_pose[1] << "\n");
    ROS_INFO_STREAM("Z PLATFORM: " << b_odom_pose[2] << "\n");

    Eigen::Matrix3d pos_b_odom;
    
    pos_b_odom <<        0,        -b_odom_pose[2],  b_odom_pose[1],
                    b_odom_pose[2],        0,        -b_odom_pose[0],
                    -b_odom_pose[1],  b_odom_pose[0],        0;

    ROS_INFO_STREAM("Matrice Antissimetrica: \n" << pos_b_odom << "\n");

    Transform_b_odom <<  R_b_odom,
                        R_b_odom * pos_b_odom,
                        Eigen::Matrix3d::Zero(3,3),
                        R_b_odom;

    ROS_INFO_STREAM("ADJOINT MATRIX IOSFL (b)-->ODOM_COMB: \n" << Transform_b_odom << "\n");


    //-------------------------------------------------------------------------------//
    //---------------------------------- PLATFORM -----------------------------------//
    // Extract the pose of the end-effector with respect to the platform base frame  //
    // from the transformation matrix --> contained in the 4th column, so in the 3rd //
    // position this is needed to compute the mapping matrix H (ee_mapping matrix)   //
    //-------------------------------------------------------------------------------//

    Matrix4d T_O_B(4,4);
    T_O_B <<    1.0, 0.0, 0.0,  0.0,
                0.0, 1.0, 0.0,  0.0,
                0.0, 0.0, 1.0,  0.354,        //altezza della base del ur10e rispetto al frame della base
                0.0, 0.0, 0.0,  1.0;

    Matrix4d T_B_b(4,4);
    T_B_b <<    1.0, 0.0, 0.0,  -b_,
                0.0, 1.0, 0.0,  0.0,
                0.0, 0.0, 1.0,  0.0,        
                0.0, 0.0, 0.0,  1.0;

    Matrix4d T_b_odom(4,4);
    T_b_odom << cos(yaw),  -sin(yaw),   0.0,   x_platform + b_ * cos(yaw),
                sin(yaw),   cos(yaw),   0.0,   y_platform + b_ * sin(yaw),
                    0.0,        0.0,      1.0,             0.0,
                    0.0,        0.0,      0.0,             1.0;


    /*
    Eigen::VectorXd prova(4);
    prova << ee_pose_kdl, 1.0;
    ROS_INFO_STREAM("EE_DISTANCE: \n" << prova);
    */

    /*
    Eigen::VectorXd prova(4);
    Matrix4d prova_2;
    prova_2 = compute_arm_fk(joint_real_position, joint_real_velocity);
    prova = compute_arm_fk(joint_real_position, joint_real_velocity).block<3,1>(0,3);

    ROS_INFO_STREAM("Matrice trasformazione Manipolatore: \n" << prova_2);
    ROS_INFO_STREAM("EE_DISTANCE: \n" << prova);
    */

    // nel modello unico in simulazione, movit misura la distanza dell'end-effector
    // dal frame della piattaforma, quidni considera già l'altezza in z pari a 0.354

    //Matrix4d T_E_b = T_B_b * T_O_B * compute_arm_fk(joint_real_position, joint_real_velocity);
    //Matrix4d T_E_b = T_B_b * compute_arm_fk(joint_real_position, joint_real_velocity);
    Matrix4d T_E_odom = T_b_odom * T_B_b * compute_arm_fk(joint_real_position, joint_real_velocity);


    Eigen::Vector3d ee_b_pose;
    ee_b_pose = T_E_odom.block<3,1>(0,3);

    //ee_b_pose = R_b_odom * ee_b_pose;

    
    ee_mapping_matrix <<  1.0, 0.0, 0.0, 0.0, 0.0, -ee_b_pose[1],
                            0.0, 1.0, 0.0, 0.0, 0.0, ee_b_pose[0],
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    
    /*
    Eigen::Matrix3d antisimmetric_pose;
    antisimmetric_pose <<       0,        -ee_pose_kdl[2],  ee_pose_kdl[1],
                            ee_pose_kdl[2],        0,        -ee_pose_kdl[0],
                        -ee_pose_kdl[1],  ee_pose_kdl[0],        0;



    ee_mapping_matrix.topLeftCorner(3,3) = Eigen::MatrixXd::Identity(3,3);
    ee_mapping_matrix.topRightCorner(3,3) = antisimmetric_pose; 
    ee_mapping_matrix.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    ee_mapping_matrix.bottomRightCorner(3,3) = Eigen::MatrixXd::Identity(3,3);
    */
    ROS_INFO_STREAM("LA MATRICE H: \n" << ee_mapping_matrix << "\n");

}



Vector6d Controller_2::limit_joints_dynamics (Vector6d joint_velocity) {

    double max_vel = 1.0;
    double max_acc = 0.5;
    

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



Vector2d Controller_2::limit_wheels_dynamics(Vector2d wheel_velocity){


    double max_vel[2];
    max_vel[0]= 1.0;
    max_vel[1]= 1.5;
    double max_acc[6];
    max_acc[0] = 2.0;
    max_acc[1] = 2.5;
    Vector2d cartesian_speed;

    cartesian_speed[0] = R / 2 * (wheel_velocity[0] + wheel_velocity[1]);
    cartesian_speed[1] = R / L * (wheel_velocity[0] - wheel_velocity[1]);


    for (int i = 0; i < cartesian_speed.size(); i++) {
        if (fabs(cartesian_speed[i]) > max_vel[i]) {       
          cartesian_speed[i] = sign(cartesian_speed[i]) * max_vel[i];
        }
    }

    // Limit Joint Acceleration

    for (int i = 0; i < cartesian_speed.size(); i++) {
        if (fabs(cartesian_speed[i] - adm_twist_last_cycle[i]) > max_acc[i] * cycle_time) {
            cartesian_speed[i] = adm_twist_last_cycle[i] + sign(cartesian_speed[i] - adm_twist_last_cycle[i]) * max_acc[i] * cycle_time;
        }
    }

    adm_wheels_last_cycle = cartesian_speed;


    return cartesian_speed;
}



int Controller_2::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}



void Controller_2::Spinner(){

    
    //Compute_Transformation();
    Kinematic_mapping();
    //ROS_INFO_STREAM("LA TRASFORMAZIONE SPAZIALE: \n" << Transform_ee_platform << "\n");

    Matrix6d jacobian_manipulator;
    jacobian_manipulator = H_arm * compute_arm_jacobian(joint_real_position, joint_real_velocity);
    //jacobian_manipulator = Transform_b_odom * Transform_platform_b * Transform_ee_platform * compute_arm_jacobian(joint_real_position, joint_real_velocity);
    //jacobian_manipulator = Transform_b_odom * Transform_platform_b * Transform_ee_platform * compute_arm_jacobian(joint_real_position, joint_real_velocity);    
    //jacobian_manipulator = Transform_platform_odom * jacobian_arm_kdl;
    //jacobian_manipulator = jacobian_arm_kdl;
    ROS_INFO_STREAM("Jacobian del Braccio Complessivo: \n" << jacobian_manipulator << "\n");


    Eigen::MatrixXd jacobian_base_mapped;
    jacobian_base_mapped = H_platform * compute_platform_jacobian(yaw);
    //jacobian_base_mapped = ee_mapping_matrix * compute_platform_jacobian(yaw);
    //jacobian_base_mapped = compute_platform_jacobian(yaw);    
    //jacobian_base_mapped = jacobian_base;
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

    q_dot_base(0,0) = omega_l;
    q_dot_base(1,0) = omega_r;

    ROS_INFO_STREAM("Velocità dei giunti delle ruote: \n" << q_dot_base << "\n");

    q_dot << q_dot_arm, q_dot_base;
    //twist = jacobian_augmented * q_dot;

    adm_acceleration = M_tot.inverse() * ( - D_tot * twist + F_ext );


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

    Eigen::MatrixXd adm_qdot(8,1);
    Eigen::MatrixXd adm_qdot_arm(6,1);
    Eigen::MatrixXd adm_qdot_base(2,1);
    Eigen::MatrixXd delta_q(6,1);
    Eigen::MatrixXd adm_twist_base(6,1);

    //twist = adm_twist;
    adm_qdot = J_pinv * adm_twist;
    twist = adm_twist;
    ROS_INFO_STREAM("ADMISSIBLE Q DOT: \n" << adm_qdot << "\n");

    //le variabili di giunto del manipolatore sono solo le prime 6
    for(int i = 0; i < adm_qdot_arm.rows(); i++){
      adm_qdot_arm(i, 0) = adm_qdot(i,0);
    }    

    //mentre le variabili di giunto della base sono le ultime 2  
    for(int i = 6; i < adm_qdot.rows(); i++){
      adm_qdot_base(i-6, 0) = adm_qdot(i,0);
    }


    adm_qdot_arm = limit_joints_dynamics(adm_qdot_arm);

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

    //q_arm(0,0) = 0.0;

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
    Vector2d vel_cartesiane = limit_wheels_dynamics(adm_qdot_base);


    base_message.linear.x = vel_cartesiane[0];
    base_message.angular.z = vel_cartesiane[1];


    ROS_INFO_STREAM("Lineare: \n" << vel_cartesiane[0] << "\n");
    ROS_INFO_STREAM("Angolare: \n" << vel_cartesiane[1] << "\n");
    ROS_INFO_STREAM("Omega sinistro: \n" << adm_qdot_base(0,0) << "\n");
    ROS_INFO_STREAM("Omega destro: \n" << adm_qdot_base(1,0) << "\n");


    mobile_pub.publish(base_message);
   
}

