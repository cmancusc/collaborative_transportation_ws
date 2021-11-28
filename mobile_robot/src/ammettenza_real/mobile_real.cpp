#include "ammettenza_real/mobile_real.h"

Controller_mobile_real::Controller_mobile_real(){

    odometry_sub = nh.subscribe("/odom_enc", 1, &Controller_mobile_real::OdometryCallback, this);
    mir_joint_sub = nh.subscribe("/mir_joint_states", 1, &Controller_mobile_real::RuoteCallback, this);
    joints_state_sub= nh.subscribe("/joint_states", 1, &Controller_mobile_real::JointStateCallback, this);    
    //joystick_sub = nh.subscribe("/joy", 1, &Controller_mobile_real::joystickCallback, this);
    force_sub = nh.subscribe("/wrench", 1, &Controller_mobile_real::ForceCallback, this);    
    mobile_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    R = 0.0625;
    L = 0.445208;
    cycle_time = 0.02;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    vel_x = 0.0;
    omega_z = 0.0;
    //IO-SFL lead value
    b_ = -0.8;

    F_ext.conservativeResize(6,1);
    F_ext << Eigen::MatrixXd::Zero(6,1);

    std::vector<double> M_tot_parameters;
    std::vector<double> D_tot_parameters;

    if (!nh.getParam("mass_platform", M_tot_parameters)) {
        ROS_ERROR("Couldn't retrieve the desired mass of the robot.");
    }

    if (!nh.getParam("damping_platform", D_tot_parameters)) {
        ROS_ERROR("Couldn't retrieve the desired damping of the robot.");
    }

    M_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(M_tot_parameters.data());
    D_tot = Eigen::Map<Eigen::Matrix<double, 6, 6> >(D_tot_parameters.data());


    // ---- MoveIt Robot Model ---- //
    robot_model_loader = robot_model_loader::RobotModelLoader ("robot_description");
    kinematic_model = robot_model_loader.getModel();
    kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    joint_names = joint_model_group->getJointModelNames();


}



void Controller_mobile_real::OdometryCallback(const nav_msgs::Odometry& msg){
  
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



void Controller_mobile_real::RuoteCallback(const sensor_msgs::JointState& msg){

    omega_l = msg.velocity[0];
    omega_r = msg.velocity[1];

}



void Controller_mobile_real::JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg){


  joint_state = *msg;

  // Ur10e Real Robot has Inverted Joints
  std::swap(joint_state.name[0], joint_state.name[2]);
  std::swap(joint_state.effort[0], joint_state.effort[2]);
  std::swap(joint_state.position[0], joint_state.position[2]);
  std::swap(joint_state.velocity[0], joint_state.velocity[2]);

  for (unsigned int i = 0; i < joint_state.position.size(); i++) {joint_real_position[i] = joint_state.position[i];}
  for (unsigned int i = 0; i < joint_state.velocity.size(); i++) {joint_real_velocity[i] = joint_state.velocity[i];}

}




//void Controller_mobile_real::joystickCallback(const sensor_msgs::Joy& msg){

    /*
    Guida:  msg.axes[0]; //levetta sx orizzontale --> Fx
            msg.axes[1]; //levetta sx verticale --> Fy
            msg.axes[4]; //levetta dx verticale --> Fz
            msg.axes[3]; //levetta dx orizzontale --> Mx
            msg.axes[6]; //analogico orizzontale --> My
            msg.axes[7]; //analogico verticale --> Mz
    */
/*
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
    else
        F_ext(5,0) = 0.0;

}
*/



void Controller_mobile_real::ForceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg){


  geometry_msgs::WrenchStamped force_sensor = *msg;

  F_ext(0,0) = force_sensor.wrench.force.x;
  F_ext(1,0) = force_sensor.wrench.force.y;
  F_ext(2,0) = force_sensor.wrench.force.z;
  F_ext(3,0) = force_sensor.wrench.torque.x;
  F_ext(4,0) = force_sensor.wrench.torque.y;
  F_ext(5,0) = force_sensor.wrench.torque.z;

/*
  F_ext(0,0) = 0.0;
  F_ext(1,0) = 0.0;
  F_ext(2,0) = 0.0;
  F_ext(3,0) = 0.0;
  F_ext(4,0) = 0.0;
  F_ext(5,0) = 1.0;
*/


    for(int i = 0; i < 3; i++){
        if((F_ext(i,0) > 0.0) && (F_ext(i,0) < 4.0))
            F_ext(i,0) = 0.0;

        else if((F_ext(i,0) < 0.0) && (F_ext(i,0) > - 4.0))
            F_ext(i,0) = 0.0;
    }

    for(int i = 3; i < F_ext.rows(); i++){
        if((F_ext(i,0) > 0.0) && (F_ext(i,0) < 0.5))
            F_ext(i,0) = 0.0;

        else if((F_ext(i,0) < 0.0) && (F_ext(i,0) > - 0.5))
            F_ext(i,0) = 0.0;
    }


}




Matrix62d Controller_mobile_real::compute_platform_jacobian(double rotation){
    

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



Matrix6d Controller_mobile_real::get_ee_rotation_matrix(double joint_position[], double joint_velocity[]) {

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
/*
    Eigen::Vector3d ee_trasl = end_effector_state.translation();
    
    Eigen::Matrix3d skewsymetric_ee_base;
    skewsymetric_ee_base <<      0.0,     -ee_trasl[2],  ee_trasl[1],
                             ee_trasl[2],      0.0,     -ee_trasl[0],
                            -ee_trasl[1],  ee_trasl[0],     0.0;
*/

    Eigen::Matrix3d Rot_ee_odom;
    Eigen::Matrix3d Rot_b_World;
    Eigen::Matrix3d Rot_Platform_b;
    Eigen::Matrix3d Rot_ArmBase_Platform;

    //double angle = -45*M_PI/180.0;
    double angle = 135*M_PI/180.0;
    
    Rot_ArmBase_Platform <<  cos(angle),  -sin(angle),   0.0,
                             sin(angle),   cos(angle),   0.0,
                             0.0,         0.0,   1.0;

    Rot_Platform_b = Eigen::Matrix3d::Identity(3,3);

    Rot_b_World << cos(yaw), -sin(yaw), 0.0,
                   sin(yaw),  cos(yaw), 0.0,
                        0.0,            0.0,      1.0;

    Rot_ee_odom = Rot_b_World * Rot_Platform_b * Rot_ArmBase_Platform * end_effector_state.rotation();

    rotation_matrix.topLeftCorner(3, 3) = Rot_ee_odom;
    rotation_matrix.bottomRightCorner(3, 3) = Rot_ee_odom;

    //Eigen::Vector3d euler_angles = end_effector_state.rotation().eulerAngles(0, 1, 2);
    //Eigen::Quaterniond rotation_quaternion(end_effector_state.rotation());

    return rotation_matrix;

}




Vector6d Controller_mobile_real::limit_cartesian_dynamics (Vector6d joint_velocity) {

    double max_vel = 1.0;
    double max_acc = 0.2;

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



Vector2d Controller_mobile_real::limit_wheels_dynamics(Vector2d wheel_velocity){

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



int Controller_mobile_real::sign (double num) {

    if (num >= 0) {return +1;}
    else {return -1;}
    
}



void Controller_mobile_real::Spinner(){
    
        
    Eigen::MatrixXd jacobian_base_mapped;
    jacobian_base_mapped = compute_platform_jacobian(yaw);
    ROS_INFO_STREAM("Jacobian della Piattaforma: \n" << jacobian_base_mapped << "\n");

    Matrix26d J_base_pinv;
    J_base_pinv = jacobian_base_mapped.completeOrthogonalDecomposition().pseudoInverse();
    ROS_INFO_STREAM("Jacobian Aumentato PSEUDO-INVERSO: \n" << J_base_pinv << "\n");
    ROS_INFO_STREAM("MASS: \n" << M_tot << "\nDAMPING: \n" << D_tot << "\n");
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

/*
    q_dot_base(0,0) = vel_x;
    q_dot_base(1,0) = omega_z;
*/
    ROS_INFO_STREAM("Velocità dei giunti: \n" << q_dot_base << "\n");
    
    twist = jacobian_base_mapped * q_dot_base;
    ROS_INFO_STREAM("Velocità cartesiana: \n" << twist << "\n");

    //adm_acceleration = M_tot.inverse() * ( - D_tot * twist +  F_ext);

    adm_acceleration = M_tot.inverse() * ( - D_tot * twist + get_ee_rotation_matrix(joint_real_position, joint_real_velocity) * F_ext);


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

    Vector6d Prova_Forze = get_ee_rotation_matrix(joint_real_position, joint_real_velocity) * F_ext;
    ROS_INFO_STREAM("Forza applicata: \n" << Prova_Forze << "\n");
    ROS_INFO_STREAM("Compilazione \tn. 06");
    mobile_pub.publish(base_message);

}

