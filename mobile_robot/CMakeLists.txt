cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs
  geometry_msgs
  sensor_msgs
# interactive_markers
  moveit_core
#  moveit_visual_tools
# moveit_ros_planning
  moveit_ros_planning_interface
  orocos_kdl
#  moveit_ros_perception
#  pluginlib
#  geometric_shapes
#  pcl_ros
#  pcl_conversions
#  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  #message_generation
  #andrea_filters
)


find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  INCLUDE_DIRS include
  cvxgen
  #MATLAB_TANK

  LIBRARIES mobile_robot
  libreria
  #matlab_opt_tank

  CATKIN_DEPENDS 
  		roscpp
  		std_msgs 
  		trajectory_msgs 
  		geometry_msgs
  		sensor_msgs
  	  moveit_core
    	moveit_ros_planning_interface
   		tf2_geometry_msgs
#  DEPENDS system_lib
  DEPENDS
    	EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  cvxgen
  MATLAB_TANK
  MATLAB_TANK_CBF
  MATLAB_4_CBF
  ${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)


file(GLOB matlab_opt_tank
  include/MATLAB_TANK/*.cpp
  include/MATLAB_TANK/*.h
)

file(GLOB matlab_opt_tank_cbf
  include/MATLAB_TANK_CBF/*.cpp
  include/MATLAB_TANK_CBF/*.h
)


file(GLOB matlab_opt_4_cbf
  include/MATLAB_4_CBF/*.cpp
  include/MATLAB_4_CBF/*.h
)

add_library(
  matlab_opt_tank
  ${matlab_opt_tank}
)

add_library(
  matlab_opt_tank_cbf
  ${matlab_opt_tank_cbf}
)

add_library(
  matlab_opt_4_cbf
  ${matlab_opt_4_cbf}
)

add_library(libreria
  cvxgen/ldl.c
  cvxgen/solver.c
  cvxgen/matrix_support.c
  cvxgen/util.c
  cvxgen/variables_definition.c
)




add_executable(joint_publisher src/joint_publisher.cpp)
target_link_libraries(joint_publisher ${catkin_LIBRARIES})

add_executable(prova_Kalman src/prova_Kalman.cpp)
target_link_libraries(prova_Kalman ${catkin_LIBRARIES})

add_executable(mobile_publisher src/mobile_publisher.cpp)
target_link_libraries(mobile_publisher ${catkin_LIBRARIES})

add_executable(joint_1 src/joint_1.cpp)
target_link_libraries(joint_1 ${catkin_LIBRARIES})

add_executable(joint_2 src/joint_2.cpp)
target_link_libraries(joint_2 ${catkin_LIBRARIES})

add_executable(mobile_subscriber src/mobile_subscriber.cpp)
target_link_libraries(mobile_subscriber ${catkin_LIBRARIES})

add_executable(joystick_node src/joystick_node.cpp)
target_link_libraries(joystick_node ${catkin_LIBRARIES})
#add_dependencies(joystick_node mobile_robot_generate_messages_cpp)

add_executable(tf_to_odom src/tf_to_odom.cpp)
target_link_libraries(tf_to_odom ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(joint_prova_node 
              src/joint_prova_node.cpp
              src/joint_prova.cpp)
target_link_libraries(joint_prova_node ${catkin_LIBRARIES})
add_dependencies(joint_prova_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(controller_node 
              src/ammettenza/controller_node.cpp
              src/ammettenza/controller.cpp)
target_link_libraries(controller_node ${catkin_LIBRARIES})
add_dependencies(controller_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(controller_arm_node 
              src/ammettenza/controller_arm_node.cpp
              src/ammettenza/controller_arm.cpp)
target_link_libraries(controller_arm_node ${catkin_LIBRARIES})
add_dependencies(controller_arm_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(controller_mobile_node 
              src/ammettenza/controller_mobile_node.cpp
              src/ammettenza/controller_mobile.cpp)
target_link_libraries(controller_mobile_node ${catkin_LIBRARIES})
add_dependencies(controller_mobile_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(arm_real_node 
              src/ammettenza_real/arm_real_node.cpp
              src/ammettenza_real/arm_real.cpp)
target_link_libraries(arm_real_node ${catkin_LIBRARIES})
add_dependencies(arm_real_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(mobile_real_node 
              src/ammettenza_real/mobile_real_node.cpp
              src/ammettenza_real/mobile_real.cpp)
target_link_libraries(mobile_real_node ${catkin_LIBRARIES})
add_dependencies(mobile_real_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(robot_real_node 
              src/ammettenza_real/robot_real_node.cpp
              src/ammettenza_real/robot_real.cpp)
target_link_libraries(robot_real_node 
                      ${catkin_LIBRARIES}
                      libreria)
add_dependencies(robot_real_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#set_target_properties(robot_real_node PROPERTIES LINKER_LANGUAGE CXX)



add_executable(collaborative_transportation_node 
              src/ammettenza_real/collaborative_transportation_node.cpp
              src/ammettenza_real/collaborative_transportation.cpp)
target_link_libraries(collaborative_transportation_node 
                      ${catkin_LIBRARIES}
                      matlab_opt_tank)
add_dependencies(collaborative_transportation_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
set_target_properties(collaborative_transportation_node PROPERTIES LINKER_LANGUAGE CXX)



add_executable(controller_2_node 
              src/ammettenza/controller_2_node.cpp
              src/ammettenza/controller_2.cpp)
target_link_libraries(controller_2_node ${catkin_LIBRARIES})
add_dependencies(controller_2_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(controllerSNS_node 
              src/ammettenza/controllerSNS_node.cpp
              src/ammettenza/controllerSNS.cpp)
target_link_libraries(controllerSNS_node ${catkin_LIBRARIES})
add_dependencies(controllerSNS_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(admittance_node 
              src/ammettenza/admittance_node.cpp
              src/ammettenza/admittance.cpp)
              target_link_libraries(admittance_node 
              ${catkin_LIBRARIES}
              libreria)
add_dependencies(admittance_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(admittance_tank_node 
			   src/ammettenza/admittance_tank_node.cpp
               src/ammettenza/admittance_tank.cpp)
target_link_libraries(admittance_tank_node 
					  ${catkin_LIBRARIES}
					  matlab_opt_tank)
add_dependencies(admittance_tank_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
set_target_properties(admittance_tank_node PROPERTIES LINKER_LANGUAGE CXX)


add_executable(admittance_tank_cbf_node 
			   src/ammettenza/admittance_tank_cbf_node.cpp
               src/ammettenza/admittance_tank_cbf.cpp)
target_link_libraries(admittance_tank_cbf_node 
					  ${catkin_LIBRARIES}
					  matlab_opt_tank_cbf)
add_dependencies(admittance_tank_cbf_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
set_target_properties(admittance_tank_cbf_node PROPERTIES LINKER_LANGUAGE CXX)


add_executable(sensore_Forza src/sensore_Forza.cpp)
target_link_libraries(sensore_Forza ${catkin_LIBRARIES})
