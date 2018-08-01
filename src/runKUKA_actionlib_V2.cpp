/*
This is the base class for the robot plugin, which takes care of interfacin with the robot.
*/
// Headers.
#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include "agent_ros_robot/PositionCommand.h"
#include "agent_ros_robot/SampleResult.h"
#include "agent_ros_robot/DataRequest.h"
#include "agent_ros_robot/num.h"

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// Convenience defines.
#define NUMBER_OF_JOINTS  7
#define NUMBER_OF_FORCES  3
#ifndef PI
#define PI  3.1415926535897932384626433832795
#endif

// global variables.
std_msgs::Float32MultiArray arm_state_;
geometry_msgs::Vector3 arm_forces_;
bool position_command_waiting_;
std::vector<double> xyzYPR_command;

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* active_arm_controller_; //controller
//-------------------------------------------------------------------------------publish and subscriber function
//call position command.
void position_subscriber_callback(const agent_ros_robot::PositionCommand::ConstPtr& msg)
{
  xyzYPR_command = msg->position;
  position_command_waiting_ = true;
}
//call force.
void robotForce_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  arm_forces_ = msg->wrench.force;
}
//＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "actionlib_yuchen");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  /* Needed for ROS_INFO commands to work */
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //-----------------------------------------------------------------Create subscribers.
  fprintf(stdout, "Starting pub_sub...\n");
  ros::Subscriber position_subscriber_ = n.subscribe("/yuchen_controller_position_command", 1000, position_subscriber_callback);
  ros::Subscriber robotForce_subscriber_ = n.subscribe("/estimatedExternalWrench", 1000, robotForce_callback);
  // Create publishers.
  ros::Publisher report_publisher_ = n.advertise<agent_ros_robot::SampleResult>("/yuchen_controller_report", 1000);
  //--------------------------------------------------------------------Initialize position controllers.
   active_arm_controller_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("lwr_controller/follow_joint_trajectory", true);
   while(!active_arm_controller_->waitForServer(ros::Duration(1.0))){
     ROS_INFO("Waiting for the joint_trajectory_action action server to come up");
   }
  //-----------------------------------------------------------------------kinematics
  fprintf(stdout, "Starting kinematics...\n");
  robot_model_loader::RobotModelLoader robotModelLoader= robot_model_loader::RobotModelLoader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robotModelLoader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("arm");
  ROS_INFO( "Moveit: model frame: %s", kinematic_model->getModelFrame().c_str());

  //-----------------------------------------------------------------------control loop
  fprintf(stdout, "Starting control lopp...\n");
  bool data_request_waiting_ = true;
  position_command_waiting_ = false;
  while (ros::ok())
  {
    // publish sample to agent.
    if (data_request_waiting_) {
      // 1. update (xyzYPR).
      tf::Vector3 xyz;
      tf::Matrix3x3 YPR_matrix;
      double Y, P, R;
      // Forward Kinematics
      //forward kinematics for a joint values. Note that to find the pose of the “r_wrist_roll_link” which is the most distal link in the “right_arm” of the robot.
      const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("lwr_arm_7_link");
      tf::Transform tform_command_tmp = tf::Transform();
      tf::transformEigenToTF(end_effector_state, tform_command_tmp);
      xyz = tform_command_tmp.getOrigin();
      YPR_matrix = tform_command_tmp.getBasis();
      YPR_matrix.getEulerYPR(Y, P, R);
      // 2. update sample.position
      agent_ros_robot::SampleResult current_time_step_sample_;// Sensor data for the current time step.
      current_time_step_sample_.position.resize(6);
      current_time_step_sample_.position[0] = xyz.getX(); current_time_step_sample_.position[1] = xyz.getY();
      current_time_step_sample_.position[2] = xyz.getZ(); current_time_step_sample_.position[3] = Y;
      current_time_step_sample_.position[4] = P; current_time_step_sample_.position[5] = R;
      // 3. update sample.force.
      current_time_step_sample_.force.resize(3);
      current_time_step_sample_.force[0] = arm_forces_.x = 0.0;
      current_time_step_sample_.force[1] = arm_forces_.y = 0.0;
      current_time_step_sample_.force[2] = arm_forces_.z = 0.0;
      //send sample to agent.
      report_publisher_.publish(current_time_step_sample_);
      //ROS_INFO("-------runKUKA_actionlib: send sample to agent");
    }

    // position control.
    if (position_command_waiting_){
      // 1. start joint beforce control.
      std::vector<double> joint_values;
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for(std::size_t i = 0; i < NUMBER_OF_JOINTS; ++i)
      {
        ROS_INFO("start Joint before control: %f",  joint_values[i]);
      }
      // 2. command convert: xyzYPR --> Transform-->Eigen matrix
      tf::Transform tform_command_tmp = tf::Transform();
      Eigen::Affine3d eigne_command;
      tf::Matrix3x3 basis;
      basis.setEulerYPR( xyzYPR_command[3], xyzYPR_command[4], xyzYPR_command[5] );
      tform_command_tmp.setOrigin( tf::Vector3(xyzYPR_command[0], xyzYPR_command[1], xyzYPR_command[2]));
      tform_command_tmp.setBasis(basis);
      tf::transformTFToEigen(tform_command_tmp, eigne_command);
      // 3. Inverse Kinematics: Eigen matrix --> joints
      bool found_ik = kinematic_state->setFromIK(joint_model_group, eigne_command, 10, 0.1); // number of attempts = 10, timeout= 0.1
      // 4. execute.
      if (found_ik)
      {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i = 0; i < NUMBER_OF_JOINTS; ++i)
        {
          ROS_INFO("target Joint after control: %f", joint_values[i]);
        }
        kinematic_state->setJointGroupPositions( joint_model_group, joint_values);
        // execute.
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.joint_names.push_back("lwr_arm_0_joint");
        goal.trajectory.joint_names.push_back("lwr_arm_1_joint");
        goal.trajectory.joint_names.push_back("lwr_arm_2_joint");
        goal.trajectory.joint_names.push_back("lwr_arm_3_joint");
        goal.trajectory.joint_names.push_back("lwr_arm_4_joint");
        goal.trajectory.joint_names.push_back("lwr_arm_5_joint");
        goal.trajectory.joint_names.push_back("lwr_arm_6_joint");
        goal.trajectory.points.resize(1);
        for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
          goal.trajectory.points[0].positions.push_back(joint_values[i]);
          //goal.trajectory.points[0].velocities.push_back(0.0);
          //goal.trajectory.points[0].accelerations.push_back(0.0);
        }
        goal.trajectory.points[0].time_from_start = ros::Duration(0.1);
        active_arm_controller_->sendGoal(goal);
        ROS_INFO("------------actionLib: execute!");
        while(!active_arm_controller_->getState().isDone() && ros::ok())
        {
          ros::Duration(0.05).sleep();
        }
      }
      else
      {
        ROS_INFO("Did not find IK solution");
      }
      position_command_waiting_ = false;
    }

    ros::spinOnce();
    //ROS_INFO("-------runKUKA_actionlib: ros is ok");
    loop_rate.sleep();
  }
//ros::spin();
  exit( 0 );
}
