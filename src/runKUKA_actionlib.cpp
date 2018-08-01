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
std_msgs::Float32MultiArray arm_state_; // Temporary storage for active arm joint to be applied at each step.
geometry_msgs::Vector3 arm_forces_;
agent_ros_robot::SampleResult current_time_step_sample_;// Sensor data for the current time step.
bool data_request_waiting_; // Is a trial arm data request pending?

actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* active_arm_controller_; //controller
control_msgs::FollowJointTrajectoryGoal goal;

//robot_model::RobotModelPtr kinematic_model;
boost::shared_ptr<robot_state::RobotState> kinematic_state;
const robot_state::JointModelGroup *joint_model_group; //joint
//------------------------------------------------------------------------------function declaration
/*
void position_subscriber_callback(const agent_ros_robot::PositionCommand::ConstPtr& msg);
void data_request_subscriber_callback(const agent_ros_robot::DataRequest::ConstPtr& msg);
void test_callback(const std_msgs::Empty::ConstPtr& msg);
void robotJoint_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void robotForce_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
*/
//-------------------------------------------------------------------------------publish and subscriber function
//call position command.
void position_subscriber_callback(const agent_ros_robot::PositionCommand::ConstPtr& msg)
{
  ROS_INFO("-----runKUKA_actionlib: get position command");
  float x = msg->position[0]; float y = msg->position[1]; float z = msg->position[2];
  float Y = msg->position[3]; float P = msg->position[4]; float R = msg->position[5];

  // convert: xyzYPR --> Transform-->Eigen matrix
  tf::Transform tform_command_tmp = tf::Transform();
  Eigen::Affine3d eigne_command;
  tf::Matrix3x3 basis;
  basis.setEulerYPR( Y, P, R );
  tform_command_tmp.setOrigin( tf::Vector3(x, y, z));
  tform_command_tmp.setBasis(basis);
  tf::transformTFToEigen(tform_command_tmp, eigne_command);

  // Inverse Kinematics: Eigen matrix --> joints
  bool found_ik = kinematic_state->setFromIK(joint_model_group, eigne_command, 10, 0.1); // number of attempts = 10, timeout= 0.1
  std::vector<double> joint_values;
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    ROS_INFO("Joint: (%f %f %f %f %f %f %f)",
            joint_values[0],
            joint_values[1],
            joint_values[2],
            joint_values[3],
            joint_values[4],
            joint_values[5],
            joint_values[6]);
    // execute.
    goal.trajectory.points.resize(1);
    for(unsigned int i=0; i < 7; i++)
      goal.trajectory.points[0].positions.push_back(joint_values[i]);
      //goal.trajectory.points[0].velocities.push_back(0.0);
      //goal.trajectory.points[0].accelerations.push_back(0.0);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.0);
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
}
//call test.
void test_callback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO_STREAM("Received test message");
}
//call data request.
void data_request_subscriber_callback(const agent_ros_robot::DataRequest::ConstPtr& msg)
{
  ROS_INFO_STREAM("received data request");
  data_request_waiting_ = true;
}
//call robot joints
//void robotJoint_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
void robotJoint_callback(const sensor_msgs::JointState::ConstPtr msg)
{
  tf::Vector3 xyz;
  tf::Matrix3x3 YPR_matrix;
  double Y, P, R;
  // Forward Kinematics
  //set global coordinate.
  const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("lwr_arm_base_link");
  tf::Transform tform_command_tmp = tf::Transform();
  tf::transformEigenToTF(end_effector_state, tform_command_tmp);
  xyz = tform_command_tmp.getOrigin();
  YPR_matrix = tform_command_tmp.getBasis();
  YPR_matrix.getEulerYPR(Y, P, R);

  // update state
  arm_state_.data[0] = xyz.getX(); arm_state_.data[1] = xyz.getY(); arm_state_.data[2] = xyz.getZ();
  arm_state_.data[3] = Y; arm_state_.data[4] = P; arm_state_.data[5] = R;
}
//call force.
void robotForce_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  arm_forces_ = msg->wrench.force;
}

//＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊＊8＊＊＊＊main function
int main(int argc, char **argv)
{
  ros::init(argc, argv, "actionlib_yuchen");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  /* Needed for ROS_INFO commands to work */
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //------------------------------Create subscribers.
  fprintf(stdout, "Starting pub_sub...\n");
  ros::Subscriber position_subscriber_ = n.subscribe("/yuchen_controller_position_command", 1000, position_subscriber_callback);
  ros::Subscriber test_sub_ = n.subscribe("/yuchen_test_sub", 1000, test_callback);
  ros::Subscriber data_request_subscriber_ = n.subscribe("/yuchen_controller_data_request", 1000, data_request_subscriber_callback);
  //ros::Subscriber robotJoint_subscriber_ = n.subscribe("/lwr_measuredJointPositions", 1, robotJoint_callback);
  ros::Subscriber robotJoint_subscriber_ = n.subscribe("/joint_states", 1000, robotJoint_callback);
  ros::Subscriber robotForce_subscriber_ = n.subscribe("/estimatedExternalWrench", 1000, robotForce_callback);
  // Create publishers.
  ros::Publisher report_publisher_ = n.advertise<agent_ros_robot::SampleResult>("/yuchen_controller_report", 1000);

  //-------------------------------Initialize position controllers.
   active_arm_controller_ = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("lwr_controller/follow_joint_trajectory", true);
   while(!active_arm_controller_->waitForServer(ros::Duration(1.0))){
     ROS_INFO("Waiting for the joint_trajectory_action action server to come up");
   }
   //set joint name.
  goal.trajectory.joint_names.push_back("lwr_arm_0_joint");
  goal.trajectory.joint_names.push_back("lwr_arm_1_joint");
  goal.trajectory.joint_names.push_back("lwr_arm_2_joint");
  goal.trajectory.joint_names.push_back("lwr_arm_3_joint");
  goal.trajectory.joint_names.push_back("lwr_arm_4_joint");
  goal.trajectory.joint_names.push_back("lwr_arm_5_joint");
  goal.trajectory.joint_names.push_back("lwr_arm_6_joint");

  //----------------------------------kinematics
  fprintf(stdout, "Starting kinematics...\n");
  robot_model_loader::RobotModelLoader robotModelLoader= robot_model_loader::RobotModelLoader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robotModelLoader.getModel();
  ROS_INFO( "Moveit: model frame: %s", kinematic_model->getModelFrame().c_str());
  kinematic_state.reset(new robot_state::RobotState(kinematic_model));
  joint_model_group = kinematic_model->getJointModelGroup("arm");

  //---------------------------------control loop
  fprintf(stdout, "Starting control lopp...\n");

  while (ros::ok())
  {
    //update sample.
    if (data_request_waiting_)
    {
      // 1. add joints to sample.
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
          current_time_step_sample_.position[i] = arm_state_.data[i];
      }
      // 2. add force to sample.
      current_time_step_sample_.force[0] = arm_forces_.x;
      current_time_step_sample_.force[1] = arm_forces_.y;
      current_time_step_sample_.force[2] = arm_forces_.z;
      //send sample to agent.
      report_publisher_.publish(current_time_step_sample_);
      ROS_INFO("-------runKUKA_actionlib: send sample to agent");
      data_request_waiting_ = false;
    }
    ros::spinOnce();
    //ROS_INFO("-------runKUKA_actionlib: ros is ok");
    loop_rate.sleep();
  }
//ros::spin();
  exit( 0 );
}
