/*
This is the base class for the robot plugin, which takes care of interfacin with the robot.
*/
// Headers.
#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>

#include "agent_ros_robot/PositionCommand.h"
#include "agent_ros_robot/SampleResult.h"
#include "agent_ros_robot/DataRequest.h"
#include "agent_ros_robot/num.h"
#include <LWRJointPositionController.h>

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


boost::shared_ptr<LWRJointPositionController> active_arm_controller_; //controller

//robot_model::RobotModelPtr kinematic_model;// kinematic_model.
robot_state::RobotStatePtr kinematic_state;// kinematic_state.
//robot_model_loader::RobotModelLoader robotModelLoader;
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
  ROS_INFO_STREAM("----Received position command");
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
    float jointValues_command[NUMBER_OF_JOINTS];
    for (int i = 0; i < NUMBER_OF_JOINTS; i++)
    {
      jointValues_command[i] = joint_values[i];
    }
    active_arm_controller_->SetCommandedJointPositions(jointValues_command); // execute the params
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
void robotJoint_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
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
  ros::init(argc, argv, "agent_ros_robot_yuchen");
  ros::NodeHandle n;

  //------------------------------Create subscribers.
  fprintf(stdout, "Starting pub_sub...\n");
  ros::Subscriber position_subscriber_ = n.subscribe("/yuchen_controller_position_command", 1, position_subscriber_callback);
  ros::Subscriber test_sub_ = n.subscribe("/yuchen_test_sub", 1, test_callback);
  ros::Subscriber data_request_subscriber_ = n.subscribe("/yuchen_controller_data_request", 1, data_request_subscriber_callback);
  ros::Subscriber robotJoint_subscriber_ = n.subscribe("/lwr_measuredJointPositions", 1, robotJoint_callback);
  ros::Subscriber robotForce_subscriber_ = n.subscribe("/estimatedExternalWrench", 1, robotForce_callback);
  // Create publishers.
  ros::Publisher report_publisher_ = n.advertise<agent_ros_robot::SampleResult>("/yuchen_controller_report", 1);

  //-------------------------------Initialize position controllers.
  fprintf(stdout, "Starting fri controller...\n");
  int    ResultValue    =  0;
  active_arm_controller_.reset(new LWRJointPositionController("/home/yuchen/catkin_ws/src/RL_twolayer/agent_ros_robot/driver_init/980039-FRI-Driver.init"));
  ResultValue  =  active_arm_controller_->StartRobot();
  if (ResultValue == EOK)
  {
    fprintf(stdout, "Robot successfully started.\n");
  }
  else
  {
    fprintf(stderr, "ERROR, could not start robot: %s\n", strerror(ResultValue));
  }

  //----------------------------------kinematics
  fprintf(stdout, "Starting kinematics...\n");
  robot_model_loader::RobotModelLoader robotModelLoader= robot_model_loader::RobotModelLoader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robotModelLoader.getModel();
  ROS_INFO( "Moveit: model frame: %s", kinematic_model->getModelFrame().c_str());
  kinematic_state.reset(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

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
        data_request_waiting_ = false;
    }
    ros::spinOnce();
  }
  return 0;
}
