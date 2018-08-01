/*
1. to realize cartesian position control:  cartesian position -- IK--joint position

this file aim to: obtain control command from agent, run the kuka robot
History:
2017.05.20 - zhen deng
*/
#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <csignal>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>

// KuKA LWR reflexxes
#include <ros_fri_msgs/RMLVelocityInputParameters.h>
#include <ros_fri_msgs/RMLPositionInputParameters.h>

// Schunk WSG-50 gripper
#include <wsg_50_common/Move.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Eigen>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

//RL agent.
#include "agent_ros_robot/PositionCommand.h"
#include "agent_ros_robot/SampleResult.h"

//gripper
#include "ros/ros.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Status.h"
#include "agent_ros_robot/GripperCommand.h"


#define  NUMBER_OF_JOINTS                        7
#define  NUMBER_OF_CARTESIAN                     6
#ifndef PI
#define PI  3.1415926535897932384626433832795
#endif

#define MAX_DATA_POINTS 5

pthread_mutex_t nodeStateMutex;
// global variables
std::vector<std::string>    jointNames;       // lwr_arm_0_joint, ...
std::map<std::string, int>  jointIndexMap;   // joint name -> array index
std::vector<double>         jointAngles;      // current robot joint angles, radians
std::vector<double>         jointVelocities;  // radians per second
std::vector<double>         jointTorques;     // Newton-meters
std::vector<double>         zeroVelocities; // rad/sec
std::vector<double>         targetAngles;     // where the robot should be
std::vector<double>         jointVelocityLimits;     // rad/sec
std::vector<double>         jointAccelerationLimits; // rad/sec^2
bool                        position_command_waiting_;
std::vector<double>         xyzPYR_command;

ros::ServiceClient          gripper_grasp_client;//gripper control.
ros::ServiceClient          gripper_release_client;
ros::ServiceClient          gripper_homing_client;
wsg_50_common::Move         srv_grasp;
wsg_50_common::Move         srv_release;
wsg_50_common::Move         srv_homing;
float                       wsg_width;
float                       wsg_force;
bool                        Gripper_command_waiting_;
float                       gripper_width;
int                         gripper_mode;

// function declare.
void jointStateCallback(const sensor_msgs::JointState jointStateMessage);
void printTransform_tf(std::string prefix, tf::Transform trafo);
void printTransform_eigen(std::string prefix, Eigen::Affine3d affine3d);
bool xyzRpyToTransform( tf::Transform* trafo, std::string tokens ); // x y z r p y -> transform
void parseInitializationValues( std::vector<double> &data, std::string s );
void positionCommand_callback(const agent_ros_robot::PositionCommand::ConstPtr& msg);
void gripperCallback(const wsg_50_common::Status status);
void GripperCommand_callback(const agent_ros_robot::GripperCommand::ConstPtr& msg);

// callback function.
void jointStateCallback(const sensor_msgs::JointState jointStateMessage)
{
  //pthread_mutex_lock( &nodeStateMutex );
  jointAngles = jointStateMessage.position;
  jointVelocities = jointStateMessage.velocity;
  jointTorques = jointStateMessage.effort;
}

//call position command.
void positionCommand_callback(const agent_ros_robot::PositionCommand::ConstPtr& msg)
{
  xyzPYR_command = msg->position;
  position_command_waiting_ = true;
}

void gripperCallback(const wsg_50_common::Status status){
    wsg_width = status.width;
    wsg_force = status.force;
}
//call gripper command.
void GripperCommand_callback(const agent_ros_robot::GripperCommand::ConstPtr& msg)
{
  gripper_width = msg->width;
  gripper_mode = msg->mode;//0-grasp, 1-release
  Gripper_command_waiting_ = true;
}

// some utility function.
void printTransform_tf(std::string prefix, tf::Transform trafo)
{
  tfScalar roll, pitch, yaw;
  trafo.getBasis().getEulerYPR( yaw, pitch, roll );
  ROS_INFO( "%s: origin: %7.4lf %7.4lf %7.4lf quat: %7.4lf %7.4lf %7.4lf %7.4lf rpy: %6.3lf %6.3lf %6.3lf",
            prefix.c_str(),
            trafo.getOrigin().x(),
            trafo.getOrigin().y(),
            trafo.getOrigin().z(),
            trafo.getRotation().x(),
            trafo.getRotation().y(),
            trafo.getRotation().z(),
            trafo.getRotation().w(),
            roll, pitch, yaw );
}

void printTransform_eigen(std::string prefix, Eigen::Affine3d affine3d)
{
  tf::Transform trafo;
  tf::transformEigenToTF( affine3d, trafo );
  printTransform_tf( prefix, trafo );
}

// intialize the given transfrom from a string with six double,namely x y z roll pitch yaw (in meters and radians).
//Allocates the transform if NULL is given as input.Returns true on success, false on failure.
bool xyzRpyToTransform( tf::Transform* trafo, std::string tokens ) // x y z r p y -> transform
{
  if (trafo == NULL) trafo = new tf::Transform();

  double x, y, z, r, p, Y;
  if (6 == sscanf(tokens.c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r, &p, &Y))
  {
    tf::Matrix3x3 basis;
    basis.setEulerYPR( Y, p, r );
    trafo->setOrigin( tf::Vector3(x, y, z));
    trafo->setBasis( basis );
    return true;
  }
  else
  {
    fprintf( stderr, "Failed to create transform from given xyz rpy string '%s'", tokens.c_str());
    return false;
  }
}

//utility method to parse seven double values from a given param string.
void parseInitializationValues( std::vector<double> &data, std::string s )
{
  assert( NUMBER_OF_JOINTS == 7 );
  double a, b, c, d, e, f, g;
  int n = sscanf( s.c_str(),   "%lf %lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f, &g );
  if (n != NUMBER_OF_JOINTS) {
    ROS_ERROR( "Failed to parse initialization data '%s', found %d tokens.",
               s.c_str(), n );
    exit( 1 );
  }
  data.resize( NUMBER_OF_JOINTS );
  data[0] = a;
  data[1] = b;
  data[2] = c;
  data[3] = d;
  data[4] = e;
  data[5] = f;
  data[6] = g;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++ main fucntion.

int main(int argc, char ** argv) {
  // node state and mutex

  ros::init(argc, argv, "runKUKA_fri_control_yuchen", 1 );
  ros::NodeHandle nh;
  ros::Rate loop_rate_other(0.5);
  ros::Rate loop_rate_control(20); // keep 2Hz for control loop, I.e.sleep 0.5 seconds*(1/2)

  // those are the "standard" joint-names for the Kuka
  std::string tf_prefix;
  nh.param( "tf_prefix", tf_prefix, std::string( "" ));
  jointNames.push_back( tf_prefix + "lwr_arm_0_joint" );
  jointNames.push_back( tf_prefix + "lwr_arm_1_joint" );
  jointNames.push_back( tf_prefix + "lwr_arm_2_joint" );
  jointNames.push_back( tf_prefix + "lwr_arm_3_joint" );
  jointNames.push_back( tf_prefix + "lwr_arm_4_joint" );
  jointNames.push_back( tf_prefix + "lwr_arm_5_joint" );
  jointNames.push_back( tf_prefix + "lwr_arm_6_joint" );
  for ( unsigned int j = 0; j < jointNames.size(); j++ ) {
    jointIndexMap[jointNames[j]] = j;
  }

  //Initialize some variables
  jointAngles.resize(     NUMBER_OF_JOINTS );
  jointVelocities.resize( NUMBER_OF_JOINTS );
  jointTorques.resize(    NUMBER_OF_JOINTS );
  targetAngles.resize(NUMBER_OF_JOINTS ); // latest robot target position
  zeroVelocities.resize( NUMBER_OF_JOINTS );
  for ( int j = 0; j < NUMBER_OF_JOINTS; j++ ) zeroVelocities[j] = 0.0;
  jointVelocityLimits.resize( NUMBER_OF_JOINTS );
  jointAccelerationLimits.resize( NUMBER_OF_JOINTS );
  std::string velocity_limits, acceleration_limits;
  nh.param( "velocity_limits_radians", velocity_limits, // 55 deg/sec = 0.9599 rad/sec
             std::string( "1.9 1.9 2.25 2.25 2.25 3.14 3.14" ));
  parseInitializationValues( jointVelocityLimits, velocity_limits );
  nh.param( "acceleration_limits_radians", acceleration_limits, // 50 deg/sec^2 = 0.87 rad/sec/sec
             std::string( "0.3 0.3 0.5 0.5 0.5 0.5 0.5" ));
  parseInitializationValues( jointAccelerationLimits, acceleration_limits );
  wsg_width = 0.0; wsg_force = 0.0;

  // MoveIt robot model loader
  ROS_INFO( "Moveit: creating the RobotModelLoader now..." );
  robot_model_loader::RobotModelLoader robot_model_loader = robot_model_loader::RobotModelLoader( "robot_description" );
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  //ROS_INFO( "Moveit: model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState( kinematic_model));
  robot_state->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup( "arm" );

  // ROS publishers and subscribers
  //moveit-rviz display.
  ros::Publisher positionGoalPublisher = nh.advertise<sensor_msgs::JointState>( "joint_position_goal", 1 );
  ros::Publisher velocityGoalPublisher = nh.advertise<sensor_msgs::JointState>( "joint_velocity_goal", 1 );
  // extra topics for RML-style commands
  ros::Publisher rmlPositionGoalPublisher = nh.advertise<ros_fri_msgs::RMLPositionInputParameters>( "/lwr/jointPositionGoal", 1 );
  ros::Publisher rmlVelocityGoalPublisher = nh.advertise<ros_fri_msgs::RMLVelocityInputParameters>( "jointVelocityGoal", 1 );
  ros::Subscriber sub = nh.subscribe( "lwr/joint_states", 1, jointStateCallback );
  ros::Subscriber position_subscriber_ = nh.subscribe("/yuchen_controller_position_command", 1, positionCommand_callback);
  ros::Subscriber gripper_sub = nh.subscribe("wsg_50/status", 1, gripperCallback);
  ros::Subscriber gripper_subscriber_ = nh.subscribe("/yuchen_controller_gripper_command", 1, GripperCommand_callback);
  ROS_INFO("publisher, Subscriber: ---is finished");
  loop_rate_other.sleep(); // must need!--yuchen
  ros::spinOnce();
  for (unsigned int i = 0; i < jointNames.size(); ++i)
  {
    ROS_INFO("----------- before while loop %s: %f", jointNames[i].c_str(), jointAngles[i]);
  }

  //*************************cartesian position control***********************//
  ROS_INFO( "ready, waiting for *cartesian position control*" );
  bool first_flag = true;
  position_command_waiting_ = false;
  Gripper_command_waiting_ = false;
  while (ros::ok())
  {
    if (first_flag){ROS_INFO("----------the first time of while loop!!!");
      first_flag = false; ros::spinOnce();
    }
    else {
      if (position_command_waiting_) {
        // ........................................phase 2: joint position control.
        // step 1: target cartesian position
        double x, y, z, P, Y, R;
        x = xyzPYR_command[0]; y = xyzPYR_command[1]; z = xyzPYR_command[2];
        P = xyzPYR_command[3]; Y = xyzPYR_command[4]; R = xyzPYR_command[5];
        tf::Transform TransformGoal = tf::Transform();
        tf::Matrix3x3 basis;
        basis.setEulerYPR(Y, P, R);
        TransformGoal.setOrigin( tf::Vector3(x, y, z));
        TransformGoal.setBasis( basis );
        ROS_INFO("execute command: %f %f %f", x, y, z);

        // step 2: Ik
        Eigen::Affine3d updatedToolPose;
        tf::transformTFToEigen(TransformGoal, updatedToolPose); //tf --> eigen
        std::vector<double> ik_angles;ik_angles.resize( NUMBER_OF_JOINTS );
        /*for (unsigned int i = 0; i < jointNames.size(); ++i)
        {
          ROS_INFO("----------- before IK solving %s: %f", jointNames[i].c_str(), jointAngles[i]);
        }*/
        robot_state->setJointGroupPositions( joint_model_group, jointAngles ); //update robot.
        bool found_ik = robot_state->setFromIK( joint_model_group, updatedToolPose, "lwr_arm_7_link", 10, 0.1);
        if (found_ik)
        {
          //ROS_INFO("step 2-1: find a ik solution!");
          robot_state->copyJointGroupPositions( joint_model_group, ik_angles );
          for ( int j = 0; j < NUMBER_OF_JOINTS; j++ )
          {
            targetAngles[j] = ik_angles[j];
          }
          // step 3: sent control command.
          sensor_msgs::JointState msg; // send to Rviz
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "world";
          msg.name = jointNames;
          ros_fri_msgs::RMLPositionInputParameters ppp; // send to robot
          ppp.header = msg.header;
          ppp.TargetPositionVector.resize( NUMBER_OF_JOINTS );
          ppp.TargetVelocityVector.resize( NUMBER_OF_JOINTS );
          ppp.MaxVelocityVector.resize( NUMBER_OF_JOINTS );
          ppp.MaxAccelerationVector.resize( NUMBER_OF_JOINTS );

          msg.position = targetAngles; // in radians
          msg.velocity = zeroVelocities;
          msg.effort.resize( 7 ); msg.effort[6] = 0.02;// * (nk_teleop_seq & 0x1);
          for ( int j = 0; j < NUMBER_OF_JOINTS; j++ ) {
            ppp.TargetPositionVector[j] = msg.position[j];
            ppp.TargetVelocityVector[j] = msg.velocity[j];
            ppp.MaxVelocityVector[j] = jointVelocityLimits[j];
            ppp.MaxAccelerationVector[j] = jointAccelerationLimits[j];
          }
          positionGoalPublisher.publish( msg );
          rmlPositionGoalPublisher.publish( ppp );
        }
        else {
          ROS_INFO("step 2-2: do not find a ik solution!");
          }
        position_command_waiting_ = false;
        //************************************************end of cartesian control
      }
      if (Gripper_command_waiting_) {
        if (gripper_mode == 0) {
          //grasp.
          gripper_grasp_client = nh.serviceClient<wsg_50_common::Move>("/wsg_50/grasp");
          srv_grasp.request.width = gripper_width;
          srv_grasp.request.speed = 50.0;
          if (gripper_grasp_client.call(srv_grasp)){
            ROS_INFO("grasp");
          }
          else{
            ROS_ERROR("Failed to call service: /wsg_50/grasp");
            return 1;
          }
        }
        else{
          //-----------release.
          gripper_release_client = nh.serviceClient<wsg_50_common::Move>("/wsg_50/release");
          srv_release.request.width = gripper_width;
          srv_release.request.speed = 50.0;
          if (gripper_release_client.call(srv_release))
          {
            ROS_INFO("release");
          }
          else
          {
            ROS_ERROR("Failed to call service: /wsg_50/release");
            return 1;
          }
        }
        Gripper_command_waiting_ = false;
        //************************************************end of gripper control
      }
    }
    loop_rate_control.sleep();
    ros::spinOnce();
  }
  ROS_INFO( "ok." );
  exit( 0 );
  return 0;
}
