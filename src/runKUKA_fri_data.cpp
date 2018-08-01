/*
1. to realize cartesian position control:  cartesian position -- IK--joint position
this file aim to :
obtain robot state and interactive force , publish to agent.
History:
2017.06.20 - zhen deng
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

#define  NUMBER_OF_JOINTS                        7
#define  NUMBER_OF_CARTESIAN                     6
#define NUMBER_OF_FORCES  3
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
geometry_msgs::Vector3      arm_forces_;  //yuchen

// function declare.
void jointStateCallback(const sensor_msgs::JointState jointStateMessage);
void printTransform_tf(std::string prefix, tf::Transform trafo);
void printTransform_eigen(std::string prefix, Eigen::Affine3d affine3d);
bool xyzRpyToTransform( tf::Transform* trafo, std::string tokens ); // x y z r p y -> transform
void parseInitializationValues( std::vector<double> &data, std::string s );
void estimatedExternalWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
geometry_msgs::Vector3 MovingAverageFilter_process(geometry_msgs::Vector3 force_tmp);

// mean filter.
int k = 0; // k stores the index of the current array read to create a circular memory through the array
int dataPointsCount;
int i;
float values_x[MAX_DATA_POINTS], values_y[MAX_DATA_POINTS], values_z[MAX_DATA_POINTS];
float sum_forcebais[3];
int count_bais = 0;

// callback function.
void jointStateCallback(const sensor_msgs::JointState jointStateMessage)
{
  jointAngles = jointStateMessage.position;
  jointVelocities = jointStateMessage.velocity;
  jointTorques = jointStateMessage.effort;
}

//call force.
void estimatedExternalWrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  arm_forces_ = msg->wrench.force;
  arm_forces_ = MovingAverageFilter_process(arm_forces_);
}
// mean filter.
geometry_msgs::Vector3 MovingAverageFilter_process(geometry_msgs::Vector3 force_tmp) {
  float out_x = 0,out_y = 0,out_z = 0;
  values_x[k] = force_tmp.x;
  values_y[k] = force_tmp.y;
  values_z[k] = force_tmp.z;

  k = (k+1) % MAX_DATA_POINTS;
  for (i=0; i<MAX_DATA_POINTS; i++) {
    out_x += values_x[i];
    out_y += values_y[i];
    out_z += values_z[i];
  }
  geometry_msgs::Vector3 force_result;
  force_result.x = out_x/MAX_DATA_POINTS;
  force_result.y = out_y/MAX_DATA_POINTS;
  force_result.z = out_z/MAX_DATA_POINTS;
  return force_result;
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
  ros::init(argc, argv, "runKUKA_fri_data_yuchen", 1 );
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
  for (int i=0; i<MAX_DATA_POINTS; i++)
  {
    values_x[i] = 0; values_y[i] = 0; values_z[i] = 0;
  }// fill the array with 0's
  sum_forcebais[0] = 0;sum_forcebais[1] = 0;sum_forcebais[2] = 0;

  nh.param( "velocity_limits_radians", velocity_limits, // 55 deg/sec = 0.9599 rad/sec
             std::string( "1.9 1.9 2.25 2.25 2.25 3.14 3.14" ));
  parseInitializationValues( jointVelocityLimits, velocity_limits );
  nh.param( "acceleration_limits_radians", acceleration_limits, // 50 deg/sec^2 = 0.87 rad/sec/sec
             std::string( "0.3 0.3 0.5 0.5 0.5 0.5 0.5" ));
  parseInitializationValues( jointAccelerationLimits, acceleration_limits );

  // MoveIt robot model loader
  ROS_INFO( "Moveit: creating the RobotModelLoader now..." );
  robot_model_loader::RobotModelLoader robot_model_loader = robot_model_loader::RobotModelLoader( "robot_description" );
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO( "Moveit: model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState( kinematic_model));
  robot_state->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup( "arm" );

  // ROS publishers and subscribers
  ros::Subscriber sub = nh.subscribe( "lwr/joint_states", 1, jointStateCallback );
  ros::Subscriber externalWrenchSubscriber = nh.subscribe("/lwr/estimatedExternalTcpWrench", 1,estimatedExternalWrenchCallback);
  ros::Publisher report_publisher_ = nh.advertise<agent_ros_robot::SampleResult>("/yuchen_controller_report", 1);
  ROS_INFO("publisher, Subscriber: ---is finished");
  loop_rate_other.sleep(); // must need!--yuchen
  ros::spinOnce();
  for (unsigned int i = 0; i < jointNames.size(); ++i)
  {
    ROS_INFO("----------- before while loop %s: %f", jointNames[i].c_str(), jointAngles[i]);
  }

  //**********************************cartesian position control***************
  bool first_flag = true;
  while (ros::ok())
  {
    if (first_flag){ROS_INFO("----------the first time of while loop!!!");
      first_flag = false; ros::spinOnce();
    }
    else{//.....................................phase 1: publish sample to agent.
      // 1. Forward Kinematics
      double Y, P, R;
      robot_state->setJointGroupPositions( joint_model_group, jointAngles); // from real-robot joint position
      const Eigen::Affine3d &arm7_transform_tmp = robot_state->getGlobalLinkTransform("lwr_arm_7_link");
      tf::Transform tform_tmp = tf::Transform();
      tf::transformEigenToTF(arm7_transform_tmp, tform_tmp);
      tf::Vector3 xyz = tform_tmp.getOrigin();
      tf::Matrix3x3 YPR_matrix = tform_tmp.getBasis();
      YPR_matrix.getEulerYPR(Y, P, R);

      // 2. update sample.position
      agent_ros_robot::SampleResult current_time_step_sample_;// Sensor data for the current time step.
      current_time_step_sample_.position.resize(6);
      current_time_step_sample_.position[0] = xyz.getX(); current_time_step_sample_.position[1] = xyz.getY();
      current_time_step_sample_.position[2] = xyz.getZ(); current_time_step_sample_.position[3] = P;
      current_time_step_sample_.position[4] = Y; current_time_step_sample_.position[5] = R;
      //ROS_INFO("start Joint before control*(joint_values): %f %f %f",  Y, P, R);
      // 3. update sample.force.
      count_bais +=1;
      if (count_bais > MAX_DATA_POINTS && count_bais < MAX_DATA_POINTS*2) {
        sum_forcebais[0] += arm_forces_.x;
        sum_forcebais[1] += arm_forces_.y;
        sum_forcebais[2] += arm_forces_.z;

        current_time_step_sample_.force.resize(3);
        current_time_step_sample_.force[0] = arm_forces_.x;
        current_time_step_sample_.force[1] = arm_forces_.y;
        current_time_step_sample_.force[2] = arm_forces_.z;
        ROS_INFO("---perfroming mean filter and setBasis of force: %d", count_bais);
      }
      else{
        current_time_step_sample_.force.resize(3);
        current_time_step_sample_.force[0] = arm_forces_.x - sum_forcebais[0]/MAX_DATA_POINTS;
        current_time_step_sample_.force[1] = arm_forces_.y - sum_forcebais[1]/MAX_DATA_POINTS;
        current_time_step_sample_.force[2] = arm_forces_.z - sum_forcebais[2]/MAX_DATA_POINTS;

        /*ROS_INFO("force of arm: %f %f %f", current_time_step_sample_.force[0],
                                            current_time_step_sample_.force[1],
                                            current_time_step_sample_.force[2]);*/
      }
      //4. send sample to agent.
      report_publisher_.publish(current_time_step_sample_);
      //ROS_INFO("-------runKUKA_actionlib: send sample to agent");
    }
    loop_rate_control.sleep();
    ros::spinOnce();
  }
  ROS_INFO( "ok." );
  exit( 0 );
  return 0;
}
