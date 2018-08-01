
#include "ros/ros.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Status.h"
#include "agent_ros_robot/GripperCommand.h"

using namespace ros;

ros::ServiceClient gripper_grasp_client;
ros::ServiceClient gripper_release_client;
ros::ServiceClient gripper_homing_client;
wsg_50_common::Move srv_grasp;
wsg_50_common::Move srv_release;
wsg_50_common::Move srv_homing;

float wsg_width;
float wsg_force;
bool Gripper_command_waiting_;
float gripper_width;
int gripper_mode;

void gripperCallback(const wsg_50_common::Status status);
void GripperCommand_callback(const agent_ros_robot::GripperCommand::ConstPtr& msg);

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

int main(int argc, char ** argv) {
  ros::init(argc, argv, "runKUKA_fri_gripper_yuchen", 1 );
  ros::NodeHandle nh;
  ros::Rate loop_rate(1); // keep 2Hz for control loop, I.e.sleep 0.5 seconds*(1/2)
  wsg_width = 0.0; wsg_force = 0.0;
  ros::Subscriber gripper_sub = nh.subscribe("wsg_50/status", 1000, gripperCallback);

  ros::Subscriber gripper_subscriber_ = nh.subscribe("/yuchen_controller_gripper_command", 1, GripperCommand_callback);

  //----------- homing.
  /*
  gripper_homing_client = n.serviceClient<wsg_50_common::Move>("/wsg_50/homing");
  srv_homing.request.width = 0.0;
  srv_homing.request.speed = 400.0;
  if (gripper_homing_client.call(srv_homing))
  {
    ROS_INFO("homing");
  }
  else
  {
    ROS_ERROR("Failed to call service: /wsg_50/grasp");
    return 1;
  }
  loop_rate.sleep(); */

  // ----------grasp.
  gripper_grasp_client = nh.serviceClient<wsg_50_common::Move>("/wsg_50/grasp");
  srv_grasp.request.width = 30;
  srv_grasp.request.speed = 50.0;
  if (gripper_grasp_client.call(srv_grasp))
  {
    ROS_INFO("grasp");
  }
  else
  {
    ROS_ERROR("Failed to call service: /wsg_50/grasp");
    return 1;
  }
  loop_rate.sleep();

  //-----------release.
  gripper_release_client = nh.serviceClient<wsg_50_common::Move>("/wsg_50/release");
  srv_release.request.width = 70;
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
  loop_rate.sleep();
  ros::spinOnce();

  // gripper command from agent
  Gripper_command_waiting_ = false;
  while (ros::ok()) {
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
        loop_rate.sleep();
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
        loop_rate.sleep();
      }
    Gripper_command_waiting_ = false;
    loop_rate.sleep();
    }
    ros::spinOnce();
  }
  return 0;
}
