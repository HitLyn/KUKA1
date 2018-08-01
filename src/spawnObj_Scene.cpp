#include <iostream>
#include <stdio.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/Grasp.h>
#include <manipulation_msgs/GraspPlanning.h>
#include <vector>
#include <ros/ros.h>
#include "agent_ros_robot/objCommand.h"
//gloabl variables
std::vector<double> obj_xyz;
bool obj_command_waiting_;

//function declare
void obj_callback(const agent_ros_robot::objCommand::ConstPtr& msg);

//call position command.
void obj_callback(const agent_ros_robot::objCommand::ConstPtr& msg)
{
  obj_xyz = msg->position;
  obj_command_waiting_ = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "spawnObject_yuchen");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate loop_rate_other(0.5);
    ros::Rate loop_rate_control(5);
    loop_rate_control.sleep();
    ros::Duration sleep_time(10.0);

    ros::NodeHandle nh;
    ros::ServiceClient planning_scene_diff_client= nh.serviceClient<moveit_msgs::ApplyPlanningScene>("planning_scene", 1);
    planning_scene_diff_client.waitForExistence();

    ROS_INFO("1. Starting spawnObject into planning scene:");
    ros::Subscriber obj_subscriber_ = nh.subscribe("/yuchen_obj_position_command", 1, obj_callback);
    ROS_INFO("publisher, Subscriber: ---is finished");
    loop_rate_other.sleep(); // must need!--yuchen
    ros::spinOnce();

    ROS_INFO( "2. ready, waiting for *cartesian position control*" );
    bool first_flag = true;
    obj_command_waiting_ = false;
    while (ros::ok()){
      if (first_flag){ROS_INFO("----------the first time of while loop!!!");
        first_flag = false; ros::spinOnce();
      }
      else {
        //if (obj_command_waiting_) {
        if (true) {
          ROS_INFO("execute obj)xyz: %f %f %f", obj_xyz[0], obj_xyz[1], obj_xyz[2]);
          // 1. define planning_scene
          moveit_msgs::ApplyPlanningScene srv;
          moveit_msgs::PlanningScene planning_scene;
          planning_scene.is_diff = true;
          planning_scene.robot_state.is_diff = true;

          // 2. define CollisionObject
          moveit_msgs::CollisionObject object;
          object.header.frame_id = "table_top";
          object.id = "object";

          shape_msgs::SolidPrimitive primitive;
          primitive.type = primitive.CYLINDER;
          primitive.dimensions.push_back(1);

          geometry_msgs::Pose pose;
          pose.orientation.w = 1;
          pose.position.x = 0.4;
          pose.position.y = 0.5;
          pose.position.z = primitive.dimensions[0]/2 + 0.814;

          object.primitives.push_back(primitive);
          object.primitive_poses.push_back(pose);
          object.operation = object.ADD;

          // 3. add object to scene.
          planning_scene.world.collision_objects.push_back(object);

          // 4. remove attached object in case it is attached
          moveit_msgs::AttachedCollisionObject aco;
          object.operation = object.REMOVE;
          aco.object = object;
          planning_scene.robot_state.attached_collision_objects.push_back(aco);

          //5. send
          srv.request.scene = planning_scene;
          planning_scene_diff_client.call(srv);

          }
          obj_command_waiting_ = false;
          //************************************************end of spawnObject
        }
      loop_rate_control.sleep();
      ros::spinOnce();
    }
    ROS_INFO( "ok." );
    exit( 0 );
    return 0;
  }
