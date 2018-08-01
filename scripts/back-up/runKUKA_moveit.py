# -*- coding: utf-8 -*-
'''
implementations for reinforcement learning

using moveit to control UR5

yuchen 2017.02.15
deng@informatik.uni-hamburg.de
'''

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

import numpy as np
import scipy as sp
import sys
import tf

class RunKUKA_Moveit():
    '''Run ur5 based on Moviet'''
    def __init__(self):
        rospy.init_node('RL_kuka_moveit')

        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()

        # for RVIZ to visualize.
        display_trajectory_publisher = rospy.Publisher(
                                          '/move_group/display_planned_path',
                                          moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.group = moveit_commander.MoveGroupCommander('arm')

        self.end_effector_link = self.group.get_end_effector_link()
        #rospy.loginfo("end_effector:" + str(self.end_effector_link) )

    #end of init method

    def execute_Action(self, position_target):
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = position_target[0]
        pose_target.position.y = position_target[1]
        pose_target.position.z = position_target[2]
        quat = tf.transformations.quaternion_from_euler(position_target[3:6])
        print('test:', quat)

        pose_target.orientation.x = quat[0]
        pose_target.orientation.y = quat[1]
        pose_target.orientation.z = quat[2]
        pose_target.orientation.w = quat[3]
        self.group.set_pose_target(pose_target)
        plan = self.group.plan()
        self.group.execute(plan)
    #end of execute_ActionUR5 method

    def reset_RoScenaro(self, position_start):
        '''Planning to a Pose start'''
        pose_start = geometry_msgs.msg.Pose()
        pose_start.position.x = position_start[0]
        pose_start.position.y = position_start[1]
        pose_start.position.z = position_start[2]
        quat = tf.transformations.quaternion_from_euler(position_start[3], position_start[4],position_start[5])
        print('test:quat =', quat)

        pose_start.orientation.x = quat[0]
        pose_start.orientation.y = quat[1]
        pose_start.orientation.z = quat[2]
        pose_start.orientation.w = quat[3]
        self.group.set_pose_target(pose_start)
        plan = self.group.plan()

        self.group.execute(plan)
    #end of reset_RoScenaro method

    def get_RoState(self):
        pose = self.group.get_current_pose(self.end_effector_link).pose
        #rospy.loginfo("right--:" + str(pose) )
        state_position = np.array([pose.position.x, pose.position.y, pose.position.z])
        return state_position
    #end of reset_RoScenaro method

    def get_joint(self):
        joint_angle = self.group.get_current_joint_values()
        return joint_angle
    #end of get_joint method

    def get_objectState(self):
        return self.object_position
    #end of return_objectState method

    def object_callback(self, msg):
        x = msg.pose.position.x # update object position
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.object_position = np.array([x, y, z])
        #print 'object state:', self.object_position
    #end of object_callback method

if __name__=='__main__':
        RunKUKA_Moveit_obj = RunKUKA_Moveit()

        joint_angle = RunKUKA_Moveit_obj.get_joint()
        print(joint_angle)

        joint_angle = [1.5351591399699331, 1.0150655731355351, -0.5996188049950094, -1.8963631396058056, -0.33493435375398845, -1.4494985666885416, 0.5753884018917604]
        RunKUKA_Moveit_obj.group.set_joint_value_target(joint_angle)

        plan = RunKUKA_Moveit_obj.group.plan()
        RunKUKA_Moveit_obj.group.execute(plan)
        print "============ Waiting while RVIZ displays plan2..."
        rospy.sleep(3)
        curr_state = RunKUKA_Moveit_obj.get_RoState()
        print(curr_state)
        '''
        start_position = [0.52, 0.4, 1.3]
        RunKUKA_Moveit_obj.reset_RoScenaro(start_position)
        rospy.sleep(2)

        target_position =np.array([0.45, 0.5, 1.3])
        RunKUKA_Moveit_obj.execute_ActionUR5(target_position)
        '''

