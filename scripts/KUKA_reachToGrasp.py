# -*- coding: utf-8 -*-
'''
a python interface of kuka robot, use for project: Reach_to_Grasp.
yuchen
Hamburg, 18.10.2017
'''
import rospy
import copy
import numpy as np
import threading
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
from math import radians
import sensor_msgs
from geometry_msgs.msg import PoseArray, PoseStamped
# tip: from package_name.msg  import ; and package_name != file_name
from agent_ros_robot.msg import SampleResult, PositionCommand, DataRequest, GripperCommand, objCommand

# global variable
curr_sample_xyzPYR = np.zeros(6)*np.nan #global variables: Temporary save Sample
curr_sample_force = np.zeros(3)*np.nan
class AgentKUKA():
    def __init__(self, start_xyzPYR = np.zeros(6), dt=0.01):
        '''initiallize agent'''
        self.dt = dt
        self.DOF = 3
        self.JDOF = 7
        self.start_xyzPYR = start_xyzPYR

        rospy.init_node('agent_kuka_node')
        self.thread_pubsub = threading.Thread(target=self.init_pubs_subs())
        self.thread_pubsub.start()
        self.listener = tf.TransformListener()

        # for visit varible
        self.state = np.zeros(self.DOF*2)*np.nan # 6*1
    #end of __init__ method

    def init_pubs_subs(self):
        # publisher
        self.action_pub = rospy.Publisher('/yuchen_controller_position_command', PositionCommand, queue_size=1)
        self.Gripper_pub = rospy.Publisher('/yuchen_controller_gripper_command', GripperCommand, queue_size=1)
        #self.data_request_pub = rospy.Publisher('/yuchen_controller_data_request', DataRequest, queue_size=1000)
        self.graspframe_pub = rospy.Publisher('/grasp_add_graspFrame', PoseStamped, queue_size=1)
        #subscriber
        self.sample_result_sub = rospy.Subscriber('/yuchen_controller_report', PoseStamped, self.sample_callback)

        # img:
        self.cv_bridge = CvBridge()
        self.color_sub = rospy.Subscriber('/camera/rgb/image_raw', sensor_msgs.msg.Image, self.color_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber('/camera/depth_registered/image', sensor_msgs.msg.Image, self.depth_callback, queue_size=1)
        self.camerainfo_sub = rospy.Subscriber('/camera/depth_registered/camera_info', sensor_msgs.msg.CameraInfo, self.cameraInfo_callback, queue_size=1)
        self.color_ros = sensor_msgs.msg.Image()
        self.depth_ros = sensor_msgs.msg.Image()
        self.cameraInfo_ros = sensor_msgs.msg.CameraInfo()

    #end of init_pubs_subs method

    def sample_callback(self, msg):
        '''get sample under data-request'''
        global curr_sample_xyzPYR, curr_sample_force
        curr_sample_xyzPYR = msg.position # is xyzPYR
        curr_sample_force = msg.force
    # end of sample_callback method

    def color_callback(self, color_rosmsg):
        # use CV_brige convert ros msg to CV msg
        self.color_ros = color_rosmsg # self.cv_bridge.imgmsg_to_cv2(color_rosmsg, "rgb8")

    def depth_callback(self, depth_rosmsg):
        # use CV_brige convert ros msg to CV msg
        try:
            self.depth_ros = depth_rosmsg #self.cv_bridge.imgmsg_to_cv2(depth_rosmsg, desired_encoding = "passthrough")
        except CvBridgeError, e:
            print e

    def cameraInfo_callback(self, cameraInfo_rosmsg):
        # use CV_brige convert ros msg to CV msg
        try:
            self.cameraInfo_ros = cameraInfo_rosmsg
        except CvBridgeError, e:
            print e

    def image_Read(self):
        return self.color_ros, self.depth_ros, self.cameraInfo_ros

    def get_data(self):
        global curr_sample_xyzPYR, curr_sample_force
        robot_position = curr_sample_xyzPYR[0:self.DOF] # xyz position
        robot_PYR = curr_sample_xyzPYR[self.DOF:self.DOF*2] # PYR euler angle
        ext_force= np.zeros(3)*np.nan
        ext_force[0] = curr_sample_force[0]
        ext_force[1] = curr_sample_force[1]
        ext_force[2] = curr_sample_force[2]
        return robot_position, robot_PYR, ext_force
    #end of get_data method
    def reset_arm(self, reset_xyzPYR = None):
        reset_xyzPYR = self.start_xyzPYR if reset_xyzPYR is None else reset_xyzPYR
        #xyzPYR = np.copy(reset_xyzPYR)

        # command control.
        reset_command = PositionCommand()
        reset_command.position = reset_xyzPYR
        rospy.sleep(0.3)
        self.action_pub.publish(reset_command)
    #end of reset_arm method

    '''
    def reset_arm(self, reset_xyzPYR = None):
    reset_xyzPYR = self.start_xyzPYR if reset_xyzPYR is None else reset_xyzPYR
    reset_xyz = reset_xyzPYR[:self.DOF]
    reset_euler = reset_xyzPYR[self.DOF:]
    reset_quat = tf.transformations.quaternion_from_euler(reset_euler[0],reset_euler[1],reset_euler[2])

    msg_action = PoseStamped()
    msg_action.header.frame_id = '/world'
    msg_action.header.stamp = rospy.Time.now()
    msg_action.pose.position.x = reset_xyz[0]
    msg_action.pose.position.y = reset_xyz[1]
    msg_action.pose.position.z = reset_xyz[2]
    msg_action.pose.orientation.x = reset_quat[0]
    msg_action.pose.orientation.y = reset_quat[1]
    msg_action.pose.orientation.z = reset_quat[2]
    msg_action.pose.orientation.w = reset_quat[3]
    rospy.sleep(0.3)  # -----------------------why must need sleep(0.5)??
    self.action_pub.publish(msg_action)
    #end of reset_arm method
    '''


    def gripper_Control(self, width, mode):
        # width - a distance, mode: 0-grasp, 1-release
        gripper_command = GripperCommand()
        gripper_command.width = width
        gripper_command.mode = mode
        rospy.sleep(0.1)  # why must sleep ??
        self.Gripper_pub.publish(gripper_command)
    #end of gripper_Control method.

    def excute_allAction(self, state_all, target_pose):
        '''execute all the action obtained from iLQR'''
        print('***************execute all the action of iLQR**********')
        print('execute: orientation = ', target_pose[self.DOF:self.DOF*2])
        for i in range(len(state_all)):
            if i%3==0:
                next_state = np.copy(state_all[i])
                next_position = np.copy(next_state[0:self.DOF])
                #moveit
                #self.RunKUKA_Moveit_obj.execute_Action(next_position)
                #rospy.sleep(0.5)

                #joint control of arm.
                xyzPYR = np.zeros(self.DOF*2)*np.nan
                xyzPYR[0:self.DOF] = next_position #xyz
                xyzPYR[self.DOF:self.DOF*2] = target_pose[self.DOF:self.DOF*2] #PYR
                action_command = PositionCommand()
                action_command.position = xyzPYR
                rospy.sleep(0.3)  #-----------------------why must need sleep(0.5)??
                self.action_pub.publish(action_command)
        # go to target
        action_command = PositionCommand()
        action_command.position = target_pose
        rospy.sleep(0.3)  #-----------------------why must need sleep(0.5)??
        self.action_pub.publish(action_command)
        print('***************(finished)execute all the action of iLQR**********')
    #end of execute_allaction method
    '''
    def excute_allAction(self, state_all, T_world_grasp_p, T_world_grasp_q):
        print 'target_pose=',T_world_grasp_p, 'T_world_grasp_q=', T_world_grasp_q
        for i in range(len(state_all)):
            tmp_state = state_all[i]
            tmp_position = tmp_state[0:self.DOF]

            # position control of arm.
            msg_action = PoseStamped()
            msg_action.header.frame_id = '/world'
            msg_action.header.stamp = rospy.Time.now()
            msg_action.pose.position.x = tmp_position[0]
            msg_action.pose.position.y = tmp_position[1]
            msg_action.pose.position.z = tmp_position[2]
            msg_action.pose.orientation.x = T_world_grasp_q[0]
            msg_action.pose.orientation.y = T_world_grasp_q[1]
            msg_action.pose.orientation.z = T_world_grasp_q[2]
            msg_action.pose.orientation.w = T_world_grasp_q[3]
            rospy.sleep(0.3)  #-----------------------why must need sleep(0.5)??
            self.action_pub.publish(msg_action)
        print('***************(finished)execute all the action of iLQR**********')
    #end of execute_allaction method

    '''


    def add_graspFrame(self, T_camera_grasp_p, T_camera_grasp_q):
        #  add the grasp frame wrt. the camera frame.
        msg_gFrame = PoseStamped()
        msg_gFrame.header.frame_id = '/camera_rgb_optical_frame'
        msg_gFrame.header.stamp = rospy.Time.now()
        msg_gFrame.pose.position.x = T_camera_grasp_p[0]
        msg_gFrame.pose.position.y = T_camera_grasp_p[1]
        msg_gFrame.pose.position.z = T_camera_grasp_p[2]
        msg_gFrame.pose.orientation.x = T_camera_grasp_q[0]
        msg_gFrame.pose.orientation.y = T_camera_grasp_q[1]
        msg_gFrame.pose.orientation.z = T_camera_grasp_q[2]
        msg_gFrame.pose.orientation.w = T_camera_grasp_q[3]
        rospy.sleep(0.3)
        self.graspframe_pub.publish(msg_gFrame)
        print 'please add grasp frame'

    def get_T_frame_grasp(self):
        '''compute T_frame_grasp, wrt all the frame(camera, world)'''

        while not rospy.is_shutdown():
            try:
                (T_world_grasp_p,T_world_grasp_quat)  = self.listener.lookupTransform('/world', '/gPoint_frame',
                                                                                      rospy.Time(0))
                (T_camera_grasp_p, T_camera_grasp_quat) = self.listener.lookupTransform('/camera_rgb_optical_frame',
                                                                                         '/gPoint_frame',rospy.Time(0))
                T_world_grasp_euler = tf.transformations.euler_from_quaternion(T_world_grasp_quat)
                T_camera_grasp_euler = tf.transformations.euler_from_quaternion(T_camera_grasp_quat)
                T_frame_grasp_dict = {
                    'T_world_grasp_p': T_world_grasp_p,
                    'T_world_grasp_quat': T_world_grasp_quat,
                    'T_world_grasp_euler':T_world_grasp_euler,
                    'T_camera_grasp_p': T_camera_grasp_p,
                    'T_camera_grasp_quat': T_camera_grasp_quat,
                    'T_camera_grasp_euler':T_camera_grasp_euler,
                }
                print 'success to get T_frame_grasp_dict'
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return T_frame_grasp_dict
if __name__ == '__main__':
    startP_Robot = np.array([0.3, 0.6, 1.1, 3.14, 1.57, 0]) #xyzPYR
    dt = 0.01
    target = np.array([0.3, 0.5, 1.3, 3.14, 1.57, 0])
    AgentKUKA_obj = AgentKUKA(startP_Robot, target, dt)
    rospy.sleep(2)
    robot_state, robot_PYR, ext_force = AgentKUKA_obj.get_data()
    print('----get robot_state 1', robot_state, 'robot_PYR=', robot_PYR, 'ext_force', ext_force)

    rospy.sleep(2)
    AgentKUKA_obj.reset_arm(startP_Robot)
    rospy.sleep(2)
    robot_state, robot_PYR, ext_force = AgentKUKA_obj.get_data()
    print('----getrobot_state 2', robot_state, 'robot_PYR=', robot_PYR, 'ext_force', ext_force)
    rospy.spin()
