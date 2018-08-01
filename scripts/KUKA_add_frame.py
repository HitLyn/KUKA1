#!/bin/bash
#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
import time

def grasp_callback(msg):
    print('received grasp display command...')
    global T_camera_grasp_p, T_camera_grasp_q, is_graspdisplay
    T_camera_grasp_p = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    T_camera_grasp_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    is_graspdisplay = True
    print('T_camera_grasp: position', T_camera_grasp_p)
# end of grasp_callback method

def main():
    rospy.init_node('add_frameDisplay')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(2.0)

    global T_camera_grasp_p, T_camera_grasp_q, is_graspdisplay
    T_camera_grasp_p = np.zeros(3)
    T_camera_grasp_q = np.zeros(4)
    is_graspdisplay = False
    rospy.Subscriber("/grasp_add_graspFrame", PoseStamped, grasp_callback)

    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.7, 0.80),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "/tag_yuchen",
                         "/world")
        # for testing
        '''
        br.sendTransform((0.0, 0.01, 1.1),
                         tf.transformations.quaternion_from_euler(-2.5, 0, 0),
                         rospy.Time.now(),
                         "/camera_rgb_optical_frame",
                         "/world")
        '''


        # 1. add the graspDisplay frame. i.e., the gPoint
        if is_graspdisplay:
            br.sendTransform((T_camera_grasp_p[0], T_camera_grasp_p[1], T_camera_grasp_p[2]),
                             (T_camera_grasp_q[0], T_camera_grasp_q[1], T_camera_grasp_q[2], T_camera_grasp_q[3]),
                             rospy.Time.now(),
                             "/gPoint_frame",
                             "/camera_rgb_optical_frame")
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
