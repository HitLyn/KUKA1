# -*- coding: utf-8 -*-
'''
a env for human robot cooperation
yuchen
Hamburg, 2017.10.19
'''
import sys
sys.path.append('../')
import rospy
import copy
import numpy as np
import math
from math import radians
import pandas as pd
import threading

from config import config_KUKA
from runKUKA_moveit import RunKUKA_Moveit
from agent_utils import msg_to_sample, action_to_msg
#from agent_ros_robot.msg import SampleResult, PositionCommand, DataRequest, num

curr_sample = {}
class AgentKUKA():
    def __init__(self, start_xyzPYR = np.zeros(6), dt=0.01):
        '''initiallize agent'''
        config = copy.deepcopy(config_KUKA)
        self._hyperparams = config
        self.dt = dt
        self.DOF = 3
        self.JDOF = 7
        self.startP_Robot = start_xyzPYR
        #print('self._hyperparams',self._hyperparams)

        '''
        #rospy.init_node('agent_kuka_node')
        #---using ros_controller
        self.thread_pubsub = threading.Thread(target=self.init_pubs_subs())
        self.thread_pubsub.start()
        self.seq_id = 0 # used for setting seq in ROS conmands
        '''
        #--using moveit of Kuka
        self.RunKUKA_Moveit_obj = RunKUKA_Moveit()
        # for visit varible
        self.state = np.array([0,0,0,0,0,0])*np.nan # 6*1
    #end of __init__ method

    def init_pubs_subs(self):
        # publisher
        #self.trail_pub = rospy.publisher(self._hyperparams['trial_command_topic'], TrialCommand)
        self.action_pub = rospy.Publisher('position_control_yuchen', PositionCommand, queue_size=10)
        #self.reset_pub = rospy.Publisher(self._hyperparams['reset_command_topic'], PositionCommand, queue_size=10)
        self.data_request_pub = rospy.Publisher('data_request_topic_yuchen', DataRequest, queue_size=10)
        self.pub_test = rospy.Publisher('custom_chatter', num, queue_size=10)
        #subscriber
        self.sample_result_sub = rospy.Subscriber('sample_result_topic_yuchen', SampleResult, self.sample_callback)
    #end of init_pubs_subs method

    def sample_callback(self, msg):
        '''get sample under data-request'''
        rospy.loginfo('get sample from topic: sample_result_topic..')
        global curr_sample
        curr_sample ={'joint_position': msg.jointAngle, 'ext_force': msg.force}
    # end of sample_callback method

    def get_data(self):
        rospy.loginfo('send data_request to robot....')
        data_request = DataRequest()
        data_request.request_tag = True
        rospy.sleep(0.5)
        self.data_request_pub.publish(data_request)
        rospy.sleep(0.1)

        global curr_sample
        state, ext_force = np.array(curr_sample['joint_position']), np.array(curr_sample['ext_force'])
        return state[0:7], ext_force
    #end of get_data method

    def reset_arm(self, reset_state = None):
        """send a position command to an arm."""
        '''
        rospy.loginfo('send reset_arm conmand..')
        reset_command = PositionCommand()
        reset_command.position = np.array([0.0, radians(0.0), radians(0.0),radians(0), radians(0.0),radians(0.0), radians(0.0)])
        rospy.sleep(0.5)  # why must sleep ??
        self.action_pub.publish(reset_command)
        self.state = np.array([0.0, radians(0.0), radians(0.0),radians(0), radians(0.0),radians(0.0), radians(0.0)])
        '''
        reset_state = self.startP_Robot if reset_state is None else reset_state
        # run moveit
        self.RunKUKA_Moveit_obj.reset_RoScenaro(reset_state)
        rospy.sleep(0.5)

        #get state
        next_pxy = self.RunKUKA_Moveit_obj.get_RoState()
        next_vel = np.zeros(self.DOF)
        next_state = np.hstack((next_pxy, next_vel))
        #reset agent state
        self.state = np.copy(next_state)
        return next_state
    #end of reset_arm method

    def test_num(self):
        num_msg = num()
        num_msg.name = 'yuchen'
        num_msg.age = 29
        rospy.loginfo('send test_num conmand..')
        self.pub_test.publish(num_msg)
    #end of test_num method

    def robot_step(self, curr_state, action, noisy = True):
        '''with cartesian force to compute the change of joint '''
        Md = np.eye(3)
        Cd = np.eye(3)

        # impedance control.
        A = np.array([[0,0,0,1,0,0], [0,0,0,0,1,0], [0,0,0,0,0,1],
                        [0,0,0,-1,0,0], [0,0,0,0,-1,0], [0,0,0,0,0,-1]])
        B = np.array([[0,0,0], [0,0,0], [0,0,0], [1,0,0], [0,1,0], [0,0,1]])
        #print('A.shape', A.shape, 'B.shape', B.shape, 'dt', self.dt, \
                    #'curr_state.shape',curr_state.shape, 'action.shape', action.shape )
        f_z = A * self.dt + np.eye(6)
        f_u = B * self.dt
        next_state = np.dot(f_z, curr_state) + np.dot(f_u, action)
        #print('---------impenance', 'z_next=', z_next)

        #moveit
        next_position = next_state[0:self.DOF]
        self.RunKUKA_Moveit_obj.execute_Action(next_position)
        rospy.sleep(0.5)

        #reset agent state
        self.state = np.copy(next_state)
        #self.jState = np.copy(jnext)
        return next_state

    def simulate(self, startP_robot, action_all):
        """ do a rollout, starting at x0 and the control sequence U
        x0 : the initial state of the system
        U  : the control sequence to apply
        """
        tN = action_all.shape[0]
        num_states = startP_robot.shape[0]
        dt = self.dt
        state_all = np.zeros((tN, num_states)) #Cartesian state: position and velocity
        state_all[0] = startP_robot
        cost = 0

        # Run simulation with substeps
        for t in range(tN-1):
            state_all[t+1] = self.robot_step(Z_state[t], action_all[t]) # z(t+1) = f(x(t), u(t))
            l,_,_,_,_,_ = self.cost(state_all[t], U[t])
            cost = cost + dt * l

        # Adjust for final cost, subsample trajectory
        l_f,_,_ = self.cost_final(state_all[-1])
        cost = cost + l_f
        return state_all, cost
    #end of simulate method

    def cost(self, curr_state, action):
        """ the immediate state cost function """
        u = np.copy(action)
        x = np.copy(curr_state)
        dof = u.shape[0]
        num_states = x.shape[0]

        # immediate cost function:
        cost_tmp = np.sum(u**2)
        return cost_tmp
    #end of cost method

    def cost_final(self, curr_state):
        z = np.copy(curr_state)
        num_states = z.shape[0]
        pxy_EE = z[0:self.DOF]
        vel_EE = z[self.DOF:self.DOF*2]

        Q = 1e4 * np.eye(self.DOF)
        R = 1e4 * np.eye(self.DOF)

        cost_final = np.dot(np.dot((pxy_EE - self.target).T, Q), pxy_EE - self.target) + \
                                np.dot(np.dot(vel_EE.T, R), vel_EE)
        return cost_final
    #end cost_final method

    def excute_allAction(self, state_all):
        '''execute all the action obtained from iLQR'''
        print('***************execute all the action of iLQR**********')
        for i in range(len(state_all)):
            next_state = np.copy(state_all[i])
            next_position = np.copy(next_state[0:self.DOF])
            self.RunKUKA_Moveit_obj.execute_Action(next_position)
            rospy.sleep(0.5)
        print('***************(finished)execute all the action of iLQR**********')
    #end of execute_allaction method

''' test'''
if __name__ == '__main__':
    startP_Robot = np.array([0.3, 0.3, 1.2, 0, 0, 0])
    dt = 0.01
    target = np.array([0.67, 0.1, 1.0])
    AgentKUKA_obj = AgentKUKA(startP_Robot, dt)

    tmp_state = np.array([0.3, 0.3, 1.6, 0, 0, 0])
    tmp_state = np.array([ 0.55,  0.40,  1.4,  -1.57,0, 0])
    tmp_state = np.array([0.23, 0.7, 1.01, -1.57, 0, 0])
    AgentKUKA_obj.reset_arm(tmp_state)
    print('***************reset_arm**********')
    rospy.sleep(2)

    '''
    for timer in range(50):
        print('-----time:', timer)
        if timer == 0:
            AgentKUKA_obj.reset_arm()

            action = np.array([0, 30, 0])

            curr_pxy = np.copy(startP_Robot)
            curr_vel = np.zeros(3)
            state_new = np.hstack((curr_pxy, curr_vel))
            print('------force control', 'z =', state_new )
            joint_new = AgentKUKA_obj.RunKUKA_Moveit_obj.get_joint()
            #do action.
            state_new, joint_new = AgentKUKA_obj.robot_step(state_new, joint_new, action)
            rospy.sleep(1)

        else:
            action = np.array([0, 30, 0])
            #do action.
            state_new, joint_new = AgentKUKA_obj.robot_step(state_new, joint_new, action)
            rospy.sleep(1)

    rospy.spin()
    '''
