# -*- coding: utf-8 -*-
'''
Usage: a agenet for controling the kuka robot.
yuchen
Hamburg, 2016.12.21
'''
import sys
sys.path.append('../')
import rospy
import copy
import numpy as np
import math
from math import radians
from config import config_KUKA
# tip: from package_name.msg  import ; and package_name != file_name
from agent_KUKA_robot.msg import SampleResult, PositionCommand, DataRequest, GripperCommand
import threading

# global variable
curr_sample_xyzPYR = np.zeros(6)*np.nan #global variables: Temporary save Sample
curr_sample_force = np.zeros(3)*np.nan
class AgentKUKA():
    def __init__(self, dt=0.01, start_xyzPYR = np.zeros(6), target_xyzPYR=np.zeros(6)):
        '''initiallize agent'''
        config = copy.deepcopy(config_KUKA)
        self._hyperparams = config
        self.dt = dt
        self.DOF = 3
        self.JDOF = 7
        self.start_xyzPYR = start_xyzPYR
        self.target_xyzPYR = target_xyzPYR
        print(self.target_xyzPYR)

        rospy.init_node('agent_kuka_node')
        self.thread_pubsub = threading.Thread(target=self.init_pubs_subs())
        self.thread_pubsub.start()

        # for visit varible
        self.state = np.zeros(self.DOF*2)*np.nan # 6*1
    #end of __init__ method

    def init_pubs_subs(self):
        # publisher
        self.action_pub = rospy.Publisher('/yuchen_controller_position_command', PositionCommand, queue_size=1)
        self.Gripper_pub = rospy.Publisher('/yuchen_controller_gripper_command', GripperCommand, queue_size=1)
        #self.data_request_pub = rospy.Publisher('/yuchen_controller_data_request', DataRequest, queue_size=1000)
        #subscriber
        self.sample_result_sub = rospy.Subscriber('/yuchen_controller_report', SampleResult, self.sample_callback)
    #end of init_pubs_subs method

    def sample_callback(self, msg):
        '''get sample under data-request'''
        global curr_sample_xyzPYR, curr_sample_force
        curr_sample_xyzPYR = msg.position #is xyzPYR
        curr_sample_force = msg.force
    # end of sample_callback method

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

    def reset_arm(self, c, Num_Qlearning, reset_xyzPYR = None):
        reset_xyzPYR = self.start_xyzPYR if reset_xyzPYR is None else reset_xyzPYR
        xyzPYR = np.copy(reset_xyzPYR)

        # command control.
        reset_command = PositionCommand()
        reset_command.position = xyzPYR
        #rospy.sleep(0.5)  # why must sleep ??
        if c == Num_Qlearning-1:
            self.action_pub.publish(reset_command)
        print('--------RL_agent: send reset_arm command', 'xyzPYR=',xyzPYR)

        #reset agent state for Q-learning: position + velocity
        next_position = np.copy(xyzPYR[0:self.DOF])
        next_velocity = np.zeros(3)
        next_state = np.concatenate((next_position, next_velocity))
        self.state = np.copy(next_state)
        return next_state
    #end of reset_arm method
    def gripper_Control(self, width, mode):
        # width - a distance, mode: 0-grasp, 1-release
        gripper_command = GripperCommand()
        gripper_command.width = width
        gripper_command.mode = mode
        rospy.sleep(0.1)  # why must sleep ??
        self.Gripper_pub.publish(gripper_command)
        print('--------RL_agent: control gripper.')
    #end of gripper_Control method.

    def robot_step(self, curr_state, impe_action, c, Num_Qlearning, human_action = None):

        #next_state = self.force2jointControl(curr_state, impe_action,c, Num_Qlearning)

        next_state = self.force2jointControl_human(curr_state, impe_action,human_action, c, Num_Qlearning)
        return next_state
    #end of robot_step method

    def force2jointControl(self, curr_state, impe_action, c, Num_Qlearning):
        '''with cartesian force to compute the change of joint '''
        action = impe_action
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

        # on-line adjust state
        '''
        state_varing = np.zeros(3)
        if np.linalg.norm(human_action) > 2:
            state_varing =  0.1*human_action
        print('-------test:', 'state_varing = ', state_varing, 'next_state=', next_state)
        next_state[0:self.DOF] += state_varing
        '''
        '''
        # Jacobians convert- joint velocity.
        jacEE = self.gen_jacEE()
        #J_part = np.dot(np.dot(jacEE.T, jacEE) + gama**2 *np.eye(2), jacEE.T)
        dq_next = np.dot(jacEE.T, vel_EE_next) #catesian velocity to joint velocity- distanvve 2 angle
        print('----test', 'jacEE * dq_next=', np.dot(jacEE, dq_next))

        curr_q = self.arm.q
        q_next = curr_q + dq_next * self.dt
        #print('selected change of joint of force2jointControl method:', dq_next * self.dt)
        '''
        #joint control of arm.
        xyzPYR = np.zeros(self.DOF*2)*np.nan
        next_position = next_state[0:self.DOF]
        #print('----------test1:', 'next_position =', next_position

        xyzPYR[0:self.DOF] = np.copy(next_position) #xyz
        xyzPYR[self.DOF:self.DOF*2] = np.copy(self.target_xyzPYR[self.DOF:self.DOF*2]) #PYR
        action_command = PositionCommand()
        action_command.position = xyzPYR
        rospy.sleep(0.2)
        if c == Num_Qlearning-1:
            self.action_pub.publish(action_command)

        '''
        #moveit
        next_position = next_state[0:self.DOF]
        self.RunKUKA_Moveit_obj.execute_Action(next_position)
        rospy.sleep(0.5)
        '''
        #reset agent state for Q-learning : position + velocity
        self.state = np.copy(next_state)
        return next_state
    #end of force2jointControl method


    def force2jointControl_human(self, curr_state, impe_action, human_action, c, Num_Qlearning):
        '''with cartesian force to compute the change of joint '''
        action = impe_action
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

        #joint control of arm.
        xyzPYR = np.zeros(self.DOF*2)*np.nan
        next_position = next_state[0:self.DOF]
        #print('----------test1:', 'next_position =', next_position)

        #implenetation of human compliant.
        if np.linalg.norm(human_action) > 10: # due to the noisy.
            human_action[-1] = -1 * human_action[-1] # change the direction
            next_position = next_position + 0.001 * human_action
            #print('-----------------test2:', '0.001 * human_action=', 0.001 * human_action)

        xyzPYR[0:self.DOF] = np.copy(next_position) #xyz
        xyzPYR[self.DOF:self.DOF*2] = np.copy(self.target_xyzPYR[self.DOF:self.DOF*2]) #PYR
        action_command = PositionCommand()
        action_command.position = xyzPYR
        rospy.sleep(0.2)
        if c == Num_Qlearning-1:
            self.action_pub.publish(action_command)

        #reset agent state for Q-learning : position + velocity
        next_state[0:self.DOF] = next_position
        self.state = np.copy(next_state)
        return next_state
    #end of force2jointControl method
    '''
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
    '''
    def cost(self, curr_state, impe_action, human_action):
        pxy_EE = curr_state[0:self.DOF]
        vel_EE = curr_state[self.DOF:self.DOF*2]

        Q = 1e2 * np.eye(self.DOF)
        R = 1 * np.eye(self.DOF)
        #cost_task = np.dot(np.dot((pxy_EE - self.target_xyzPYR[0:self.DOF]).T, Q), pxy_EE - self.target_xyzPYR[0:self.DOF]) + \
        #                        np.dot(np.dot(vel_EE.T, R), vel_EE)
        cost_task = np.dot(np.dot((pxy_EE - self.target_xyzPYR[0:self.DOF]).T, Q), pxy_EE - self.target_xyzPYR[0:self.DOF])

        cost_human = 0.0
        if np.linalg.norm(human_action) > 10: # consider noise
            cost_human = 2 * np.linalg.norm(np.cross(impe_action, human_action))/np.linalg.norm(impe_action)
        print('------test: ','cost_human = ', cost_human, 'cost_task=', cost_task)
        #immediate cost function.
        cost = cost_task + cost_human
        return cost, cost_task, cost_human
    #end of cost method

    def cost_final(self, curr_state):
        pxy_EE = curr_state[0:self.DOF]
        vel_EE = curr_state[self.DOF:self.DOF*2]

        Q = 1e2 * np.eye(self.DOF)
        R = 1 * np.eye(self.DOF)

        cost_final = np.dot(np.dot((pxy_EE - self.target_xyzPYR[0:self.DOF]).T, Q), pxy_EE - self.target_xyzPYR[0:self.DOF])
        #cost_final = np.dot(np.dot((pxy_EE - self.target_xyzPYR[0:self.DOF]).T, Q), pxy_EE - self.target_xyzPYR[0:self.DOF])

        return cost_final, cost_final, 0
    #end cost_final method

    def excute_allAction(self, state_all):
        '''execute all the action obtained from iLQR'''
        print('***************execute all the action of iLQR**********')
        for i in range(len(state_all)):
            next_state = np.copy(state_all[i])
            next_position = np.copy(next_state[0:self.DOF])
            #moveit
            #self.RunKUKA_Moveit_obj.execute_Action(next_position)
            #rospy.sleep(0.5)

            #joint control of arm.
            xyzPYR = np.zeros(self.DOF*2)*np.nan
            xyzPYR[0:self.DOF] = next_position #xyz
            xyzPYR[self.DOF:self.DOF*2] = np.copy(self.target_xyzPYR[self.DOF:self.DOF*2]) #PYR
            action_command = PositionCommand()
            action_command.position = xyzPYR
            #rospy.sleep(0.5)  #-----------------------why must need sleep(0.5)??
            self.action_pub.publish(action_command)
        print('***************(finished)execute all the action of iLQR**********')
    #end of execute_allaction method

''' test'''
if __name__ == '__main__':
    startP_Robot = np.array([0.3, 0.6, 1.1, 3.14, 1.57, 0]) #xyzPYR
    dt = 0.01
    target = np.array([0.3, 0.5, 1.3, 3.14, 1.57, 0])
    AgentKUKA_obj = AgentKUKA(startP_Robot, dt, target)
    rospy.sleep(2)
    robot_state, robot_PYR, ext_force = AgentKUKA_obj.get_data()
    print('----get robot_state 1', robot_state, 'robot_PYR=', robot_PYR, 'ext_force', ext_force)

    rospy.sleep(2)
    AgentKUKA_obj.reset_arm(startP_Robot)
    rospy.sleep(2)
    robot_state, robot_PYR, ext_force = AgentKUKA_obj.get_data()
    print('----getrobot_state 2', robot_state, 'robot_PYR=', robot_PYR, 'ext_force', ext_force)

    rospy.spin()
