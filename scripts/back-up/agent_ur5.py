# -*- coding: utf-8 -*-
'''
a env for human robot cooperation
yuchen
Hamburg, 2016.12.21
'''

import rospy
import numpy as np
from agent2robot.runUR5_moveit import RunUR5_Moveit
from utility.miniJerk_movement import miniJerk_3D, miniJerk_1D
import pandas as pd


class AgentUR5(object):
    def __init__(self, startState = np.array([0.67, 0.82, 0.78])):
        self.startState = startState

        #self.runUR5Moveit_obj = RunUR5_Moveit()
        #reference trajectory
        self.referTraj = self.get_referTraj()
        '''
        #for simulation: use the minimum-jerk produce dataset
        start_position = [0, 0, 0]
        target_position = [1, 2, 3]
        px, py, pz = miniJerk_3D(start_position, target_position)

        sp= 0.05 # Sigma for position noise
        m = len(px)
        Xm = px + sp * (np.random.randn(m))
        Ym = py + sp * (np.random.randn(m))
        Zm = pz + sp * (np.random.randn(m))
        self.objectMeasure_simulate = np.vstack((Xm,Ym,Zm))
        '''
        #for simulation: using collect object data
        referTraj_pd = pd.read_csv('/home/yuchen/catkin_ws/src/demostration_learning/script/results/objectTraj_01.csv')
        referTraj = referTraj_pd.values
        referTraj = np.delete(np.array(referTraj), (0), axis=1) # delete index_col
        traj = referTraj[11:41]
        px = traj[:, 0]
        py = traj[:, 1]
        pz = traj[:, 2]
        m = len(px)
        self.objectMeasure_simulate = np.vstack((px,py,pz))
        print('init method of UR5Env:', 'objectMeasure_simulate', self.objectMeasure_simulate, self.objectMeasure_simulate.shape )

    #end of __init__ method

    def reset_ur5(self):
        #for simulation:
        self.state = self.startState
        # for UR5:
        #self.runUR5Moveit_obj.reset_RoScenaro(startState) # run ur5 to state point
        rospy.sleep(0.05)
        #self.state = self.get_robotState()
        return self.state
    #end of robot_reset method

    def robot_step(self, robot_action, i, n):
        # excute action on UR5:
        # for simulation:
        self.dt = 0.5
        #print('robot_step method:', 'state', self.state, 'robot_action', robot_action)
        self.state = self.state + np.array(robot_action) * self.dt
        #for ur5:
        #self.state = self.runUR5Moveit_obj.execute_ActionUR5(action)
        rospy.sleep(1)
        #self.state = self.get_robotState()

        # compute reward:
        #--for real-running
        #object_state = self.runUR5Moveit_obj.get_objectState()
        #--for simulation:
        object_state = self.objectMeasure_simulate[:, i]
        distance_p2p = []
        for k in range(len(self.referTraj)):
            distance_tmp = np.linalg.norm(object_state - self.referTraj[k])
            distance_p2p.append(distance_tmp)
        reward = np.nanmin(np.array(distance_p2p)) - 0.01 * n * np.random.random_sample()
        print('robot_step method:', 'object_state',object_state, 'reward', reward)
        return self.state, reward
    #end of robot_step method

    def get_robotState(self):
        #state_position = self.runUR5Moveit_obj.get_UR5State()
        return state_position
    #end of get_robotState method

    def get_referTraj(self):
        referTraj_pd = pd.read_csv('/home/yuchen/catkin_ws/src/demostration_learning/script/results/objectTraj_02.csv')
        referTraj = referTraj_pd.values
        referTraj = np.delete(np.array(referTraj), (0), axis=1) # delete index_col
        referTraj = referTraj[11:41]

        print('get_referTraj method of UR5Env:', 'referTraj', referTraj, referTraj.shape)
        return referTraj
    #end of get_referTraj method




''' test'''
if __name__ == '__main__':
    UR5Env_obj = UR5Env()

    UR5Env_obj.reset_ur5()

    robot_action = np.array([0.2, 0.15, 0.1])
    UR5Env_obj.robot_step(robot_action, 1)
