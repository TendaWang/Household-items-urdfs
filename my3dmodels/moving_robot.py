import math
import time

import numpy as np
import pybullet as p
import pybullet_data
from navi import get_trajectory
class Moving():
    def __init__(self):
        if not p.isConnected():
            p.connect(p.GUI)
            p.setGravity(0, 0, -9.8)
            p.setRealTimeSimulation(True)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            plane_id = p.loadURDF("plane.urdf", useMaximalCoordinates=True)
        self.position = np.array([0, -1, 0])
        self.load_robot()

        self.rotation=np.array([1.57,0,0])
    def load_robot(self):
        self.robot=p.loadURDF('urdf/moving_robot.urdf',basePosition=self.position,useFixedBase=True,globalScaling=0.5)
    def keyboardevent(self):
        keys = p.getKeyboardEvents()
        if p.B3G_DOWN_ARROW in keys :
            self.step(np.array([-0.01,0,0]),np.array([0,0,0]))
        if p.B3G_UP_ARROW in keys:
            self.step(np.array([0.01,0,0]),np.array([0,0,0]))
        if p.B3G_LEFT_ARROW in keys :
            self.step(np.array([0.0,0,0]),np.array([0,0,0.05]))
        if p.B3G_RIGHT_ARROW in keys :
            self.step(np.array([0.0,0,0]),np.array([0,0,-0.05]))

        time.sleep(0.01)
    def step(self,dpos,drot):
        self.position=self.position+dpos[0]*np.array([math.cos(self.rotation[2]),math.sin(self.rotation[2]),0])
        self.rotation=self.rotation+drot
        p.resetBasePositionAndOrientation(self.robot,self.position,p.getQuaternionFromEuler(self.rotation))

    def track(self,trajectory):
        """

        Parameters
        ----------
        trajectory: t[0]:x t[1]:y
        tracking a series points of trajectory
        Returns
        -------
        """
        tx=trajectory[0]
        ty=trajectory[1]
        start_pos=self.position
        for i in range(len(tx)):
            nextpoint=np.array([tx[i],ty[i],0])
            vector=nextpoint-start_pos
            euler=np.array([1.57,0,math.atan2(vector[1],vector[0])])
            p.resetBasePositionAndOrientation(self.robot,nextpoint,p.getQuaternionFromEuler(euler))
            time.sleep(0.1)
            start_pos=nextpoint



if __name__ == '__main__':

    robot=Moving()
    # t=np.zeros((2,200))
    # for i in range(200):
    #     t[0][i]=i*0.03
    #     t[1][i]=math.cos(i*0.04*3.14)
    t=get_trajectory([50,50],[50,200])
    for i in range(len(t[0])):
        t[0][i]=t[0][i]*0.04+6
        t[1][i]=t[1][i]*0.04+7
    robot.track(t)
    while p.isConnected():
        robot.keyboardevent()

        pass