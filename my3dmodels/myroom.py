import time
import math
import numpy as np
import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import cv2

from moving_robot import Moving
from navi import Plot_navi

class Mapping():
    def __init__(self):
        self.numRays = 512
        self.rayLen = 5
        self.mymap=np.zeros((1000,1400))

    def make_rays(self,pos):
        rayFrom=[]
        rayTo=[]

        for i in range(self.numRays):
            for j in range(-6, 6):
                rayFrom.append(pos)
                rayTo.append(np.array([
                    pos[0]+self.rayLen * math.sin(2. * math.pi * float(i) / self.numRays),
                    pos[1]+self.rayLen * math.cos(2. * math.pi * float(i) / self.numRays),
                    pos[2]+j*0.1])
                )
                # p.addUserDebugLine(pos, np.array([
                #     pos[0]+self.rayLen * math.sin(2. * math.pi * float(i) / self.numRays),
                #     pos[1]+self.rayLen * math.cos(2. * math.pi * float(i) / self.numRays),
                #     pos[2]+j*0.2]), [0, 0, 1])




        results = p.rayTestBatch(rayFrom, rayTo)
        for i in range(self.numRays*12):
            hitObjectUid = results[i][0]

            if not (hitObjectUid < 0):
                hitPosition = results[i][3]
                if hitPosition[2]<0.1 or hitPosition[2]>1.8:
                    continue
                map_posx=max(min(998,int((6+hitPosition[0])*100)),1)
                map_posy=max(min(1398,int((7+hitPosition[1])*100)),1)
                self.mymap[map_posx,map_posy]=hitPosition[2]
                # p.addUserDebugLine(rayFrom[i], hitPosition, [0,0,1])

        return results
    def mapping(self):

        for i in range(20):
            for j in range(28):

                pos=[-6+i*0.5,-7+j*0.5,0.6]
                houseloader.drawline(pos)
                self.make_rays(pos)

    def save_map(self):
        plt.matshow(self.mymap)
        plt.show()
        np.save('map.npy',self.mymap)
    def load_map(self,name='map.npy'):
        self.mymap=np.load(name)
        self.mymap=cv2.resize(self.mymap,(350,250))
    def adjust_map(self):
        newmap=np.zeros(self.mymap.shape,np.uint8)
        ox=[]
        oy=[]
        for i in range(newmap.shape[0]):
            for j in range(newmap.shape[1]):
                if self.mymap[i][j]>0:
                    newmap[i][j]=1
                    ox.append(i)
                    oy.append(j)



        kernel=np.ones((3,3),np.uint8)
        img=cv2.dilate(np.uint8(newmap * 255),kernel,iterations=1)
        cv2.imshow('1',img)
        cv2.waitKey(0)
        img = cv2.Canny(img, 100, 120)
        # img=cv2.resize(img,(350,250))
        cv2.imshow('1',img)
        cv2.waitKey(0)

        obstacle=np.array([ox,oy])
        np.save('obstacle.npy',obstacle)
        np.save('obstacle_img.npy',img)

class HouseLoader():
    def __init__(self):
        p.connect(p.GUI)
        p.setGravity(0,0,-9.8)
        p.setRealTimeSimulation(True)
        self.objects_name=['apple']
        self.scale=1
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF('plane.urdf')
        # p.loadURDF("table/table.urdf")
        self.allobjs=['apple','banana','bottle1','bottle2','carrot','coffee_cup','coffee_plate','cola',
                      'fork1','fork2','fork3','glass2','glass3','glass4','glass6','glass7','glass9',
                      'knife','mug','phone1','scissors','spoon','toothbrush','toothpaste','vase1']

    def drawline(self,line,line2=None):
        # p.removeAllUserDebugItems()
        if type(line2)==np.ndarray:
            line_id = p.addUserDebugLine(line, line2, [0,0,1],lineWidth=3)
        else:
            line_id = p.addUserDebugLine(line, line+np.array([0,0,2]), [1, 0, 0],lineWidth=3)
        return line_id
    def control_lines(self):
        self.line_desk=np.array([0, -4, 0.1])
        self.drawline(self.line_desk)
        self.line_bed=np.array([1, -5, 0.1])
        self.drawline(self.line_bed)
        self.drawline(self.line_desk,self.line_bed)
        self.line_door_bedroom=np.array([2.25,-3,0.1])
        self.drawline(self.line_door_bedroom)
        self.drawline(self.line_door_bedroom,self.line_bed)
        self.drawline(self.line_door_bedroom,self.line_desk)
        #livingroom
        self.line_table=np.array([2.25,-2,0.1])
        self.line_table2=np.array([-1.5,-2,0.1])
        # self.line_
        # self.line_
        # self.line_
        # self.line_
        self.line_door_bathroom=np.array([-2.25,-3,0.1])
        desk_bed=p.addUserDebugLine([0, -4, 0.], [0, -4, 2], [0, 0, 1],lineWidth=3)
        line_bed = p.addUserDebugLine([1, -5, 2], [1, -5, 0], [1, 0, 0],lineWidth=3)
        line_sofa = p.addUserDebugLine([0.5, 0, 2], [0.5, 0, 0], [1, 0, 0],lineWidth=3)
        line_table = p.addUserDebugLine([-1.5, 0, 2], [-1.5, 0, 0], [1, 0, 0],lineWidth=3)
        line_bath = p.addUserDebugLine([-2, -5, 2], [-2, -5, 0], [1, 0, 0],lineWidth=3)
        line_kitchen = p.addUserDebugLine([2, 4, 2], [2, 4, 0], [1, 0, 0],lineWidth=3)


    def load_house(self,position,quat,name,gl=None):
        urdf_path='urdf/'+str(name)+'.urdf'
        if gl==None:
            obj=p.loadURDF(urdf_path,basePosition=position,baseOrientation=quat,globalScaling=1,useFixedBase=True)
        else:
            obj=p.loadURDF(urdf_path,basePosition=position,baseOrientation=quat,globalScaling=gl,useFixedBase=True)
        print('load ok')
        return obj
    def load_living_room(self):
        quat=[0,0,0,1]
        self.load_house([-2.8, 0.7, 0],quat, 'chair')
        self.load_house([-2.8, -1, 0],quat, 'chair')
        self.load_house([0, 0, 0], quat,'living')

    def load_bedroom(self):
        quat=p.getQuaternionFromEuler([0,0,1.57])
        # self.load_house([-2.8, -1, 0], 'chair')
        self.load_house([1.2, -5, -0.08],quat, 'bedroom',gl=2.1)

    def load_bathroom(self):
        quat = p.getQuaternionFromEuler([0, 0, 3.14])
        # self.load_house([-2.8, -1, 0], 'chair')
        self.load_house([-3.1, -4.8, 0],quat, 'bathroom',gl=0.9)

    def load_kitchen(self):
        quat = p.getQuaternionFromEuler([0, 0, 3.14])
        # self.load_house([-2.8, -1, 0], 'chair')
        self.load_house([0,3.7, -0.3], quat, 'kitchen', gl=0.4)



if __name__ == '__main__':
    # maper = Mapping()
    # maper.load_map()
    # maper.adjust_map()
    # maper.save_map()



    houseloader=HouseLoader()
    #
    houseloader.load_bathroom()
    houseloader.load_bedroom()
    houseloader.load_kitchen()
    houseloader.load_living_room()
    #
    #
    robot=Moving()
    # time.sleep(1)
    # maper=Mapping()
    # maper.mapping()
    # maper.save_map()
    plot_navi=Plot_navi([125,125],robot=robot)

    #
    # trajx=[]
    # trajy=[]
    # for i in range(len(t[0])-1,-1,-1):
    #     trajx.append(t[1][i] * 0.04 - 6)
    #     trajy.append(t[0][i] * 0.04 - 7)
    # robot.track([trajx,trajy])
    while p.isConnected():
        robot.keyboardevent()
        time.sleep(0.001)
        pass