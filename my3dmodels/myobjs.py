import time

import pybullet as p
import pybullet_data
class ObjLoader():
    def __init__(self,prefix=""):
        if not p.isConnected():
            p.connect(p.GUI)
            p.setGravity(0,0,-9.8)
            p.setRealTimeSimulation(True)
        self.objects_name=['apple']

        self.scale=1
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF('plane.urdf')

        self.allobjs=['apple','banana','bottle1','bottle2','carrot','coffee_cup','coffee_plate','cola','fork1','fork2',
                      'fork3','glass2','glass3','glass4','glass6','glass7','glass9','knife','mug','phone1',
                      'scissors','spoon','toothbrush','toothpaste','vase1']
        self.prefix=prefix
    def drawline(self):
        p.removeAllUserDebugItems()

        line_id = p.addUserDebugLine([0,0,0.8], [0.1,0,0.8], [1, 0, 0])
        line_id2 = p.addUserDebugLine([0,0,1.8], [0.1,0,1.8], [1, 0, 0])
        return line_id


    def load_obj(self,position,name):
        urdf_path=self.prefix+'urdf/'+str(name)+'.urdf'
        obj=p.loadURDF(urdf_path,basePosition=position,globalScaling=1.,useFixedBase=False)
        p.changeDynamics(obj, -1, lateralFriction=1,rollingFriction=0.01)
        return obj

    def load_on_table(self):
        p.loadURDF("table/table.urdf")
        for i in range(5):
            for j in range(5):
                position = [-0.5 + 0.2 * i, -0.5 * 0.2 * j, 0.66]
                self.load_obj(position, self.allobjs[i * 5 + j])

    def scene1(self):
        """
        load apple banana and a glass
        Returns
        -------

        """
        p.loadURDF("table/table.urdf")
        self.load_obj([0,0.1,0.66], self.allobjs[0])#apple
        self.load_obj([0, -0.1, 0.66], self.allobjs[4])#carrot
        self.load_obj([ -0.1,0, 0.66], self.allobjs[12])#glass
        self.load_obj([ -0.1,0, 0.66], self.allobjs[22])#toothbrush
        self.load_obj([ 0.1,0, 0.66], 'hotdog')#toothbrush
    def scene2(self):
        """
        load fork knife and bottle and carrot
        Returns
        -------

        """
        p.loadURDF("table/table.urdf")
        self.load_obj([0, -0.1, 0.66], self.allobjs[9])  # fork
        self.load_obj([0, 0.1, 0.66], self.allobjs[17])  # knife
        self.load_obj([0.1,0,  0.66], self.allobjs[3])  # bottle
        self.load_obj([-0.1,0,  0.66], self.allobjs[4])  # carrot

    def scene3(self):
        """
        load fork knife and bottle and carrot
        Returns
        -------

        """
        p.loadURDF("table/table.urdf")
        self.load_obj([0,0.1,0.66], self.allobjs[0])#apple
        self.load_obj([0, 0.1, 0.66], 'hotdog')
        self.load_obj([0.1,0,  0.66], self.allobjs[3])  # bottle
        self.load_obj([-0.1,0,  0.66], self.allobjs[4])  # carrot

    def scene4(self):
        p.loadURDF("table/table.urdf")
        self.load_obj([0, 0.1, 0.66], 'donuts')
        self.load_obj([0, -0.1, 0.66], 'apple')
        self.load_obj([0.1,0,  0.66], 'scissors')



if __name__ == '__main__':
    objloader=ObjLoader()

    # objloader.load_on_table()
    # objloader.load_obj([0,0,0],'hotdog')
    objloader.scene4()
    while p.isConnected():

        pass
        # p.stepSimulation()