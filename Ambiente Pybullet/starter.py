import numpy as np
import pybullet as p
import pybullet_data 
import time


physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version (useful for training purpose)
p.resetSimulation
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally, 
#but makes it so that you don't have to indicate source for urdf files
p.setGravity(0,0,-9.81)
p.setRealTimeSimulation(0)
p.setTimeOut(10)

# LOAD ASSETS
p.loadURDF("plane.urdf" , [0, 0, 0] , [0 , 0 , 0 , 1]) # orientation as a quaternion you can use p.getQuaternionFromEuler
# loadURDF(...)
#         bodyUniqueId = loadURDF(fileName, basePosition=[0.,0.,0.], baseOrientation=[0.,0.,0.,1.], useMaximalCoordinates=0, useFixedBase=0, flags=0, globalScaling=1.0, physicsClientId=0)
#         Create a multibody by loading a URDF file.
# planeId = p.loadURDF("plane.urdf")
rd2d = p.loadURDF("r2d2.urdf"  , [0, 0, 1] , [0 , 0 , 0 , 1], useFixedBase = 0)
# test = p.loadURDF("C:\Users\gabri\Desktop\Topological_mapping\models\assets\hb.urdf"  , [0, 0, 1] , [0 , 0 , 0 , 1], useFixedBase = 0)
# each URDF is a set of links and a baselink, useFixedBase glues the base to the floor.
target_id = rd2d

print('il numero di joints è', p.getNumJoints(target_id))

# BOX  per estrarre le info su ogni joint
for i in range(p.getNumJoints(target_id)):
    print(p.getJointInfo(target_id, i))

for step in range(10000):
    position_of_target , orientation_of_target =    p.getBasePositionAndOrientation(target_id)
    #p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=180, cameraPitch=-40, cameraTargetPosition=position_of_target)
    p.stepSimulation()
    time.sleep(.01)

p.disconnect()



