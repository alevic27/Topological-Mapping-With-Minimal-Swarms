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
# p.loadURDF("plane.urdf" , [0, 0, 0] , [0 , 0 , 0 , 1]) # orientation as a quaternion you can use p.getQuaternionFromEuler
# loadURDF(...)
#         bodyUniqueId = loadURDF(fileName, basePosition=[0.,0.,0.], baseOrientation=[0.,0.,0.,1.], useMaximalCoordinates=0, useFixedBase=0, flags=0, globalScaling=1.0, physicsClientId=0)
#         Create a multibody by loading a URDF file.
# planeId = p.loadURDF("plane.urdf")
# rd2d = p.loadURDF("r2d2.urdf"  , [0, 0, 1] , [0 , 0 , 0 , 1], useFixedBase = 0)
   # ROBOT_1 = p.loadURDF("../assets/box.urdf",[0,0,0], p.getQuaternionFromEuler([0,0,0]), physicsClientId=PYB_CLIENT)

# labyr = p.loadURDF("labyr6.urdf" , [0, 0, 1] , [0 , 0 , 0 , 1], useFixedBase = 0)
plane = p.loadURDF("plane.urdf")
'''
labyr = p.loadURDF("labyr6.urdf",
                   [0, 0, 0],
                   p.getQuaternionFromEuler([0,0,10]),
                   useFixedBase = 1
                   )
'''
p.loadURDF("sphere2.urdf",
                   [0, 2, .5],
                   p.getQuaternionFromEuler([0,0,0]),
                   )
# test = p.loadURDF("C:\Users\gabri\Desktop\Topological_mapping\models\assets\hb.urdf"  , [0, 0, 1] , [0 , 0 , 0 , 1], useFixedBase = 0)
# each URDF is a set of links and a baselink, useFixedBase glues the base to the floor.
# target_id = labyr
target_id = [0, 0, 0]

DRONE_CAM_VIEW = p.computeViewMatrix(cameraEyePosition=[0, 0, 5],
                                             cameraTargetPosition=target_id,
                                             cameraUpVector=[0, 0, 1],                                             
                                             )

DRONE_CAM_PRO =  p.computeProjectionMatrixFOV(fov=60.0,
                                                    aspect=1.0,
                                                    nearVal=1, #messo a mano
                                                    farVal=1000.0
                                                      )
#SEG_FLAG = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX if segmentation else p.ER_NO_SEGMENTATION_MASK
[w, h, rgb, dep, seg] = p.getCameraImage(width=1080,
                                                 height=760,
                                                 shadow=1,
                                                 viewMatrix=DRONE_CAM_VIEW,
                                                 projectionMatrix=DRONE_CAM_PRO,
                                                 #flags=SEG_FLAG,
                                                 #physicsClientId=self.CLIENT
                                                 )
rgb = np.reshape(rgb, (h, w, 4))
dep = np.reshape(dep, (h, w))
seg = np.reshape(seg, (h, w))
#return rgb, dep, seg
(Image.fromarray(img_input.astype('uint8'), 'RGBA')).save(os.path.join(path,"frame_"+str(frame_num)+".png"))



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



