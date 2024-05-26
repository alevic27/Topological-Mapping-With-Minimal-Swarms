import os
import sys
import numpy as np
import pkg_resources
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

# topo_path = [string for string in sys.path if string.endswith('Topological-Mapping-With-Minimal-Swarms')]
# p.setAdditionalSearchPath(topo_path[0])

pkg_path = 'project'
environment = 'labyr6.urdf'




labyr = p.loadURDF(pkg_resources.resource_filename('project', 'assets/'+environment),
                   [0, 0, 0],
                   p.getQuaternionFromEuler([0,0,10]),
                   useFixedBase = 0
                   )
p.loadURDF("cube_small.urdf",
                       [0, 1, 0.5],
                       p.getQuaternionFromEuler([0, 0, 0]),

                       )        
"""
####################################################################
# voglio creare un dizionario come input per la classe addobstacle
# usando self.OBSTACLE_IDS come input

qwer = p.loadURDF(  pkg_resources.resource_filename('project', 'assets/'+environment),
                    basePosition=[0, 0, 0],
                    baseOrientation=[0, 0, 0, 1],
                    useMaximalCoordinates=False,
                    globalScaling=1.0,
                    physicsClientId=0,
                    flags=0,
                    useFixedBase=False,
                    urdfFileName=None,
                    useMultiBody=True)


        ### creo la parte che carica dizionari, poi riaggiusta i vari if 

    def load_urdfs_with_params(self, urdf_params):
        
        Carica i file URDF utilizzando i parametri specificati.

        :param urdf_params: Lista di dizionari con i parametri per ciascun URDF.
        :return: Lista di ID degli oggetti caricati.
        
        loaded_objects = []

        for params in urdf_params:
            file_path = params['fileName']
            base_position = params.get('basePosition', [0, 0, 0])
            base_orientation = params.get('baseOrientation', [0, 0, 0, 1])
            use_fixed_base = params.get('useFixedBase', False)
            global_scaling = params.get('globalScaling', 1.0)

            obj_id = p.loadURDF(
                fileName=file_path,
                basePosition=base_position,
                baseOrientation=base_orientation,
                useFixedBase=use_fixed_base,
                globalScaling=global_scaling,
                physicsClientId=self.physics_client_id
            )
            loaded_objects.append(obj_id)
        
        return loaded_objects

# Utilizzo della classe URDFLoader
if __name__ == "__main__":
    # Connetti a PyBullet
    p.connect(p.GUI)

    # Definisci i parametri per ciascun URDF
    urdf_params = [
        {
            'fileName': 'path/to/first.urdf',
            'basePosition': [0, 0, 0.5],
            'baseOrientation': p.getQuaternionFromEuler([0, 0, 0]),
            'useFixedBase': True,
            'globalScaling': 1.0
        },
        {
            'fileName': 'path/to/second.urdf',
            'basePosition': [1, 0, 0.5],
            'baseOrientation': p.getQuaternionFromEuler([0, 0, 0]),
            'useFixedBase': False,
            'globalScaling': 1.0
        },
        {
            'fileName': 'path/to/third.urdf',
            'basePosition': [2, 0, 0.5],
            'baseOrientation': p.getQuaternionFromEuler([0, 0, 0]),
            'useFixedBase': False,
            'globalScaling': 1.0
        }
    ]

    # Crea un'istanza di URDFLoader
    urdf_loader = URDFLoader()

    # Carica gli URDF utilizzando i parametri specificati
    loaded_objects = urdf_loader.load_urdfs_with_params(urdf_params)

####################################################################
"""

# p.loadURDF("sphere2.urdf",
#                   [0, 2, .5],
#                   p.getQuaternionFromEuler([0,0,0]),
#                   )
# test = p.loadURDF("C:\Users\gabri\Desktop\Topological_mapping\models\assets\hb.urdf"  , [0, 0, 1] , [0 , 0 , 0 , 1], useFixedBase = 0)
# each URDF is a set of links and a baselink, useFixedBase glues the base to the floor.
target_id = labyr
# target_id = [0, 0, 0]


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



