import pybullet as p
import pybullet_data
import time
import numpy as np
# Connettersi a PyBullet in modalità GUI
p.connect(p.GUI)

# Aggiungere il percorso ai dati di PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
# Caricare un piano per riferimento
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("cube.urdf",
                           [2, 0, 0.5],
                           p.getQuaternionFromEuler([0, 0, 0]),                           
                           )
# settare la cam 
camera_distance = 2
camera_yaw = -90
camera_pitch = -30 # (- np.pi/6)
camera_target_position =   [ 0 , 0 , 0.5 ]      # self.initial_xyzs
p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, camera_target_position)


# Impostare la posizione della terna dimensionale
origin = [0, 0, 0]

# Definire la lunghezza degli assi
axis_length = 1.0

# Colori degli assi (rosso per X, verde per Y, blu per Z)
colors = [
    [1, 0, 0],  # Rosso per l'asse X
    [0, 1, 0],  # Verde per l'asse Y
    [0, 0, 1]   # Blu per l'asse Z
]

# Definire i punti finali degli assi
axes = [
    [axis_length, 0, 0],  # Asse X
    [0, axis_length, 0],  # Asse Y
    [0, 0, axis_length]   # Asse Z
]

# test raytest
#from_positions = np.array([0.0, 0.0, 0.1])
#to_positions = np.array([3.0, 0.0, 0.1])
#result =p.rayTest(from_positions, to_positions)
#

from_positions = np.array([0.0, 0.0, 0.1])
from_positions = np.vstack((from_positions, from_positions, from_positions))
a = np.array([3.0, 0.0, 0.1])
b = np.array([0.0, -3.0, 0.1])
c = np.array([0.0, 3.0, 0.1])
to_positions = np.vstack((a, b, c))
result =p.rayTestBatch(from_positions, to_positions)
print(result)

# Aggiungere linee di debug per ciascun asse
for i in range(3):
    p.addUserDebugLine(origin, axes[i], colors[i], lineWidth=3)

# Eseguire la simulazione per qualche passo per vedere il risultato
for _ in range(240*10):
    p.stepSimulation()
    time.sleep(1./240.)

# Disconnettersi da PyBullet
p.disconnect()