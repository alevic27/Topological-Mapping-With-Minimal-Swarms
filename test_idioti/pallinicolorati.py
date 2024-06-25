import pybullet as p
import pybullet_data
import numpy as np
import time

# Configurazione di PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Carica un piano
plane_id = p.loadURDF("plane.urdf")

# Lista dei punti da visualizzare
points = [
    [3.3, 2.1, 1.5],
    [3.3, 2.3, 1.5],
    [4.3, 3.1, 2.5],
    [4.5, 3.3, 2.5],
    [1.0, 1.0, 1.0]
]

# Funzione per aggiungere pallini grafici senza collisioni
def add_visual_ball(position, radius=0.03, color=[1, 0, 0, 1]):
    visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    ball_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=position)
    return ball_id

# Funzione per aggiungere una linea grafica senza collisioni tra due punti
def add_visual_line(point_a, point_b, color=[0, 0, 1, 1], width=1):
    line_id = p.addUserDebugLine(point_a, point_b, color, width)
    return line_id

# Aggiungi pallini grafici ai punti
for point in points:
    add_visual_ball(position=point, color=[0, 1, 0, 1])  # Verde

# Aggiungi una linea grafica tra due punti
add_visual_line([3.3, 2.1, 1.5], [3.3, 2.3, 1.5], color=[0, 255, 0], width=3)  # Rosso

# Mantieni la simulazione attiva per vedere i pallini e la linea
while True:
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()