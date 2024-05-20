import pybullet as p
import os

class URDFLoader:
    def __init__(self, physics_client_id=0):
        """
        Inizializza l'URDFLoader.

        :param physics_client_id: ID del client fisico PyBullet. Default è 0.
        """
        self.physics_client_id = physics_client_id

    def load_urdfs_with_params(self, urdf_params):
        """
        Carica i file URDF utilizzando i parametri specificati.

        :param urdf_params: Lista di dizionari con i parametri per ciascun URDF.
        :return: Lista di ID degli oggetti caricati.
        """
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

    # Iniziare la simulazione
    for _ in range(10000):
        p.stepSimulation()

    # Disconnettersi da PyBullet
    p.disconnect()