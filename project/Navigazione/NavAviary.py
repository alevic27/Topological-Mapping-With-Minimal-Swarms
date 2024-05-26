import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet as p
from gymnasium import spaces

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

class NavAviary(CtrlAviary):
    """Multi-drone environment class for control applications."""

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 num_drones: int=1,
                 neighbourhood_radius: float=np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics=Physics.PYB,
                 pyb_freq: int = 240,
                 ctrl_freq: int = 240,
                 gui=False,
                 record=False,
                 obstacles=False,
                 user_debug_gui=True,
                 sensors_attributes = False,
                 max_sensors_range: float = np.inf,
                 ref_distance : int = 0,
                 s_WF: int = -1,
                 c_omega : float = 0,
                 c_vel: float = 0,
                 output_folder='results'
                 ):
        """Initialization of an aviary environment for control applications.

        Parameters
        ----------
        drone_model : DroneModel, optional
            The desired drone type (detailed in an .urdf file in folder `assets`).
        num_drones : int, optional
            The desired number of drones in the aviary.
        neighbourhood_radius : float, optional
            Radius used to compute the drones' adjacency matrix, in meters.
        initial_xyzs: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial XYZ position of the drones.
        initial_rpys: ndarray | None, optional
            (NUM_DRONES, 3)-shaped array containing the initial orientations of the drones (in radians).
        physics : Physics, optional
            The desired implementation of PyBullet physics/custom dynamics.
        pyb_freq : int, optional
            The frequency at which PyBullet steps (a multiple of ctrl_freq).
        ctrl_freq : int, optional
            The frequency at which the environment steps.
        gui : bool, optional
            Whether to use PyBullet's GUI.
        record : bool, optional
            Whether to save a video of the simulation.
        obstacles : bool, optional
            Whether to add obstacles to the simulation.
        user_debug_gui : bool, optional
            Whether to draw the drones' axes and the GUI RPMs sliders.

        """
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         pyb_freq=pyb_freq,
                         ctrl_freq=ctrl_freq,
                         gui=gui,
                         record=record,
                         obstacles=obstacles,
                         user_debug_gui=user_debug_gui,
                         output_folder=output_folder
        )
      # Aggiungo tot sensori al drone
        self.SENSOR_ATTR = sensors_attributes
        self.max_range = max_sensors_range #max_sensors_range
        self.distWallRef = ref_distance  # distanza di riferimento dal muro
        self.sWF = s_WF             # wall following side: 1 for Right side, -1 for Left Side
        self.comega = c_omega
        self.cvel=c_vel
        self.NUM_SENSORS = 4
        if self.SENSOR_ATTR :
            self.sensor_position =np.zeros((4,3))
            self.sensor_direction = np.array([
                                        [1.0, 0.0, 0.0],  # front
                                        [0.0, 1.0, 0.0],  # Rx
                                        [-1.0, 0.0, 0.0],   # behind
                                        [0.0, -1.0, 0.0],  # Lx
                                        ])
            
        ################################################################################
        
        ################################################################################
    def _sensorsObs(self):
        # 
        observation = np.array([[self.max_range,self.max_range,self.max_range,self.max_range] for j in range(self.NUM_DRONES)] )
        Hit_point = []
        if self.SENSOR_ATTR:
            for i in range(self.NUM_DRONES):
                closest_hit = []
                for j in range(self.NUM_SENSORS) :
                    self.sensor_position = self._getDroneStateVector(i)[0:3]  # [1. 1. 1.] posizione
                    self.rot_mat = np.array(p.getMatrixFromQuaternion(self.quat[i, :])).reshape(3, 3)
                    self.sensor_direction = np.dot(self.sensor_direction,self.rot_mat)
                    self.from_positions = np.tile(self.sensor_position, (4, 1))
                    self.to_positions = np.array([self.sensor_position + direction * self.max_range[0] for direction in self.sensor_direction])
                    result =p.rayTestBatch(self.from_positions, self.to_positions)
                    ##  Hit fraction: {result[2]}")
                    ##  Hit position: {result[3]}")
                    closest_hit.append(result[0])
                        #closest_hit[j][0] = result_list[j][0]
                    # Se c'è un punto di contatto, ottieni la distanza
                    if closest_hit[j][0] != -1:
                        hit_distance = closest_hit[j][2]*self.max_range[0]
                        hit_point = closest_hit[j][3]
                        # Aggiungi la distanza rilevata come osservazione
                        observation[i][j]= hit_distance
                        Hit_point.append(hit_point)
                    else:
                        # Se non ci sono oggetti rilevati, assegna una distanza massima
                        observation[i][j] =  self.max_range[0]
                        #Hit_point.append([])
        return observation, Hit_point
     
    ################################################################################
    # distanza minima dall'ostacolo 
    def _MinDistToWall(self):
        observation, Hit_point = _sensorsObs(self)
        distance=[]
        for i in range(self.NUM_DRONES):
            for j in range(observation[i][0 : 19]):
                if observation[i][j] < self.max_range:
                   hit_point = Hit_point[i][j]
            if hit_point == []:
                distance.append ([])
            else : 
                idx = np.random.choice(len(hit_point), 2, replace=False)
                x1, y1 = hit_point[idx[0]][0:1]
                x2, y2 = hit_point[idx[1]][0:1]

                # Calcola l'equazione della retta usando i due punti
                slope = (y2 - y1) / (x2 - x1)
                intercept = y1 - slope * x1

                # Valuta la distanza da ogni punto alla retta
                distance.append([np.abs(slope * x1 - y1 + intercept) / np.sqrt(slope**2 + 1)])
        return observation, Hit_point , distance
    ################################################################################
    # funzione per seguire il muro -> mi da la omega mentre sto navigando vicino al muro 
    # per evitare di allontanarmi dal muro
    def _WallFollowingandAllign(self):
        td = 0.02
        omega=[]
        for j in range(self.NUM_DRONES):
            if np.abs(self.distWallRef-self.Min_dist[j]) > td :#sono lontano da td
                if self.distWallRef-self.Min_dist[j] > td :
                    omega.append([self.sWF*self.comega]) # molto lontano dal muro
                else : 
                    omega.append([-self.sWF*self.comega]) #troppo vicino al muro
            elif np.abs(self.distWallRef-self.Min_dist[j]) < td :#sono vicino al td
                if self.rs[j] > self.rf[j]*np.cos(self.beta):
                      omega.append([self.sWF*self.comega])
                else : 
                     omega.append([-self.sWF*self.comega])
            else :
                 omega.append([0])
        return omega
     
    ################################################################################     
    # funzione per seguire in modo allineato il muro -> tira fuori la rotazione 
    # desiderata da mandare ai controlli
    ''' la varibile rotate prende tre possibili valori
        state = 0 ruotare per allinearsi al muro
        state = 1 ruotare e seguire il muro
        state = 2 ruotare attorno all'angolo
    '''

    def _WallFollowing(self,state):
        td = 0.02
        hit_distance,Hit_point,self.Min_dist= self._MinDistToWall(self)
        sWF = self.sWF
        cw =self.comega
        cv =self.cvel
        self.beta = np.deg2rad(90 - self.alfa/2) #angolo tra rf e rs 
        omega = []
        vel = []
        self.rf = [] 
        self.rs = [] 
        for j in range(self.NUM_DRONES) :

            if hit_distance[j][1]!= self.max_range : 
                self.rf.append(hit_distance[j][1]) # due range finders laterai
                self.rs.append(hit_distance[j][21])
            elif hit_distance[j][20]!=self.max_range :
                self.rf.append(hit_distance [j][20])  #raggi esterni stereocamera/sensore di profondità 
                self.rs.append(hit_distance[j][21])
            else :
                self.rf.append([self.max_range])
                self.rs.append([self.max_range])
            dR = self.Min_dist #minima distanza dal muro calcolata con RANSAC
            if state == 0 :#ruoto per allinearmi al muro
                omega.append([-1*sWF[j]*cw])
                vel.append([0])
                if np.abs(self.rs - self.rf * np.cos(self.beta)) < td :
                    state = 1 # mi sono ruotato e ora voglio seguire il muro
                if self.rf == self.max_range :
                    state = 2 #mi giro ma non vedo più il muro quindi ci sta un angolo
            elif state == 1 :#ruotare e seguire il muro
                vel.append([cv])
                Omega = self._WallFollowingandAllign(self)
                omega.append(Omega)
                if dR < self.distWallRef :
                    state = 0 #sono vicino al muro devo girare per allinearmi
                if self.rf == self.max_range :
                    state = 2 #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo
            elif state == 2 :
                vel.append([cv])
                omega.append([sWF[j]*cv/self.distWallRef])
            if np.abs(self.rs - self.rf* np.cos(self.beta)) < 0.01 :
                    state = 1 # ho agirato l'angolo e conttinuo a seguire il muro
            if dR < self.distWallRef:
                    state = 0 #sto girando vedo il muro ma non sono allineato, mi devo girare    
        return omega , vel , state
    ################################################################################
   
   
    