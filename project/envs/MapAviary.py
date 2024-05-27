import numpy as np
import pybullet as p
from gymnasium import spaces
import pkg_resources
from project.envs.ProjAviary import ProjAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ImageType

class MapAviary(ProjAviary):
    "Multi-drone environment class for topological mapping of unknown places"

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 num_drones: int=1,
                 neighbourhood_radius: float=np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 pyb_freq: int = 240,
                 ctrl_freq: int = 240,
                 gui=True,
                 vision=False,
                 img_res: np.ndarray=np.array([64, 48]),
                 save_imgs=False,
                 obstacle_ids=False,
                 labyrinth_id: str = "0",
                 sensors_attributes = True,
                 max_sensors_range: float = np.inf,
                 ref_distance : int = 0,
                 s_WF: int = -1,
                 c_omega : float = 0,
                 c_vel: float = 0,
                 output_folder='results'
                 ):
        
        """Initialization of an environment with drones capable of performing topological mapping.

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
        pyb_freq : int, optional
            The frequency at which PyBullet steps (a multiple of ctrl_freq).
        ctrl_freq : int, optional
            The frequency at which the environment steps.
        gui : bool, optional
            Whether to use PyBullet's GUI.
        vision : bool, optional
            Whether to use the camera/vision sensor
        img_res: numpy array, optional
            Specify the resolution of the captured images
        save_imgs : bool, optional
            Whether to save pictures.
        obstacle_ids : list, optional
            Select how many and which obstacles/assets to add to the simulation
        labyrinth_id : string, optional
            Select the type of labyrinth to insert
        sensors_attributes : bool, optional
            mettere True se ci sono i RangeFinders
        max_sensors_range : float
            gittata RangeFinders (TODO CONTROLLARE SIA IN METRI)
        TODO: Aggiungere help delle variabili dopo max_sensors_range
        """        

        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=Physics.PYB,
                         pyb_freq=pyb_freq,
                         ctrl_freq=ctrl_freq,
                         gui=gui,
                         vision=vision,
                         img_res=img_res,
                         save_imgs=save_imgs,
                         obstacle_ids=obstacle_ids,
                         labyrinth_id=labyrinth_id,
                         sensors_attributes=sensors_attributes,
                         max_sensors_range=max_sensors_range,
                         output_folder=output_folder
                         )
        
        self.DIST_WALL_REF = ref_distance  # distanza di riferimento dal muro
        self.S_WF = s_WF             # wall following side: 1 for Right side, -1 for Left Side
        self.C_OMEGA = c_omega
        self.C_VEL=c_vel
                
    ################################################################################

    def NextWP(self): #Aggiungere gli input necessari
        """Definisce la logica di navigazione trovando il prossimo WP usando le funzioni:
        - _WallFollowing()
        - _WallFollowingAndAlign()
        - _DecisionSystem
        
        """

        #TODO: definire funzione

        pass

    ################################################################################

    def _decisionSystem(self): #Aggiungere gli input necessari
        """Definisce la logica di scelta della direzione da prendere dopo uno stop
        
        """

        #TODO: Definire funzione

        pass
    
    ################################################################################

    def _WallFollowing(self,state):
        """Funzione per seguire in modo allineato il muro -> tira fuori la rotazione desiderata da mandare ai controlli
        la varibile state prende tre possibili valori:
        - state = 0 ruotare per allinearsi al muro
        - state = 1 ruotare e seguire il muro
        - state = 2 ruotare attorno all'angolo
        
        """
        td = 0.02
        hit_distance,Hit_point,self.Min_dist= self._MinDistToWall(self)
        sWF = self.S_WF
        cw = self.C_OMEGA
        cv = self.C_VEL
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
                if dR < self.DIST_WALL_REF :
                    state = 0 #sono vicino al muro devo girare per allinearmi
                if self.rf == self.max_range :
                    state = 2 #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo
            elif state == 2 :
                vel.append([cv])
                omega.append([sWF[j]*cv/self.DIST_WALL_REF])
            if np.abs(self.rs - self.rf* np.cos(self.beta)) < 0.01 :
                    state = 1 # ho agirato l'angolo e conttinuo a seguire il muro
            if dR < self.DIST_WALL_REF:
                    state = 0 #sto girando vedo il muro ma non sono allineato, mi devo girare    
        
        return omega , vel , state
    
    ################################################################################

    def _WallFollowingandAllign(self):
        """funzione per seguire il muro -> mi da la omega mentre sto navigando vicino al muro per evitare di allontanarmi dal muro
        
        """
        td = 0.02
        omega=[]
        for j in range(self.NUM_DRONES):
            if np.abs(self.DIST_WALL_REF-self.Min_dist[j]) > td :#sono lontano da td
                if self.DIST_WALL_REF-self.Min_dist[j] > td :
                    omega.append([self.S_WF*self.C_OMEGA]) # molto lontano dal muro
                else : 
                    omega.append([-self.S_WF*self.C_OMEGA]) #troppo vicino al muro
            elif np.abs(self.DIST_WALL_REF-self.Min_dist[j]) < td :#sono vicino al td
                if self.rs[j] > self.rf[j]*np.cos(self.beta):
                      omega.append([self.S_WF*self.C_OMEGA])
                else : 
                     omega.append([-self.S_WF*self.C_OMEGA])
            else :
                 omega.append([0])
        
        return omega
    
    ################################################################################

    def _MinDistToWall(self):
        """Distanza minima dall'ostacolo 
        
        """
        observation, Hit_point = self._sensorsObs()
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
