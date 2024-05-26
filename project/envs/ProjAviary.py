import os
import numpy as np
import pybullet as p
from gymnasium import spaces
import pkg_resources

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ImageType

class ProjAviary(CtrlAviary):
    """Multi-drone environment class for topological mapping"""

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
                 vision=False,
                 save_imgs=False,
                 obstacle_ids=False,
                 labyrinth_id: str = "0",
                 sensors_attributes = True,
                 max_sensors_range: float = np.inf,
                 #ref_distance : int = 0,
                 #s_WF: int = -1,
                 #c_omega : float = 0,
                 #c_vel: float = 0,
                 output_folder='results',
                 img_res: np.ndarray=np.array([64, 48])
                 ):
        """Initialization of an environment with drones capable of performing topological mapping.

        Attributes `vision_attributes` is selected
        based on the choice of `obs`; `obstacles` is set to True 
        and overridden with landmarks for mapping applications

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
        vision : bool, optional
            Whether to use the camera/vision sensor
        save_imgs : bool, optional
            Whether to save pictures.
        obstacle_ids : list, optional
            Select how many and which obstacles/assets to add to the simulation
        sensors_attributes : bool, optional
            Whether to add rangefinders to the drone
        img_res: numpy array, optional
            Specify the resolution of the captured images
        sensors_attributes : bool, optional
            mettere True se ci sono i RangeFinders
        max_sensors_range : float
            gittata RangeFinders (TODO CONTROLLARE SIA IN METRI)
        """

        ####
        self.VISION = vision
        self.SAVE = save_imgs
        self.OBSTACLE_IDS = obstacle_ids
        self.LABYRINTH_ID = labyrinth_id
        super().__init__(drone_model=drone_model,
                         num_drones=num_drones,
                         neighbourhood_radius=neighbourhood_radius,
                         initial_xyzs=initial_xyzs,
                         initial_rpys=initial_rpys,
                         physics=physics,
                         pyb_freq=pyb_freq,
                         ctrl_freq=ctrl_freq,
                         gui=gui,
                         record=False, 
                         obstacles=True, 
                         user_debug_gui=True,
                         vision_attributes=vision,
                         output_folder=output_folder
                         )
        
        #### Set the image resoution to the desired one
        self.IMG_RES = img_res

        ### Create attributes for sensors ####################
        # Aggiungo tot sensori al drone
        self.SENSOR_ATTR = sensors_attributes
        self.max_range = max_sensors_range   # max_sensors_range
        self.NUM_SENSORS = 4
        if self.SENSOR_ATTR :
            self.sensor_position =np.zeros((4,3))
            self.sensor_direction = np.array([
                                        [1.0, 0.0, 0.0],  # front
                                        [0.0, 1.0, 0.0],  # Lx                                        
                                        [-1.0, 0.0, 0.0],   # behind
                                        [0.0, -1.0, 0.0],  # Rx
                                        ])   
            # TODO piccolo check sul se serve inizializzare rot_mat    
        
    ###################################################################
        
    def step(self,
             action
             ):
        """Advances the environment by one simulation step."""
        # Advance the simulation like in BaseAviary
        obs, reward, terminated, truncated, info = super().step(action)

        #Add output from rangefinders 
        obs_sensors=self._sensorsObs()

        # TODO aggiungere il richiamo a _storeHitPoints(self)

        return obs, obs_sensors, reward, terminated, truncated, info
    
    ###################################################################
        
    def _addObstacles(self):
        """Add obstacles to the environment.

        Only if the observation is of type RGB, landmarks are added.
        Overrides BaseAviary's method.
        
        """
        #TODO: define the logic through which we select the obstacles using the self.OBSTACLE_IDS variable
        # for the moment the function is the same as in BaseRLAviary.py
        if self.VISION :
            if self.LABYRINTH_ID == "0":
                p.loadURDF("block.urdf",
                           [1, 0, .1],
                           p.getQuaternionFromEuler([0, 0, 0]),
                           physicsClientId=self.CLIENT
                           )
                p.loadURDF("cube_small.urdf",
                           [0, 1, .1],
                           p.getQuaternionFromEuler([0, 0, 0]),
                           physicsClientId=self.CLIENT
                           )
                p.loadURDF("duck_vhacd.urdf",
                           [-1, 0, .1],
                           p.getQuaternionFromEuler([0, 0, 0]),
                           physicsClientId=self.CLIENT
                           )
                p.loadURDF("teddy_vhacd.urdf",
                           [0, -1, .1],
                           p.getQuaternionFromEuler([0, 0, 0]),
                           physicsClientId=self.CLIENT
                           )
            else:
                p.loadURDF(pkg_resources.resource_filename('project','assets/'+'labyr'+self.LABYRINTH_ID+'.urdf'),
                       [0, 0, 0],
                       p.getQuaternionFromEuler([0,0,0]),
                       useFixedBase = 1
                       )
        else:
            pass

    # TODO def _addDecorations(self):
    # funzione più vasta di addObstacles che possa caricare vari dettagli che 
    # aiutino la caratterizzazione dell'environment per il riconoscimento

    ################################################################################
        
    def _actionSpace(self):
        """Returns the action space of the environment.
        Copiata uguale da CtrlAviary.py
        Returns
        -------
        spaces.Box
            An ndarray of shape (NUM_DRONES, 4) for the commanded RPMs.

        """
        #### Action vector ######## P0            P1            P2            P3
        act_lower_bound = np.array([[0.,           0.,           0.,           0.] for i in range(self.NUM_DRONES)])
        act_upper_bound = np.array([[self.MAX_RPM, self.MAX_RPM, self.MAX_RPM, self.MAX_RPM] for i in range(self.NUM_DRONES)])
        return spaces.Box(low=act_lower_bound, high=act_upper_bound, dtype=np.float32)
    
    ################################################################################
    
    def _computeObs(self):
        """Returns the current observation of the environment. 
        If the observation type is RGB, it saves the images captured by the drones.
        Returns
        -------
        ndarray
            An ndarray of shape (NUM_DRONES, 20) with the state of each drone.

        """
        if self.VISION:
            if self.step_counter%self.IMG_CAPTURE_FREQ == 0:
                for i in range(self.NUM_DRONES):
                    self.rgb[i], self.dep[i], self.seg[i] = self._getDroneImages(i,
                                                                                 segmentation=False
                                                                                 )
                    #### Printing observation to PNG frames example ############
                    if self.SAVE:
                        self._exportImage(img_type=ImageType.RGB,
                                          img_input=self.rgb[i],
                                          path=self.ONBOARD_IMG_PATH+"/drone_"+str(i),
                                          frame_num=int(self.step_counter/self.IMG_CAPTURE_FREQ)
                                          )
            return np.array([self._getDroneStateVector(i) for i in range(self.NUM_DRONES)])
        else:
            return np.array([self._getDroneStateVector(i) for i in range(self.NUM_DRONES)])
        
    ################################################################################
        
    def _sensorsObs(self,
                    nth_drone):
        '''Returns distance from obstacles in the 4 directions # front Rx behind Lx 
        and cloud of up to 4 hit points

        Parameters
        ----------
        nth_drone : int
            The ordinal number/position of the desired drone in list self.DRONE_IDS.

        Returns
        -------
        observation 
            numpy array con le distanze dal primo ostacolo nelle 4 direzioni
        Hit_point 
            numpy array bidimensionale (lista di punti rilevati dai rangefinders)
            TODO: sviluppare qualcosa che salva i 4 punti in Hit_point a ogni step
        '''
        observation = np.array([[self.max_range,self.max_range,self.max_range,self.max_range] for j in range(self.NUM_DRONES)] )
        Hit_point = []
        if self.SENSOR_ATTR:
            for i in range(self.NUM_DRONES):
                closest_hit = []
                for j in range(self.NUM_SENSORS) :
                    self.sensor_position = self._getDroneStateVector(i)[0:3]  # [1. 1. 1.] posizione
                    self.rot_mat = np.array(p.getMatrixFromQuaternion(self.quat[i, :])).reshape(3, 3) #TODO vedi ste cazzo di rotazioni
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

    def _getDroneImages(self,
                        nth_drone,
                        segmentation: bool=True
                        ):
        """Returns camera captures from the n-th drone POV (camera pointing downwards).

        Parameters
        ----------
        nth_drone : int
            The ordinal number/position of the desired drone in list self.DRONE_IDS.
        segmentation : bool, optional
            Whehter to compute the compute the segmentation mask.
            It affects performance.

        Returns
        -------
        ndarray 
            (h, w, 4)-shaped array of uint8's containing the RBG(A) image captured from the n-th drone's POV.
        ndarray
            (h, w)-shaped array of uint8's containing the depth image captured from the n-th drone's POV.
        ndarray
            (h, w)-shaped array of uint8's containing the segmentation image captured from the n-th drone's POV.

        """
        if self.IMG_RES is None:
            print("[ERROR] in BaseAviary._getDroneImages(), remember to set self.IMG_RES to np.array([width, height])")
            exit()
        rot_mat = np.array(p.getMatrixFromQuaternion(self.quat[nth_drone, :])).reshape(3, 3)
        #### Set target point, camera view and projection matrices #
        target = np.dot(rot_mat,np.array([0.1, 0, -2000])) + np.array(self.pos[nth_drone, :]) 
        DRONE_CAM_VIEW = p.computeViewMatrix(cameraEyePosition=self.pos[nth_drone, :]+np.array([0, 0, self.L-0.2]),
                                             cameraTargetPosition=target,
                                             cameraUpVector=[0, 0, 1],
                                             physicsClientId=self.CLIENT
                                             )
        DRONE_CAM_PRO =  p.computeProjectionMatrixFOV(fov=60.0,
                                                      aspect=1.0,
                                                      nearVal=self.L,
                                                      farVal=1000.0
                                                      )
        SEG_FLAG = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX if segmentation else p.ER_NO_SEGMENTATION_MASK
        [w, h, rgb, dep, seg] = p.getCameraImage(width=self.IMG_RES[0],
                                                 height=self.IMG_RES[1],
                                                 shadow=1,
                                                 viewMatrix=DRONE_CAM_VIEW,
                                                 projectionMatrix=DRONE_CAM_PRO,
                                                 flags=SEG_FLAG,
                                                 physicsClientId=self.CLIENT
                                                 )
        rgb = np.reshape(rgb, (h, w, 4))
        dep = np.reshape(dep, (h, w))
        seg = np.reshape(seg, (h, w))
        return rgb, dep, seg
    
    ################################################################################

    def _computeReward(self):
        """Computes the current reward value(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        int
            Dummy value.

        """
        return -1

    ################################################################################
    
    def _computeTerminated(self):
        """Computes the current terminated value(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        bool
            Dummy value.

        """
        return False
    
    ################################################################################
    
    def _computeTruncated(self):
        """Computes the current truncated value(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        bool
            Dummy value.

        """
        return False

    ################################################################################
    
    def _computeInfo(self):
        """Computes the current info dict(s).

        Unused as this subclass is not meant for reinforcement learning.

        Returns
        -------
        dict[str, int]
            Dummy value.

        """
        return {"answer": 42} #### Calculated by the Deep Thought supercomputer in 7.5M years

    def _storeHitPoints(self,
                        nth_drone):
        """Saves Cloud of points from Hit_point 

        Returns
        -------
        QUALCOSA
        """
        return {"answer": 42}
