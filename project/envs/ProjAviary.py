import numpy as np
import pybullet as p
from gymnasium import spaces
import pkg_resources
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ImageType

class ProjAviary(CtrlAviary):
    """Multi-drone environment class for simulation of Crazyflie drones inside labyrinths"""

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
                 gui=True,
                 vision=False,
                 img_res: np.ndarray=np.array([64, 48]),
                 save_imgs=False,
                 obstacle_ids=False,
                 labyrinth_id: str = "0",
                 sensors_attributes = True,
                 max_sensors_range: float = np.inf,
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
        """

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
                         output_folder=output_folder
                         )   

        #### Create attributes for vision tasks ####################
        if self.VISION:
            self.IMG_RES = img_res
            self.IMG_FRAME_PER_SEC = 24
            self.IMG_CAPTURE_FREQ = int(self.PYB_FREQ/self.IMG_FRAME_PER_SEC)
            if self.IMG_CAPTURE_FREQ%self.PYB_STEPS_PER_CTRL != 0:
                print("[ERROR] in BaseAviary.__init__(), PyBullet and control frequencies incompatible with the desired video capture frame rate ({:f}Hz)".format(self.IMG_FRAME_PER_SEC))
                exit()

        #set Starting GUI POV
        if self.GUI:
            self.camera_distance = 4
            self.camera_yaw = -90 # apparentemente così ci si mette in direzione x positiva
            self.camera_pitch = -60     # (- np.pi/6)
            self.camera_target_position =   [ -3 , 0 , 0.5 ]      # TODO set at self.initial_xyzs
            p.resetDebugVisualizerCamera(self.camera_distance, self.camera_yaw, self.camera_pitch, self.camera_target_position)

        ### Create attributes for sensors ####################
        # Aggiungo tot sensori al drone
        self.SENSOR_ATTR = sensors_attributes
        if self.SENSOR_ATTR :
            self.MAX_RANGE = max_sensors_range
            self.NUM_SENSORS = 4
            self.CLOUD_POINT = np.empty((0, 3))
            self.sensor_position =np.zeros((4,3))
            self.sensor_direction = np.array([
                                        [1.0, 0.0, 0.0],   # front
                                        [0.0, 1.0, 0.0],   # Lx                                        
                                        [-1.0, 0.0, 0.0],  # behind
                                        [0.0, -1.0, 0.0],  # Rx                                       
                                        ])   
            # TODO piccolo check sul se serve inizializzare rot_mat    
        
    ###################################################################
        
    def step(self,
             action
             ):
        """Advances the environment by one simulation step.
        Output
        ----------
        observation : ndarray
            (NUM_DRONES, 4)-shaped array containing the front/right/back/left distance observed        
        """
        # Advance the simulation like in BaseAviary
        obs, reward, terminated, truncated, info = super().step(action)

        #Add output from rangefinders
        if self.SENSOR_ATTR:
            observation, Hit_point = self._sensorsObs()
            self._storeHitPoints(Hit_point)

        # TODO definire _storeHitPoints(self)

        return obs, observation, reward, terminated, truncated, info
    
    ###################################################################
        
    def _addObstacles(self):
        """Add obstacles to the environment.

        Only if the observation is of type RGB, landmarks are added.
        Overrides BaseAviary's method.
        
        """
        #TODO: define the logic through which we select the obstacles using the self.OBSTACLE_IDS variable
        # for the moment the function is the same as in BaseRLAviary.py
        
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
                       #p.getQuaternionFromEuler([0,0,0]),
                       p.getQuaternionFromEuler([0,0,np.pi]),  # PROBLEMA : creare urdf con blender decide lui le posizioni e le direzioni quindi tocca inizializzare ogni singolo asset in qualche modo
                       useFixedBase = 1
                       )
        
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
                    ):               # aggiungere nth_drone???
        '''Returns distance from obstacles in the 4 directions # front Rx behind Lx 
        and cloud of up to 4 hit points

        Parameters
        ----------
        nth_drone : int
            The ordinal number/position of the desired drone in list self.DRONE_IDS.

        Returns
        -------
        observation 
            (NUM_DRONES, NUM_SENSORS)-shaped array con le distanze dal primo ostacolo nelle 4 direzioni
                                     se non trova niente prende valore max_range
        Hit_point 
            (NUM_DRONES , NUM_SENSORS, 3)-shaped array con lista di punti rilevati dai rangefinders
            tiene [inf inf inf] se non ha hittato in quello step
            TODO: sviluppare qualcosa che salva i 4 punti in Hit_point a ogni step
        '''
        observation = np.array([[self.MAX_RANGE,self.MAX_RANGE,self.MAX_RANGE,self.MAX_RANGE] for i in range(self.NUM_DRONES)] )
        Hit_point = []
        Hit_point = np.array([[[np.inf , np.inf , np.inf] for j in range(self.NUM_SENSORS)]for i in range(self.NUM_DRONES)] )
        if self.SENSOR_ATTR:
            self.sensor_direction = np.array([
                                        [1.0, 0.0, 0.0],   # front
                                        [0.0, 1.0, 0.0],   # Lx                                        
                                        [-1.0, 0.0, 0.0],  # behind
                                        [0.0, -1.0, 0.0],  # Rx
                                        ]) 
            for i in range(self.NUM_DRONES):         
                self.sensor_position = self._getDroneStateVector(i)[0:3]  # [1. 1. 1.] posizione
                self.rot_mat = np.array(p.getMatrixFromQuaternion(self.quat[i, :])).reshape(3, 3) #TODO vedi ste cazzo di rotazioni
                self.sensor_direction = np.dot(self.sensor_direction,self.rot_mat.T)   
                self.from_positions = np.tile(self.sensor_position, (4, 1))
                self.to_positions = np.array([self.sensor_position + direction * self.MAX_RANGE for direction in self.sensor_direction])
                result =p.rayTestBatch(self.from_positions, self.to_positions)
                for j in range(self.NUM_SENSORS) :
                    ##  Hit fraction: {result[2]}")
                    ##  Hit position: {result[3]}")
                    #closest_hit.append(result[j])            # temporarily turned off, trying new logic
                        #closest_hit[j][0] = result_list[j][0]
                    # Se c'è un punto di contatto, ottieni la distanza
                    if result[j][0] != -1:
                        hit_distance = result[j][2]*self.MAX_RANGE
                        hit_point = result[j][3]
                        # Aggiungi la distanza rilevata come osservazione
                        observation[i][j]= hit_distance
                        # appendi l'hitpoint alla lista Hit_point
                        Hit_point [i][j] = hit_point
                    else:
                        # Se non ci sono oggetti rilevati, assegna una distanza massima
                        observation[i][j] =  self.MAX_RANGE
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

    def _storeHitPoints(self,
                        Hit_point: np.ndarray
                        ):
        """Saves every hit point found with sensor_Obs in the alreasy initialized self.CLOUD_POINT value
        Parameters
        -------
        Hit_point 
            (NUM_DRONES , NUM_SENSORS, 3)-shaped array con lista di punti rilevati dai rangefinders
            tiene [inf inf inf] se non ha hittato

        Returns
        -------
        QUALCOSA
        """

        for i in range(self.NUM_DRONES):
            for j in range(self.NUM_SENSORS):
                self.CLOUD_POINT = np.vstack((self.CLOUD_POINT, Hit_point[i][j])) if np.all(np.isfinite(Hit_point[i][j])) else self.CLOUD_POINT

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

    
