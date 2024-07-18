import numpy as np
import pybullet as p
import random 
import heapq
from gymnasium import spaces
import pkg_resources
from project.envs.ProjAviary import ProjAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ImageType
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
from shapely.ops import unary_union
from shapely.affinity import rotate

class MapAviary(ProjAviary):
    "Multi-drone environment class for topological mapping of unknown places"

    ################################################################################

    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 num_drones: int=1,
                 neighbourhood_radius: float=np.inf,
                 initial_xyzs=None,
                 initial_rpys=None,
                 physics: Physics = Physics.PYB,
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
                 ref_distance : float = 0.5,
                 s_WF: int = +1,
                 c_omega : float = 0.1,
                 c_vel: float = 0.1,
                 output_folder='results',
                 WFstate : int = 1,
                 td : float = 0.02,
                 merging_graphs=False,
                 edges_visualization=False,
                 self_merge=False,
                 max_distance_between_nodes=0.6,
                 total_area_polygon=Polygon([(0, 0), (2, 0), (2, 2), (4, 2), (4, 0), (6, 0), (6, 4), (0, 4)]),
                 point_coverage_radius=0.75,
                 target_coverage=90,
                 maximum_battery_time=300,
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
                         physics= physics,
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
                         output_folder=output_folder,
                         )
        
        self.DIST_WALL_REF = ref_distance  # distanza di riferimento dal muro
        self.S_WF = np.array([[s_WF] for j in range(self.NUM_DRONES)])  # wall following side: 1 for wall on Right side, -1 for wall on Left Side
        self.C_OMEGA = c_omega  # costante di velocità di correzione rotta. unica per tutti i droni
        self.C_VEL = c_vel        # output sulla velocità di controllo per il raddrizzamento
        self.WFSTATE = np.array([[WFstate] for j in range(self.NUM_DRONES)])
        # self.PREV_DIST = np.array([[] for j in range(self.NUM_DRONES)]) # obsolete
        self.CTRL_FREQ = ctrl_freq 
        self.MAX_RANGE = max_sensors_range
        self.distance = np.array([[np.inf , np.inf] for j in range(self.NUM_DRONES)] )
        self.prev_rR = np.array([[self.MAX_RANGE] for j in range(self.NUM_DRONES)] )
        self.prev_rL = np.array([[self.MAX_RANGE] for j in range(self.NUM_DRONES)] )
        self.turn_start_yaw = np.array([[0.] for j in range(self.NUM_DRONES)] )
        self.wfstate0_YAW_counter = np.array([[0.] for j in range(self.NUM_DRONES)] )
        ## vettore di 3 numeri, 
        # il primo è un counter angolare che si accerta che al massimo faccia un giro
        # il secondo è il valore della distanza frontale minima
        ###################################### (se questa è maggiore di DISTREF si entrerà in state -1)
        ###################################### (se è minore di DISTREF tocca inventare uno stato di allontanamento ortogonale)
        # il terzo si aggiorna insieme al secondo e indica la direzione della distanza minima in qualche modo
        self.START = np.array([[0] for j in range(self.NUM_DRONES)] )
        self.stateminus1counter = np.array([[-1] for j in range(self.NUM_DRONES)] )
        self.state1counter = np.array([[0] for j in range(self.NUM_DRONES)] )
        self.state3counter = np.array([[0] for j in range(self.NUM_DRONES)] )
        self.state4counter = np.array([[0] for j in range(self.NUM_DRONES)] )
        self.state5counter = np.array([[0] for j in range(self.NUM_DRONES)] )
        self.state6counter = np.array([[0] for j in range(self.NUM_DRONES)] )
        self.td = td ## threshold distance to determine wether the drone is almost at reference distance from wall
        self.IM_IN_A_CORNER = np.array([[False] for j in range(self.NUM_DRONES)] )
        self.MOVE_FORWARD =  np.array([[False] for j in range(self.NUM_DRONES)] )
        self.radius = np.array([[0.] for j in range(self.NUM_DRONES)] ) 
        self.WFSTATE_WAS_CHANGED = np.array([[False] for j in range(self.NUM_DRONES)] ) 
        self.WF_ref_angle = np.array([[0.] for j in range(self.NUM_DRONES)] ) # yaw di riferimento di start dello stato 1 per non allontanarsi troppo dalla direzione parallela
        self.memory_state = np.array([[np.inf] for j in range(self.NUM_DRONES)] ) # memory dello stato in cui ci strova prima di una collision
        self.memory_position = np.array([[0. , 0. ,0. , 0. ,0. ,0.] for j in range(self.NUM_DRONES)] )
        ## variabili collision avoidance
        self.drones_distance_pre = np.array([[0.] for j in range(self.NUM_DRONES)] )
        ## variabili database 
        self.drones_db = {} 
        self.adjacency_matrices = []
        self.custom_id_balls_map = {}
        self.MERGING = merging_graphs
        self.edges_visualization = edges_visualization
        self.SELFMERGE = self_merge
        self.max_distance_between_nodes = max_distance_between_nodes
        ## variabili missione
        self.total_area_polygon = total_area_polygon
        self.coverage_percent = 0 # percentuale di area coperta totale
        self.single_drone_coverage_percent = [] # percentuale di area coperta totale singolamente dal jesimo drone
        # fine missione con TARGET_COVERAGE
        self.TARGET_COVERAGE = target_coverage
        self.point_coverage_radius = point_coverage_radius
        self.efficiency = 0 # appena il coverage_percent raggiunge il 90% viene salvato come valore temporale
        self.COVERAGE_IS_ENOUGH = False
        # fine missione dopo tot tempo
        self.MAXIMUM_BATTERY_TIME = maximum_battery_time
        self.TIME_IS_OUT = False
        self.BLOCK_SIMULATION_WHEN_RETURNING_PHASE_STARTS = True
        self.returning_paths = {} # dizionario con i path di ritorno per tutti i droni
        self.NUM_WP = np.array([[0] for i in range(num_drones)])  # numero di WAYPOINTS del percorso di ritorno per ogni drone
        self.wp_counters = np.array([[0] for i in range(num_drones)])  # contatore del prossimo WAYPOINT DA RAGGIUNGERE
    ################################################################################

    def NextWP(self,obs,observation,INIT_XYZS): #Aggiungere gli input necessari
        """Definisce la logica di navigazione trovando il prossimo WP usando le funzioni:
        - _DecisionSystem
        - _WallFollowing()  , il quale chiama _WallFollowingAndAlign()
        """
        self.observation = observation
        TARGET_POS = np.array([[0. , 0., 0.]for j in range(self.NUM_DRONES)])
        TARGET_RPY = np.array([[0. , 0., 0.]for j in range(self.NUM_DRONES)])
        time_step  = 1 / self.CTRL_FREQ

        TARGET_VEL = np.array([[0. , 0., 0.]for j in range(self.NUM_DRONES)])
        TARGET_RPY_RATES = np.array([[0. , 0., 0.]for j in range(self.NUM_DRONES)])
        #self.S_WF=self._decisionSystem()  # controllare se aggiornare in self o no

        if self.COVERAGE_IS_ENOUGH == False and self.TIME_IS_OUT == False:
            TARGET_OMEGA, RELATIVE_FRAME_VEL , self.WFSTATE , self.S_WF, droni_nelle_vicinanze = self._WallFollowing3()

        for i in range(self.NUM_DRONES) :
            if self.COVERAGE_IS_ENOUGH == False and self.TIME_IS_OUT == False:  
                yaw = obs[i][9]
                print("absolute yaw =", np.degrees(yaw),"°")
                rot_mat = np.array([[np.cos(yaw) ,-np.sin(yaw) , 0.0],  
                                    [np.sin(yaw) , np.cos(yaw) , 0.0],                                           
                                    [-1.0        , 0.0         , 1.0]])
                ABSOLUTE_FRAME_VEL = np.dot(RELATIVE_FRAME_VEL[i], rot_mat.T )
                collision=np.nonzero(droni_nelle_vicinanze[:,i:i+1]==2)[0]  
                if self.WFSTATE[i] == 5:
                    precedenza_speciale = False
                    for j in collision:
                        if self.WFSTATE[j] == 4:
                            precedenza_speciale = True
                            break                        
                    if self.memory_position[i][2] == 0:
                        self.memory_position[i][0:3] = obs[i][0:3]
                        self.memory_position[i][3:6] = obs[i][7:10]
                        if (self.observation[i][0] >=self.observation[i][3] and self.observation[i][0] >=self.observation[i][1]) \
                                and (self.observation[i][2] >=self.observation[i][3] and self.observation[i][2] >=self.observation[i][1]) :
                            self.RELATIVE_FRAME_DIRECTION = [0. , 1. ,1.]
                            if self.observation[i][3] <= self.observation[i][1] :
                                if  self.observation[i][3] < 1.5:
                                    self.spostamento_laterale =  -np.abs(self.observation[i][3] - 0.40)
                                else:
                                    self.spostamento_laterale = -0.3
                            else : 
                                if self.observation[i][1] < 1.5 :
                                    self.spostamento_laterale =  +np.abs(self.observation[i][1] - 0.4)
                                else:
                                    self.spostamento_laterale = +0.3
                        elif self.observation[i][0] >=self.observation[i][2] :  
                            self.RELATIVE_FRAME_DIRECTION = [-1. ,  0. ,1.]
                            if  self.observation[i][3] < 1.5:
                                self.spostamento_laterale =  np.abs(self.observation[i][2] - 0.40)
                            else:
                                self.spostamento_laterale = 0.3
                        else:
                            self.RELATIVE_FRAME_DIRECTION = [1. ,  0. ,1.]
                            if  self.observation[i][3] < 1.5:
                                self.spostamento_laterale =  np.abs(self.observation[i][0] - 0.40)
                            else:
                                self.spostamento_laterale = 0.3
                    if self.memory_state[i] == -1 : #se il drone è nello stato di inizio missione permane nella posizione in attesa di partire

                        TARGET_POS[i][0] =  self.memory_position[i][0] 
                        TARGET_POS[i][1] =  self.memory_position[i][1] 
                        TARGET_POS[i][2] =  self.memory_position[i][2]

                        TARGET_RPY[i][0] = self.memory_position[i][3]
                        TARGET_RPY[i][1] = self.memory_position[i][4]   
                        TARGET_RPY[i][2] = self.memory_position[i][5]

                    elif  precedenza_speciale:
                        TARGET_POS[i][0] =  self.memory_position[i][0] 
                        TARGET_POS[i][1] =  self.memory_position[i][1] 
                        TARGET_POS[i][2] =  self.memory_position[i][2] + 0.5

                        TARGET_RPY[i][0] = self.memory_position[i][3]
                        TARGET_RPY[i][1] = self.memory_position[i][4]   
                        TARGET_RPY[i][2] = self.memory_position[i][5]    
                    else :
                        self.RELATIVE_FRAME_DIRECTION = self.RELATIVE_FRAME_DIRECTION/np.linalg.norm(self.RELATIVE_FRAME_DIRECTION)
                        ABSOLUTE_FRAME_DIRECTION = np.dot(self.RELATIVE_FRAME_DIRECTION, rot_mat.T )  

                        TARGET_POS[i][0] =  self.memory_position[i][0] + self.spostamento_laterale* ABSOLUTE_FRAME_DIRECTION[0]
                        TARGET_POS[i][1] =  self.memory_position[i][1] + self.spostamento_laterale *ABSOLUTE_FRAME_DIRECTION[1]
                        TARGET_POS[i][2] =  self.memory_position[i][2] + 0.3 * ABSOLUTE_FRAME_DIRECTION[2]
                        
                        TARGET_RPY[i][0] = self.memory_position[i][3]
                        TARGET_RPY[i][1] = self.memory_position[i][4]   
                        TARGET_RPY[i][2] = self.memory_position[i][5] 

                elif self.WFSTATE[i] == 6:
                        TARGET_POS[i][0] =  self.memory_position[i][0] 
                        TARGET_POS[i][1] =  self.memory_position[i][1] 
                        TARGET_POS[i][2] =  self.memory_position[i][2] 

                        TARGET_RPY[i][0] = self.memory_position[i][3]
                        TARGET_RPY[i][1] = self.memory_position[i][4]   
                        TARGET_RPY[i][2] = self.memory_position[i][5]
                else:
                    if self.memory_position[i][2] != 0:
                        self.memory_position[i] = np.array([0., 0., 0., 0. ,0. , 0.])
                    TARGET_POS[i][0] = obs[i][0] + ABSOLUTE_FRAME_VEL[0] * time_step  
                    TARGET_POS[i][1] = obs[i][1] + ABSOLUTE_FRAME_VEL[1] * time_step 
                    TARGET_POS[i][2] = INIT_XYZS[i][2] #+ ABSOLUTE_FRAME_VEL[2] * time_step 

                    TARGET_RPY[i][0] = obs[i][7]
                    TARGET_RPY[i][1] = obs[i][8]   
                    TARGET_RPY[i][2] = obs[i][9] #+ TARGET_OMEGA[i] * time_step # solo il terzo elemento

                TARGET_VEL[i][0] = ABSOLUTE_FRAME_VEL[0]
                TARGET_VEL[i][1] = ABSOLUTE_FRAME_VEL[1]
                TARGET_VEL[i][2] = 0.

                TARGET_RPY_RATES[i][0] = 0.
                TARGET_RPY_RATES[i][1] = 0.
                TARGET_RPY_RATES[i][2] = TARGET_OMEGA[i]
            else:
            ########## WAYPOINT ########
                # usando wp_counters e NUM_WP, restituiti da 
                    #TARGET_POS[i] = NEXT_WP
                TARGET_POS[i] = self.waypoint_nav(i)
                TARGET_VEL[i] = np.array([0.0, 0.0, 0.0])
                pass

        for j in range(self.NUM_DRONES) : # saves actual mindist to distance[0]. It will be the prev_dist of next step
            self.distance[j][0] =  self.distance[j][1]
        ### LOGICA DI FINE MISSIONE ###
        self.coverage_percent = self.calculate_coverage(self.point_coverage_radius)
        self.single_drone_coverage_percent = self.calculate_coverage_individually(self.point_coverage_radius)
        if self.TIME_IS_OUT == False:
            time_passed = self.step_counter * self.PYB_TIMESTEP
            if time_passed >= self.MAXIMUM_BATTERY_TIME:
                self.TIME_IS_OUT = True
                self.plot_coverage(self.total_area_polygon, radius=0.75)  # Plotta la copertura totale
                self.returning_phase_paths_planning()
            
        print(f"La percentuale totale di esplorazione è: {self.coverage_percent:.2f}%")
        for i, coverage in enumerate(self.single_drone_coverage_percent):
            print(f"La percentuale di esplorazione del drone {i+1} è: {coverage:.2f}%")
        return TARGET_POS, TARGET_RPY, TARGET_VEL, TARGET_RPY_RATES

    def waypoint_nav(self,
                     drone_id,
                     max_step_distance=0.1):
        """
        Parametres
        --------
        drone_id : int
            L'ID del drone. 

        Returns
        --------
        TARGET_POS : ndarray [float float float]
            (3)-shaped array con la posizione del prossimo waypoint
        TARGET_VEL : ndarray [float float float]
            (3)-shaped array con la velocità desiderata
        """
        if self.wp_counters[drone_id][0] == self.NUM_WP[drone_id][0]: # se ho raggiunto tutti i WAYPOINTS
            wp_counter = self.wp_counters[drone_id][0]
            waypoint_id = self.returning_paths[drone_id][wp_counter-1]
            target_pos = np.array(self.drones_db[drone_id][waypoint_id]['coords']) # mettici l'ultimo wp invece della pos
        else:
            wp_counter = self.wp_counters[drone_id][0]
            waypoint_id = self.returning_paths[drone_id][wp_counter] 
            target_pos = np.array(self.drones_db[drone_id][waypoint_id]['coords'])
            # comparo il comando per vedere se è aggressivo (troppo lontano)
            current_pos = self.pos[drone_id]
            direction_vector = target_pos - current_pos
            distance_to_target = np.linalg.norm(direction_vector)
            self.update_wp_counter(drone_id,target_pos)
            if distance_to_target > max_step_distance: #se troppo lontano
                direction_vector_normalized = direction_vector / distance_to_target
                target_pos = current_pos + direction_vector_normalized * max_step_distance
        return target_pos
    
    ################################################################################

    def update_wp_counter(self,
                          drone_id,
                          target_pos,
                          threshold=0.1): # TODO:tuning
        """
        Aggiorna il contatore dei waypoint per il drone specificato se il drone ha raggiunto il waypoint attuale.

        Parameters
        ----------
        drone_id : int
            L'ID del drone.
        threshold : float
            La distanza soglia per considerare il waypoint come raggiunto.

        Returns
        -------
        None
        """
        current_pos = self.pos[drone_id]
        distance = self.euclidean_distance(current_pos, target_pos)
        
        if distance < threshold:
            self.wp_counters[drone_id][0] += 1

    def _decisionSystem(self,nth_drone): #Aggiungere gli input necessari
        """Definisce la logica di scelta della direzione da prendere dopo uno stop (dopo che becca muro)
           Imposta la direzione da prendere, cioè se destra o sinistra in base alla posizione dei droni, se questi sono davanti a lui e a destra
           girera verso sinistra, se sono dietro di lui e a sinistra girerà verso destra.
           
           Return
           --------
           qualcosa
        """
        drones_position = np.array([self._getDroneStateVector(j)[0:3] for j  in range(self.NUM_DRONES)])
        drones_distance = np.array([[np.inf] for j  in range(self.NUM_DRONES)])
        right_or_left = np.array([[0.] for j  in range(self.NUM_DRONES)])
        
        for i in range(self.NUM_DRONES) :
            if  i != nth_drone:
                relative_position = drones_position[i] - drones_position[nth_drone]
                rot_mat = np.array(p.getMatrixFromQuaternion(self.quat[nth_drone, :])).reshape(3, 3) 
                relative_position_rel= np.dot(relative_position,rot_mat) 
                drones_distance[i] = self.euclidean_distance( drones_position[nth_drone][0:2], drones_position[i][0:2])
                dir = np.arctan2(relative_position_rel[1],relative_position_rel[0])
                if (dir < 3/4*np.pi and dir > -3/4*np.pi) and drones_distance[i] < self.MAX_RANGE:
                    if relative_position_rel[1]<=0 :
                        right_or_left[i] = 1 
                    else:
                        right_or_left[i]=2
        N_drones_right = len(np.nonzero(right_or_left==1)[0]) 
        N_drones_left = len(np.nonzero(right_or_left==2)[0])        
           
        if N_drones_right > N_drones_left :
                sWF = 1
        elif N_drones_right < N_drones_left:
                sWF = -1
        else:
            sWF = random.choice([-1,1])
        print("scelta randomica",sWF)
        return sWF 
    
    ################################################################################

    def _WallFollowing3(self):    #ALG B.1
        """Funzione per seguire in modo allineato il muro -> tira fuori la rotazione desiderata da mandare ai controlli
        
        Parameters (non input ma nel self)
        ---------
        WFstate (NUM_DRONES, 1)-shaped  prende tre possibili valori:
        - WFstate = -1 stato iniziale, avanzamento lineare finchè non incontra un muro
        - WFstate = 0 ruotare da fermo (v=0) per allinearsi al muro
        - WFstate = 1 WF vero e proprio (v!=0) e piccoli aggiustamenti di rotta con _WallFollowingandAlign()
        - WFstate = 2 ruotare attorno all'angolo
        - WFstate = 3 stato transitorio di uscita da 2 , piccolo avanzamento lineare per non perdere subito rR o rL
        - WFstate = 4 stato di debug di 0 quando vi si entra in situazioni troppo vicine o troppo lontane al muro #TODO: da sviluppare
                per ora il debug è interno a WFstate = 0

        Returns
        --------
        omega: ndarray - [[float] for j in range(self.NUM_DRONES)]
            (NUM_DRONES, 1)-shaped array che contiene la omega (yaw_rate) di controllo per ogni drone
        vel: ndarray - [[float float float] for j in range(self.NUM_DRONES) ]
            (NUM_DRONES, 3)-shaped array che contiene la velocita di controllo per ogni drone
        WFstate: ndarray - [[int] for j in range(self.NUM_DRONES) ]
            (NUM_DRONES, 1)-shaped array che indica lo stato nella logica di wallfollowing per ogni drone
        """
        NWdistance,SWdistance,SEdistance,NEdistance = self._MinDistToWall2()  ## updates distance [[prev_dist , curr_dist]]  
        droni_nelle_vicinanze = np.zeros((self.NUM_DRONES,self.NUM_DRONES))
        cw = self.C_OMEGA   
        cv = self.C_VEL     
        omega = np.array([[0.] for j in range(self.NUM_DRONES)])
        vel = np.array([[0. , 0. , 0.] for j in range(self.NUM_DRONES)])
        state_2_omega_coeff = 0.95  # <1 per allargare il raggio di curvatura del WFSTATE = 2

        time_step  = 1 / self.CTRL_FREQ
        
        for i in range(self.NUM_DRONES) :
            print("er WFSTATE del drone" , i , "è", self.WFSTATE[i])
            rF = self.observation[i][0] # distanza frontale
            rL = self.observation[i][1] # distanza sinistra
            rB = self.observation[i][2] # distanza retro
            rR = self.observation[i][3] # distanza destra

            droni_nelle_vicinanze[:,i:i+1] = self.find_collision(i)
            superiors = np.nonzero(droni_nelle_vicinanze[:,i] == 2)[0]# indici 0based dei droni a cui devo la precedenza
            inferiors = np.nonzero(droni_nelle_vicinanze[:,i] == 1)[0] # indici 0based dei droni su cui ho la precedenza
            if len(superiors) != 0 and self.WFSTATE[i] != 5 and self.WFSTATE[i] != 6:
                self._SwitchWFSTATE(i, 5)
            # len(inferiors) è il numero di droni su cui ho precedenza
            # len(superiors) è il numero di droni a cui devo dare precedenza

        ######################  STATO 0 : GIRO SUL POSTO PER ALLINEARSI AL MURO  ############################
        ### note: l'uscita da un angolo convesso è funzionante solo a DISTWALLREF dal muro
        ### possibili migliorie: - aumentare la precisione di ripartenza da angolo
        ###                      - il debug funge solo se rispetta le condizioni al primo giro, poi va all'infinito
            if self.WFSTATE[i] == 0 : 
                omega [i] = ([+1*self.S_WF[i][0]*cw  *   2])   
                vel [i] = ([0. , 0. , 0.])
                self.wfstate0_YAW_counter[i][0] += omega[i] * time_step                
                if self.S_WF[i][0] == 1: # wallfollowing con muro a destra
                    print("differenza tra range destro e range destro precedente: ",np.abs(rR - self.prev_rR[i][0]))
                    if self.IM_IN_A_CORNER[i][0] == True:
                        if np.abs(rB - self.DIST_WALL_REF) < 8*self.td and np.abs(rR - self.DIST_WALL_REF) < 5*self.td: #and np.abs(rF - self.DIST_WALL_REF) > 10*self.td: # se dietro e destra so circa ar top
                            if rR != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) < self.td*0.03 : # TODO: aggiusta sensibilità
                                self._SwitchWFSTATE(i, 3)
                                self.state1counter[i][0] = -80
                                print("esco da WFSTATE = 0 (in un convesso) e entro in WFSTATE = 1")
                    elif self.MOVE_FORWARD[i][0] == True:
                        print ("l'angolo di curva accumulato è" , self.wfstate0_YAW_counter[i][0])
                        if np.abs(self.wfstate0_YAW_counter[i][0]) > 0.8 and np.abs(rR - self.prev_rR[i][0]) < self.td*0.01: # se l
                            #if np.abs(rR - self.DIST_WALL_REF) < self.td :
                                self._SwitchWFSTATE(i, 3)
                                print("esco da WFSTATE = 0 (ho un muro di fronte) e entro in WFSTATE = 1")
                    elif rR != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) < self.td*0.05  :  
                        self._SwitchWFSTATE(i, 3)
                        print("esco da WFSTATE = 0 (causa parallelo al muro destro) e entro in WFSTATE = 3")
                elif self.S_WF[i][0] == -1: # wallfollowing con muro a sinistra                    
                    print("differenza tra range sinistro e range sinistro precedente: ",np.abs(rL - self.prev_rL[i][0]))
                    if self.IM_IN_A_CORNER[i][0] == True:
                        if np.abs(rB - self.DIST_WALL_REF) < 8*self.td and np.abs(rL - self.DIST_WALL_REF) < 5*self.td: #and np.abs(rF - self.DIST_WALL_REF) > 10*self.td: # se dietro e sinistra so circa ar top
                            if rL != self.MAX_RANGE and np.abs(rL - self.prev_rL[i][0]) < self.td*0.03 : # TODO: aggiusta sensibilità
                                self._SwitchWFSTATE(i, 3)
                                self.state1counter[i][0] = -80
                                print("esco da WFSTATE = 0 (in un convesso) e entro in WFSTATE = 1")
                    elif self.MOVE_FORWARD[i][0] == True:
                        print ("l'angolo di curva accumulato è" , self.wfstate0_YAW_counter[i][0])
                        if np.abs(self.wfstate0_YAW_counter[i][0]) >0.8 and np.abs(rL - self.prev_rL[i][0]) < self.td*0.01: # se l
                            #if np.abs(rL - self.DIST_WALL_REF) < self.td:
                                self._SwitchWFSTATE(i, 3)
                                print("esco da WFSTATE = 0 (ho un muro di fronte) e entro in WFSTATE = 1")
                    elif rL != self.MAX_RANGE and np.abs(rL - self.prev_rL[i][0]) < self.td*0.05 :    
                        self._SwitchWFSTATE(i, 3)
                        print("esco da WFSTATE = 0 (causa parallelo al muro sinistro) e entro in WFSTATE = 3")

        ######################  STATO -1 : AVANZAMENTO LINEARE FINO A WALLREF  ############################
        ### stato di avanzamento lineare finchè la min dist non si avvicina alla DIST_WALL_REF
            if self.WFSTATE[i] == -1 : 
                omega [i] = ([0])
                vel [i] = np.dot(  2*cv , [1. , 0. , 0.] )
                ### aggiunta nodo iniziale
                if self.stateminus1counter[i] == -1:
                    self.add_point(i,self.pos[i],'start')

                self.stateminus1counter[i][0] += 1
                if np.abs(rR - self.DIST_WALL_REF) > 1.5 or np.abs(rL - self.DIST_WALL_REF) > 1.5 : 
                    #scelta randomica del muro da seguire   
                    self._SwitchWFSTATE(i, 4)
                    print("esco da WFSTATE = -1 e entro in WFSTATE = 4")
                elif (rR - self.prev_rR[i][0]) > self.td*0.001:
                        omega[i]=([-1*self.C_OMEGA*0.5])
                elif (rR - self.prev_rR[i][0]) <- self.td*0.001: 
                        omega[i]=([1*self.C_OMEGA*0.5])
                ###### Topological
                if self.stateminus1counter[i][0] == 100: # era 100 ; TODO: aggiungi che lo fa anche se capisce di essere in junction
                    self.add_point(i,self.pos[i],'corridor')
                    self.stateminus1counter[i][0] = 0   

        ######################  STATO 1 : WALLFOLLOWING  ############################
        ### possibili migliorie: - tuning dei coefficienti di _WallFollowingandAlign per un andamento meno oscillante
            elif self.WFSTATE[i][0] == 1 : # wallfollowing
                vel [i] = np.dot(  2*cv , [1. , 0. , 0.] ) 
                omega[i] = self._WallFollowingandAlign3(i)  
                self.state1counter[i][0] += 1
                if self.S_WF[i][0] == 1: # wallfollowing con muro a destra
                        if np.abs(rF - self.DIST_WALL_REF) < 5*self.td and np.abs(rR - self.DIST_WALL_REF) < 8*self.td :
                           if len(inferiors)==0:
                                self._SwitchWFSTATE(i, 0)
                                print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono in un angolo")
                        elif np.abs(rF - self.DIST_WALL_REF) < 0.5*self.td:
                             if len(inferiors)==0:
                                self._SwitchWFSTATE(i, 0)
                                print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono in un angolo")
                        if np.abs(self.prev_rR[i][0] - rR) > 1.5*self.td : #se perdo il muro
                            turn_direction = self._decisionSystem(i)
                            if  turn_direction == -1 and self.MOVE_FORWARD[i][0] == False:
                                self._SwitchWFSTATE(i, 2) #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo 
                                print("esco da WFSTATE = 1 e entro in WFSTATE = 2")
                            elif self.MOVE_FORWARD[i][0] == False or self.state1counter[i][0]>120 : 
                                self._SwitchWFSTATE(i, 4) #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo 
                                print("esco da WFSTATE = 1 e entro in WFSTATE = 4")
                        # elif np.abs(rR- self.DIST_WALL_REF) < 0.5*self.td:
                        #     self._SwitchWFSTATE(i, 0)
                        #     print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono troppo vicino")
                elif self.S_WF[i][0] == -1: # wallfollowing con muro a sinistra
                        if np.abs(rF - self.DIST_WALL_REF) < 5*self.td and np.abs(rL - self.DIST_WALL_REF) < 8*self.td :
                            if len(inferiors)==0:
                                self._SwitchWFSTATE(i, 0)
                                print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono in un angolo")
                        elif np.abs(rF - self.DIST_WALL_REF) < 0.5*self.td:
                             if len(inferiors)==0:
                                self._SwitchWFSTATE(i, 0)
                                print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono in un angolo")
                        if np.abs(self.prev_rL[i][0] - rL) > 1.5*self.td : #condizione con la storia di rR
                            turn_direction = self._decisionSystem(i)
                            if  turn_direction ==1 and self.MOVE_FORWARD[i][0] == False:
                                self._SwitchWFSTATE(i, 2) #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo 
                                print("esco da WFSTATE = 1 e entro in WFSTATE = 2")
                            elif self.MOVE_FORWARD[i][0] == False or self.state1counter[i][0]>120 :
                                self._SwitchWFSTATE(i, 4) #dovrei seguire il muro ma non lo vedo più --> ci sta un angolo 
                                print("esco da WFSTATE = 1 e entro in WFSTATE = 4")
                        #elif rR - self.DIST_WALL_REF < 0.5*self.td:
                        #     self._SwitchWFSTATE(i, 0)
                        #     print("esco da WFSTATE = 1 e entro in WFSTATE = 0 visto che sono troppo vicino")
                ###### Topological
                if self.state1counter[i][0] == 120: # era 100 ; TODO: aggiungi che lo fa anche se capisce di essere in junction
                    self.add_point(i,self.pos[i],'corridor')
                    self.state1counter[i][0] = 0                    
                        
        ######################  STATO 2 : CURVA  ############################
        ### possibili migliorie: - rimuovere la cosa che dopo va in 3 e farlo andare in 0
            elif self.WFSTATE[i][0] == 2 :
                vel [i] = np.dot(  cv , [1. , 0. , 0.] )
                omega[i] = ([-self.S_WF[i][0]*state_2_omega_coeff*cv/self.radius[i][0]])
                self.turn_start_yaw[i][0] += omega[i] * time_step
                print ("l'angolo di curva accumulato è" , self.turn_start_yaw[i][0])
                if np.abs(self.turn_start_yaw[i][0]) > 1.4 : # se l'angolo accumulato è maggiore di un certo angolo (per ora metto 1 che è tipo 60 deg)
                    # e solo ora faccio la verifica con grosso salto di rR # TODO sensibilità da tunare
                    if self.S_WF[i][0] == 1: # wallfollowing con muro a destra
                        print("la differenza tra il range destro attuale e precedente è:", np.abs(rR - self.prev_rR[i][0]) )
                        if self.prev_rR[i][0] != self.MAX_RANGE and np.abs(rR - self.prev_rR[i][0]) < self.td*0.001:
                            self._SwitchWFSTATE(i, 0)
                            print("esco da WFSTATE = 2 e entro in WFSTATE = 0 poichè sono abbastanza allineato col muro")
                        if np.abs(rR - self.prev_rR[i][0]) > 0.5 or rR == self.MAX_RANGE:
                           self._SwitchWFSTATE(i, 3)
                        if np.abs(rF- self.DIST_WALL_REF)<self.td:
                           self._SwitchWFSTATE(i, 0)
                    elif self.S_WF[i][0] == -1: # wallfollowing con muro a sinistra
                        print("la differenza tra il range sinistro attuale e precedente è:", np.abs(rL - self.prev_rL[i][0]) )
                        if self.prev_rL[i][0] != self.MAX_RANGE and np.abs(rL - self.prev_rL[i][0]) < self.td*0.001 : # 0.0015 era bono
                            self._SwitchWFSTATE(i, 0)
                            print("esco da WFSTATE = 2 e entro in WFSTATE = 0 poichè sono abbastanza allineato col muro")
                        if np.abs(rL - self.prev_rL[i][0]) > 0.5 or rL == self.MAX_RANGE:
                            self._SwitchWFSTATE(i, 3)
                        if np.abs(rF- self.DIST_WALL_REF)<self.td:
                           self._SwitchWFSTATE(i, 0)
                    
        ######################  STATO 3 : POST-0 / POST-CURVA AVVICINAMENTO A DISTWALLREF  ############################
        ### avvia dopo la curva quando il drone è circa parallelo al muro, serve a portarlo a DISTWALLREF con una velocità laterale
        ### possibili migliorie:
        ###                    
            elif self.WFSTATE[i][0] == 3:
                self.state3counter[i][0] += 1 
                if self.S_WF[i][0] == 1: # wallfollowing con muro a destra
                    if  (np.abs(rR - self.DIST_WALL_REF)  > 0.5) and self.state3counter[i][0] < 120 :
                    #se rischio di aver perso il ranger post curva vado un pò avanti
                        vel [i] = np.dot(  cv , [1. , 0. , 0.] )
                        omega[i] = ([0]) 
                    else :
                        omega[i] = ([0]) 
                        vel [i] = np.dot(  cv , [0. , 0. , 0.] )
                        if np.abs(rR - self.DIST_WALL_REF) < self.td*0.5:
                            self._SwitchWFSTATE(i, 1)
                            print("esco da WFSTATE = 3 e entro in WFSTATE = 1 poichè sono alla dist giusta dal muro")
                        elif rR > self.DIST_WALL_REF:
                            vel[i] = np.dot(  cv , [np.cos(np.pi/3) , - np.sin(np.pi/3) , 0.] )
                        else:
                            vel[i] = np.dot(  cv ,[np.cos(np.pi/3) , np.sin(np.pi/3) , 0.] )
                elif self.S_WF[i][0] == -1: # wallfollowing con muro a sinistra
                    if  (np.abs(rL - self.DIST_WALL_REF)  > 0.5) and self.state3counter[i][0] < 120 :
                    #se rischio di aver perso il ranger post curva vado un pò avanti
                        vel [i] = np.dot(  cv , [1. , 0. , 0.] )
                        omega[i] = ([0]) 
                    else: # se ho ancora un range laterale affidabile
                        omega[i] = ([0]) 
                        vel [i] = np.dot(  cv , [0. , 0. , 0.] )
                        if np.abs(rL - self.DIST_WALL_REF) < self.td*0.5:
                            self._SwitchWFSTATE(i, 1)
                            print("esco da WFSTATE = 3 e entro in WFSTATE = 1 poichè sono alla dist giusta dal muro")
                        elif rL > self.DIST_WALL_REF:
                            vel[i] = np.dot(  cv , [np.cos(np.pi/3) , np.cos(np.pi/3) , 0.] )
                        else:
                            vel[i] = np.dot(  cv , [np.cos(np.pi/3) , -np.cos(np.pi/3) , 0.] )
        ######################  STATO 4 : AVANZAMENTO LINEARE SENZA RIFERIMENTI  ############################
        ### avvia dopo che a una junction il drone decide di andare dritto
        ### il problema è la mancanza di riferimento e le condizioni di uscita sono su rR, rL e rF
            elif self.WFSTATE[i][0] == 4:               
                omega [i] = ([0])
                vel [i] = np.dot(  2*cv , [1. , 0. , 0.] )
                self.state4counter[i][0]+=1
                if  np.abs(rF - self.DIST_WALL_REF) < self.td : # se la distanza frontale è circa DWR
                        if np.abs(rL - self.DIST_WALL_REF)>0.5 and np.abs(rR- self.DIST_WALL_REF)>0.5 : # e ho via libera a destra e sinistra
                            self.S_WF[i][0] = self._decisionSystem(i)
                        elif np.abs(rL - self.DIST_WALL_REF)>0.5 : # se ho un minimo di via libera solo Sx
                            self.S_WF[i][0]=1
                        elif np.abs(rL - self.DIST_WALL_REF)>0.5: # se ho un minimo di via libera solo Dx
                            self.S_WF[i][0]=-1
                        self._SwitchWFSTATE(i, 0)
                        print("esco da WFSTATE = 4 (ho un muro davanti) e entro in WFSTATE = 0")
                elif self.state4counter[i][0] > 150 : # distanza frontale ancora non DWR
                # se la rF è ancora maggiore e ho comunque camminato un tot
                # mi devo assicurare di non toccare un muro in maniera diagonale
                    if np.abs(rR - self.DIST_WALL_REF) < self.td:
                        self.S_WF[i][0]= 1   
                        self._SwitchWFSTATE(i, 0)
                        print("esco da WFSTATE = 4 (ho un muro a destra) e entro in WFSTATE = 3")
                    elif np.abs(rL - self.DIST_WALL_REF) < self.td:
                        self.S_WF[i][0]= -1
                        self._SwitchWFSTATE(i, 0)
                        print("esco da WFSTATE = 4 (ho un muro a sinistra) e entro in WFSTATE = 3")                        
                ###### Topological
                if self.state4counter[i][0] == 80: # era 100 ; TODO: aggiungi che lo fa anche se capisce di essere in junction
                    self.add_point(i,self.pos[i],'corridor')
                    self.state4counter[i][0] = 0  

        ######################  STATO 5 : ENTRATA NEL COLLISION AVOIDANCE  ############################
            elif self.WFSTATE[i][0] == 5:
                    vel [i] = np.dot(  cv , [0, 0 , 0] )
                    omega[i] = ([0])
                    if len(superiors) == 0:
                        self.state5counter[i][0] += 1
                        if self.state5counter[i][0] > 50 :
                            self._SwitchWFSTATE(i, 6)
        ######################  STATO 6 : USCITA DAL COLLISION AVOIDANCE  ############################
            elif self.WFSTATE[i][0] == 6:
                    self.state6counter[i][0] += 1
                    vel [i] = np.dot(  cv , [0, 0 , 0] )
                    omega[i] = ([0])  
                    print(self.state6counter[i][0])
                    if  len(superiors)!= 0:
                        self._SwitchWFSTATE(i, 5)
                    elif self.state6counter[i][0] > 150 :
                         self._SwitchWFSTATE(i, self.memory_state[i])
                         self.memory_state[i] = np.inf
            print("vel=", vel[i])
            print("omega=", omega[i][0]) 
            self.prev_rR[i] = rR
            self.prev_rL[i] = rL
            self.WFSTATE_WAS_CHANGED[i][0] = False
            ###### manual override settings ######
            #self.WFSTATE[i] = 0
            #omega[i] = ([0.5])
            #vel [i] = ([0.2])        
                
        return omega , vel , self.WFSTATE, self.S_WF , droni_nelle_vicinanze
    
    ################################################################################
    
    def _SwitchWFSTATE(self,
                       nth_drone,
                       new_WFSTATE):
        """funzione per passare a un nuovo WFSTATE , sostituisce self.WFSTATE[i] = 3 
        Parameters
        ---------
        nth_drone: int
        new_WFSTATE : int
            nuovo WFSTATE a cui passare
        """

        if self.WFSTATE_WAS_CHANGED[nth_drone][0] == False: #se non ha già cambiato
            self.WFSTATE_WAS_CHANGED[nth_drone][0] = True # allora ho cambiato
            old_WFSTATE = self.WFSTATE[nth_drone][0]  # might use to print stuff
            self.WFSTATE[nth_drone][0] = new_WFSTATE
            print("drone ",nth_drone, ": " ,old_WFSTATE, ">>", new_WFSTATE)
    ## SEZIONE CONTATORI ##
            if new_WFSTATE == 0:
                self.wfstate0_YAW_counter[nth_drone][0] = 0
            elif new_WFSTATE == 1: 
                self.state1counter[nth_drone][0] = 0
                self.WF_ref_angle[nth_drone] = self.rpy[nth_drone][2]
            elif new_WFSTATE == 2:
                self.turn_start_yaw[nth_drone][0] = 0
            elif new_WFSTATE == 3:
                self.state3counter[nth_drone][0] = 0
            elif new_WFSTATE == 4:
                self.turn_start_yaw[nth_drone][0] = 0
                self.state4counter[nth_drone][0] = 0
            elif new_WFSTATE == 5:
                self.state5counter[nth_drone][0] = 0
            elif new_WFSTATE == 6:
                self.state6counter[nth_drone][0] = 0
    ## SEZIONE AGGIUNTA PUNTI ##  
    # TODO: aggiunta punti in stato 4 (SOLO SE NECESSARIO) e -1>>>2
            if new_WFSTATE == 4:
                self.add_point(nth_drone,self.pos[nth_drone],'junction')
            if old_WFSTATE != 0 and new_WFSTATE == 1:
                self.add_point(nth_drone,self.pos[nth_drone],'corridor')
            if new_WFSTATE == 2: # inizio curva
                self.add_point(nth_drone,self.pos[nth_drone],'junction')
            if old_WFSTATE == 2: # fine curva
                if new_WFSTATE == 0 or new_WFSTATE == 3 or new_WFSTATE == 4:
                    self.add_point(nth_drone,self.pos[nth_drone],'junction')
    #### LOGICA CORNER ACKNOWLEDGEMENT ###
            if old_WFSTATE == 1 and new_WFSTATE == 0: # 
                self.IM_IN_A_CORNER[nth_drone][0] = True
            if old_WFSTATE == 0:
                if self.IM_IN_A_CORNER[nth_drone][0] == True:
                    self.add_point(nth_drone,self.pos[nth_drone],'corner')
                self.IM_IN_A_CORNER[nth_drone][0] = False
    #### gestione scelta randomica in un incrocio
    # self.MOVE_FORWARD modifica il comportamento degli stati 0 e 4 così che sappiano che sono post-4
            if (old_WFSTATE == 4 and new_WFSTATE == 0) or (old_WFSTATE == 4 and new_WFSTATE == 1):
                self.MOVE_FORWARD[nth_drone][0] = True
            else:
                self.MOVE_FORWARD[nth_drone][0] = False 
    #### OBSTACLE AVOIDANCE, salva lo stato precedente
            if new_WFSTATE == 5 and old_WFSTATE!=6 :
                self.memory_state[nth_drone] = old_WFSTATE
    #### GESTISCE SCELTA RAGGIO DELLA CURVA state 2    
            if new_WFSTATE == 2:
                if self.S_WF [nth_drone] == 1 :
                    if self.prev_rR[nth_drone] < 1:
                        self.radius[nth_drone] = self.prev_rR[nth_drone]
                    else:
                        self.radius[nth_drone] =self.DIST_WALL_REF
                elif self.S_WF [nth_drone] == -1: 
                    if self.prev_rL[nth_drone] < 1:
                        self.radius[nth_drone] = self.prev_rL[nth_drone]
                    else:
                        self.radius[nth_drone] =self.DIST_WALL_REF
            #elif new_WFSTATE == 2  :
            #    self.radius[nth_drone] = self.DIST_WALL_REF
            # flagga il contatore come True
            self.WFSTATE_WAS_CHANGED[nth_drone][0] = True
        else:
            print("ha tentato di cambiare stato 2 volte nello stesso step")

    ################################################################################

    def _WallFollowingandAlign3(self,
                                nth_drone):    #ALG B.2  versione con solo il sensore davanti, credo necessiti di un meccanismo di memoria
        """funzione per seguire il muro -> mi da la omega mentre sto navigando vicino al muro per evitare di allontanarmi dal muro
        Rifatta per usare le distanze laterali
        Sfrutta anche self.WF_ref_angle[i] angolo di partenza del WF
        Returns
        --------
        omega: tipo
            descrizione
        vel: tipo
            descrizione
        WFstate: int
            aggiorna il WFstate
        """
        outside_region_omega_reduction_factor = 2 # buono:1.5 ## TODO : tuning 
        inside_region_omega_reduction_factor = 0.6 # buono:0.4 ## TODO: tuning
        
        alfa = self.rpy[nth_drone][2] - self.WF_ref_angle[nth_drone]  # pos se sbando verso sinistra
                                                                      # neg se sbando verso destra
        alfa = self._anglefix(alfa)

        if self.S_WF[nth_drone][0] == 1: # wallfollowing con muro a destra
            lat_distance = self.observation[nth_drone][3] # distanza destra
            #lat_distance = lat_distance * np.cos(alfa)
            prev_lat_distance = self.prev_rR[nth_drone][0]
        elif self.S_WF[nth_drone][0] == -1: # wallfollowing con muro a sinistra
            lat_distance = self.observation[nth_drone][1] # distanza sinistra
            #lat_distance = lat_distance * np.cos(alfa)
            prev_lat_distance = self.prev_rL[nth_drone][0]
    
        if np.abs(alfa) < 0.3 :
            if np.abs(self.DIST_WALL_REF-lat_distance) > self.td : # sono fuori dalla regione accettabile ( o troppo lontano (distance > dist_wall_ref)) o troppo vicino  (distance < dist_wall_ref)
                if lat_distance > self.DIST_WALL_REF : #and self.rpy[nth_drone][2] < self.WF_ref_angle[nth_drone]:     # troppo lontano dal muro
                    omega = ([-self.S_WF[nth_drone][0]*self.C_OMEGA*outside_region_omega_reduction_factor])        # TODO check sign : CHECKED -
                    print("sono fuori dalla regione e troppo lontano dal muro")
                elif lat_distance < self.DIST_WALL_REF:                                                # troppo vicino al muro
                    omega = ([+self.S_WF[nth_drone][0]*self.C_OMEGA*outside_region_omega_reduction_factor])  # +
                    print("sono fuori dalla regione e troppo vicino al muro")       
            elif np.abs(self.DIST_WALL_REF-lat_distance) <  self.td : #sono dentro alla regione accettabile
                # meccanismo di fine tune alignment basato su confronto con distance[iesimo drone][0]
                if prev_lat_distance > lat_distance: # se la distanza dal muro prima era maggiore (mi sto avvicinando)
                    omega = ([+self.S_WF[nth_drone][0]*self.C_OMEGA*inside_region_omega_reduction_factor])     # TODO andiamo a culo vedi se cambiare
                    #omega = ([0.])
                    print("sono dentro alla regione e mi sto avvicinando al muro")
                else : 
                    omega = ([-self.S_WF[nth_drone][0]*self.C_OMEGA*inside_region_omega_reduction_factor])
                    #omega = ([0.])
                    print("sono dentro alla regione e mi sto allontanando dal muro")
            else :
                omega = ([0.])
        elif np.abs(alfa) > 0.3:
            if alfa > 0:
                omega = ([-1*outside_region_omega_reduction_factor*self.C_OMEGA])
                print("sto sbandando troppo verso sinistra rispetto alla direzione di partenza")
            elif alfa < 0:
                omega = ([+1*outside_region_omega_reduction_factor*self.C_OMEGA])
                print("sto sbandando troppo verso destra rispetto alla direzione di partenza")

        return omega      
    
    ################################################################################

    def _anglefix(self,
                  angle):
        """serve a fixare il fatto che angoli sotto pi sono positivi e 
        gli angoli sopra pi (pi < angle < 2pi) sono presi negatiivi quindi quando si naviga
        verso sud le differenze tra angoli escono spesso valori fallati, 
        generalmente sfalsati di 2pi
        """
        if np.abs(angle) > 4:
            if angle > 4:
                angle = angle - 2*np.pi
            elif angle < -4:
                angle = angle + 2*np.pi
        return angle
    
    ################################################################################

    def _MinDistToWall2(self):
        """Distanza minima dall'ostacolo.
            ATTENZIONE: nelle fasi di volo circa parallelo al muro non ritorna la distanza minima esatta ma l'observation del RF laterale
        TODO richiede sWF per capire da che lato  (e faccio che la aggiorna???)

        Updates
        ----------
        self.distance : ndarray   [[prev_dist , curr_dist]]
            (NUM_DRONES, 2)-shaped array containing the minimal distance observed  , se vale self.MAX_RANGE vuol dire che non vede proprio niente
        self.S_WF
            (NUM_DRONES, 1)-shaped array 
        """
        for i in range(self.NUM_DRONES):
            rF = self.observation[i][0] # distanza frontale
            rL = self.observation[i][1] # distanza sinistra
            rB = self.observation[i][2] # distanza retro
            rR = self.observation[i][3] # distanza destra
            if rF != self.MAX_RANGE and rL != self.MAX_RANGE :
                NWalfa = np.arctan(rF/rL) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                NWdistance = rF * np.cos(NWalfa)
            else:
                NWdistance = np.inf

            if rL != self.MAX_RANGE and rB != self.MAX_RANGE :
                SWalfa = np.arctan(rB/rL) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                SWdistance = rB * np.cos(SWalfa)
            else:
                SWdistance = np.inf

            if rB != self.MAX_RANGE and rR != self.MAX_RANGE :
                SEalfa = np.arctan(rB/rR) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                SEdistance = rB * np.cos(SEalfa)
            else:
                SEdistance = np.inf

            if rR != self.MAX_RANGE and rF != self.MAX_RANGE :
                NEalfa = np.arctan(rF/rR) #complementare dell'angolo di "raddrizzamento" rispetto al muro
                NEdistance = rF * np.cos(NEalfa)
            else:
                NEdistance = np.inf
            self.distance[i][1] = min(rF,rL,rB,rR,NWdistance,SWdistance,SEdistance,NEdistance)
            ######### spostato nel momento in cui si esce da WFSTATE = 0 #########
            # Imposta sWF in base al valore minimo
            #if self.distance[i][1] in [rL, NWdistance, SWdistance]:
            #    self.S_WF[i] = -1
            #elif self.distance[i][1] in [rR, NEdistance, SEdistance]:
            #    self.S_WF[i] = 1
            #else:
            #    self.S_WF[i] = 1  # preferenza antioraria
        return NWdistance,SWdistance,SEdistance,NEdistance
    
    def find_collision(self,nth_drone):
        """
        Gestisce la logica di identificazione di un ostacolo in movimento nelle vicinanze. Si basa sulla conoscenza a priori
        della posizione di tutti i droni. 

        Returns
        -------
        collision : ndarray
            (NUM_DRONES, 1)  
            l'iesimo valore è 
            0 se quel drone non lo vedo ;
            1 se vedo quel drone e ha id maggiore del mi ;
            2 se vedo quel drone e ha id minore del mio (gli devo lasciare precedenza)
        """
        collision_treshold = 1.5*self.DIST_WALL_REF
        drones_position = np.array([self._getDroneStateVector(j)[0:3] for j  in range(self.NUM_DRONES)])
        drone_velocity = self._getDroneStateVector(nth_drone)[10:13] 
        drones_distance =  np.array([[np.inf] for j in range(self.NUM_DRONES)] )
        collision =  np.array([[0] for j in range(self.NUM_DRONES)] )

        # drones_distance =  np.array([[[np.inf] for j in range(self.NUM_DRONES)] for j in range(self.NUM_DRONES)] )
        # collision =  np.array([[[0.] for j in range(self.NUM_DRONES)] for j in range(self.NUM_DRONES)] )

        for i in range(self.NUM_DRONES):
            if i!= nth_drone : #and self.WFSTATE[nth_drone]!=-1:
                relative_position = drones_position[i] - drones_position[nth_drone]
                rot_mat = np.array(p.getMatrixFromQuaternion(self.quat[nth_drone, :])).reshape(3, 3) 
                relative_position_rel = np.dot(relative_position,rot_mat)   
                drones_distance[i] = self.euclidean_distance( drones_position[nth_drone][0:2], drones_position[i][0:2])
                motion_drones = drones_distance[i] - self.drones_distance_pre[i]

                if drones_distance[i] <= collision_treshold :
                    #verifico se è dentro la mia treshold
                    collision[i] = 1 
                    if (((relative_position_rel[0]>=0 ) or ( motion_drones < 0 ) or drones_distance[i] < 0.15 ) and np.abs(relative_position[2])<0.3) or self.WFSTATE[nth_drone][0] == 0 :
                     # prima e seconda condizione sono per capire se lo ho davanti e gli sto andando in contro o se sono molto vicini
                     #la seconda è per capire se lo ho dietro e lui mi sta venendo incontro
                     #la terza controllo se è a una quota pericolosa                  
                        if (self.WFSTATE[nth_drone][0] != 4 and self.WFSTATE[nth_drone][0] != 2) and (self.WFSTATE[i][0] == 4 or self.WFSTATE[i][0] == 2): # se il drone i sta in stato 4 o 6 ha la precedenza
                            collision[i] = 2
                        elif (self.WFSTATE[i][0] != 4 and self.WFSTATE[i][0] != 2 ) and (self.WFSTATE[nth_drone][0] == 4 or self.WFSTATE[nth_drone][0] == 2 ): # se il drone nth_drone sta in stato 4 o 6 ha la precedenza
                            collision[i] = 1
                        elif (self.WFSTATE[i][0] == 4 or self.WFSTATE[i][0] == 2 ) and (self.WFSTATE[nth_drone][0] == 4 or self.WFSTATE[nth_drone][0] == 2 ):
                            if self.WFSTATE[nth_drone][0] == 4 and self.WFSTATE[i][0] == 2:
                               collision[i] = 1
                            elif self.WFSTATE[nth_drone][0] == 2 and self.WFSTATE[i][0] == 4:
                                collision[i] = 2
                            else:
                                if i < nth_drone: 
                                    collision[i] = 2
                        elif self.WFSTATE[nth_drone] != -1:
                            if self.WFSTATE[nth_drone] == 5:
                                collision[i] = 2
                            elif self.WFSTATE[i] == 5:
                                collision[i] = 1
                            elif i < nth_drone: 
                                collision[i] = 2
                        elif i < nth_drone: 
                            collision[i] = 2
                     
            self.drones_distance_pre = drones_distance
        return collision 
  
############################## CREAZIONE MAPPA TOPOLOGICA ############################################
## il database si chiama 
## self.drones_db
## la matrice di adiacenza è una sola
## self.adjacency_matrix

    def add_drone(self,
                  drone_id):
        """Aggiunge un drone col suo id nel database come chiave di primo livello drone_id
           e inizializza la matrice di adiacenza aggiungendo una matrice (1x1) con uno zero alla lista di matrici nota come self.adjacency_matrices
           Runnare in fase di inizializzazione
        Parameters
        -------
        drone_id: int
            id del drone da aggiungere
        """
        if drone_id not in self.drones_db:
            self.drones_db[drone_id] = {}
            adjacency_matrix = np.zeros((0, 0), dtype=int)
            self.adjacency_matrices.append(adjacency_matrix)

    def get_next_point_id(self,
                          drone_id):
        """restituisce il prossimo point_id inseribile nel dizionario del drone con un certo drone_id
        funziona anche se il database ha dei buchi
        Parameters
        -------
        drone_id: int
            id del drone di cui voglio sapere quale sarebbe il prossimo point_id

        Returns
        -------
        string : 'xxxx'
            stringa col prossimo point_id
        """
        if drone_id not in self.drones_db:
            return '0001'
        else:
            existing_ids = sorted(int(pid) for pid in self.drones_db[drone_id].keys())
            next_id = existing_ids[-1] + 1 if existing_ids else 1
            return f'{next_id:04}'
        
    def get_highest_point_id(self, drone_id):
        """
        Restituisce l'ID del punto con il numero più alto già inserito per un dato drone.

        Parameters:
        drone_id: int
            L'ID del drone per cui trovare il punto con l'ID più alto.

        Returns:
        str
            L'ID del punto con il numero più alto.
        """
        point_ids = list(self.drones_db[drone_id].keys())
        if not point_ids:
            return None  # Nessun punto esistente per il drone
        highest_point_id = max(point_ids, key=lambda x: int(x))
        return highest_point_id

    def get_last_added_point_id(self,
                                drone_id):
        """
        Restituisce l'ID dell'unico punto che ha nella chiave 'addition' il valore 'last'
        Poi sostituisce 'last' con 'previous'
        Parameters
        ----------
        drone_id : int
            ID del drone di cui vogliamo trovare l'ultimo punto aggiunto.

        Returns
        -------
        str
            L'ID del punto con il valore 'last'.
        """
        for point_id, data in self.drones_db[drone_id].items():
            if data['addition'] == 'last':
                self.drones_db[drone_id][point_id]['addition'] = 'previous'
                return point_id
        return None  # Nel caso in cui non ci sia nessun punto con il valore 'last'

    def get_last_added_point_id2(self,
                                 drone_id):
        """
        Restituisce l'ID dell'unico punto che ha nella chiave 'addition' il valore 'last'.
        Poi sostituisce 'last' con 'previous' per tutti i punti con questo valore.

        Parameters
        ----------
        drone_id : int
            ID del drone di cui vogliamo trovare l'ultimo punto aggiunto.

        Returns
        -------
        str
            L'ID del punto con il valore 'last'.
        """
        last_point_ids = []

        # Trova tutti i punti con il valore 'last'
        for point_id, data in self.drones_db[drone_id].items():
            if data['addition'] == 'last':
                last_point_ids.append(point_id)

        # Se ci sono punti con 'last', cambia il loro valore in 'previous'
        for point_id in last_point_ids:
            self.drones_db[drone_id][point_id]['addition'] = 'previous'

        # Restituisci l'ID del punto con il valore più alto, se esiste
        if last_point_ids:
            return max(last_point_ids, key=self.point_id_to_index)

        return None  # Nel caso in cui non ci sia nessun punto con il valore 'last'

    def add_point(self,
                  drone_id,
                  coords,
                  point_type):
        """aggiunge un punto al database del drone selezionato
        Parameters
        -------
        drone_id : int
            id del drone alla cui lista voglio aggiungere un punto
        coords : ndarray
            (3)-shaped array con le coord del punto
        point_type : string
            può essere 'corridor', 'juction', 'corner', 'start'
        addition : string
            può essere 'previous', 'last' , 'new'
            'previous': tutti i nodi precedenti
            'last' : il nodo creato per ultimo camminando e non mergiando
            'new' : il nodo creato a questo step
        """
        new_point_threshold_distance = 0.2 #0.7 funzionava
        if drone_id not in self.drones_db:
            self.drones_db[drone_id] = {}
        min_dist = self.distance_between_newpoint_and_oldpoints(drone_id,coords)
        if min_dist > new_point_threshold_distance or point_type == 'junction' or point_type == 'corner' or point_type == 'start':
            ### AGGIUNTA NUOVA CHIAVE AL DIZIONARIO self.drones_db ###
            previous_point_id = self.get_highest_point_id(drone_id)
            last_added_point = self.get_last_added_point_id2(drone_id)
            point_id = self.get_next_point_id(drone_id)
            self.drones_db[drone_id][point_id] = {'coords': coords.copy(), 'type': point_type, 'addition': 'last'}
            print(f'Added point {point_id} to drone {drone_id}')
            ### AGGIUNTA NUOVA RIGA E COLONNA alla adjacencymatrix[drone_id]
            current_matrix = self.adjacency_matrices[drone_id]
            # Creiamo una nuova matrice con una dimensione aumentata di 1
            new_matrix = np.pad(current_matrix, ((0, 1), (0, 1)), mode='constant', constant_values=0)
            #highest_id = self.get_highest_point_id(drone_id)
            #highest_index = self.point_id_to_index(highest_id)  #  '0002' >>> 1
            self.adjacency_matrices[drone_id] = new_matrix    
            ### AGGIUNTA PALLINO VISIVO ###
            self.add_visual_ball(drone_id, point_id, coords)
            ### AGGIUNTA EDGE ###
            # per ora solo tra nuovo punto e vecchio punto #
            if last_added_point:
                self.add_edge(drone_id, last_added_point, point_id) #TODO questo di strano ha che collega il punto all'ultimo in termini di codice che però potrebbe essere figlio di un merge qualsiasi anche in un'altra parte della mappa
            if self.MERGING:
                if self.SELFMERGE == True:
                    self.merge_similar_points_same_drone(drone_id,self.max_distance_between_nodes)
                for i in range(self.NUM_DRONES):
                    self.merge_similar_points_2_drones2(drone_id,i,self.max_distance_between_nodes)
            if self.edges_visualization:
                self.show_edges()

    def add_edge(self,
                 drone_id,
                 point_key1,
                 point_key2):
        """Da runnare dopo add_point
        scelto un drone drone_id, aggiunge il collegamento (edge) tra un nodo 1 e un nodo 2 
        
        Parameters
        -------
        drone_id: int
            id del drone in questione
        point_key1: str
            key del nodo 1 
        point_key2: str
            key del nodo 2
        """
        idx1 = self.point_id_to_index(point_key1)
        idx2 = self.point_id_to_index(point_key2)
        self.adjacency_matrices[drone_id][idx1, idx2] = 1
        self.adjacency_matrices[drone_id][idx2, idx1] = 1

    def euclidean_distance(self,
                           point1,
                           point2):
        """Distanza euclidea tra due nparray di qualsiasi dimensione
        Parameters
        -------
        point1: ndarray 
        point2: ndarray 

        Returns
        --------
        float
            euclidean distance between 2 points
        """
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def merge_similar_points_same_drone(self,
                                    drone_id,
                                    threshold=0.6,
                                    threshold_2=0.1):
        """ confronto incrociato tra i nodi dello stesso drone e sostituisce con la media
        Parameters
        -------
        drone_id: int 
        threshold: float - optional
        threshold_2: float - optional
            La soglia per determinare se le nuove coordinate sono troppo vicine a quelle già viste.
        """
        def are_coords_too_close(new_coords, seen_coords, threshold):
            """Verifica se le nuove coordinate sono troppo vicine a qualsiasi coordinata già vista."""
            for coords in seen_coords:
                if self.euclidean_distance(new_coords, coords) < threshold:
                    return True
            return False

        drone_points = self.drones_db[drone_id]
        to_remove = []
        to_add = []
        to_remove_balls = set()
        # Inizializza i contatori per il prossimo point_id
        k = 0
        seen_coords = set()
        for point_id1, data1 in drone_points.items():
            for point_id2, data2 in drone_points.items():
                if self.point_id_to_index(point_id2) > self.point_id_to_index(point_id1) and data1['type']!='start' and data2['type']!='start':
                    next_point_id_drone1 = self.get_next_point_id(drone_id)
                    if 0 < self.euclidean_distance(data1['coords'], data2['coords']) < threshold:
                        new_coords = np.mean([data1['coords'], data2['coords']], axis=0).tolist() #TODO: aggiungere media pesata (forse)
                        new_coords_tuple = tuple(new_coords)
                        new_type = data1['type']
                        if data1['addition'] == 'last' or data2['addition'] == 'last':
                            addition_type = 'last'
                        else:
                            addition_type = 'previous'

                        # Controlla se le nuove coordinate sono troppo vicine a quelle già viste
                        if new_coords_tuple not in seen_coords and not are_coords_too_close(new_coords_tuple, seen_coords, threshold_2):
                            # Accumula le chiavi da rimuovere
                            to_remove.append(point_id1)
                            to_remove.append(point_id2)
                            to_remove_balls.add((drone_id, point_id1))
                            to_remove_balls.add((drone_id, point_id2))
                            # Incrementa i contatori
                            next_point_id_drone1 = self.increment_point_id(next_point_id_drone1, k)
                            k += 1
                            # Accumula le chiavi con i rispettivi valori da aggiungere
                            to_add.append((next_point_id_drone1, {'coords': new_coords, 'type': new_type, 'addition': addition_type}))
                            # SOSTITUZIONE EDGE NELLA ADJACENCY MATRIX
                            self.adjacency_matrices[drone_id] = self.replace_2nodes_in_adjacency_matrix(
                                self.adjacency_matrices[drone_id],
                                point_id1,
                                point_id2,
                                next_point_id_drone1)
                            seen_coords.add(new_coords_tuple)

        # Rimuovi le vecchie istanze e le vecchie palline
        to_remove = set(to_remove)  # evita ripetizioni
        for point_id in to_remove:
            del self.drones_db[drone_id][point_id]
        for drone_id, point_id in to_remove_balls:
            self.remove_visual_ball(drone_id, point_id)

        # Aggiungi i nuovi nodi e visualizzazioni
        for point_id, data in to_add:
            self.drones_db[drone_id][point_id] = data
            self.add_visual_ball(drone_id, point_id, data['coords'])

    def merge_similar_points_2_drones(self,
                                       drone1_id,
                                       drone2_id,
                                       threshold=0.58,
                                       threshold_2=0.1):
        """ confronto incrociato tra i nodi di 2 droni diversi e sostituisce con la media
        Parameters
        -------
        drone1_id: int 
        drone2_id: int
        threshold: float - optional
        threshold_2: float - optional
            La distanza minima tra i nuovi punti aggiunti per evitare densità eccessive di nodi.
        """
        drone1_points = self.drones_db[drone1_id]
        drone2_points = self.drones_db[drone2_id]

        if drone1_id != drone2_id: # se tolta i droni mergiano anche con sè stessi
            to_remove_drone1 = []
            to_remove_drone2 = []
            to_add_drone1 = []
            to_add_drone2 = []
            to_remove_balls = set()
            seen_coords = set()

            # Inizializza i contatori per il prossimo point_id
            k = 0
            for point_id_drone1, data1 in drone1_points.items():
                for point_id_drone2, data2 in drone2_points.items():
                    next_point_id_drone1 = self.get_next_point_id(drone1_id)
                    next_point_id_drone2 = self.get_next_point_id(drone2_id)
                    if 0.1 < self.euclidean_distance(data1['coords'], data2['coords']) < threshold and data1['type']!='start' and data2['type']!='start':
                        new_coords = np.mean([data1['coords'], data2['coords']], axis=0).tolist()
                        new_coords_tuple = tuple(new_coords)

                        # Verifica che le nuove coordinate non siano troppo vicine a quelle già viste
                        if not any(self.euclidean_distance(new_coords, seen_coord) < threshold_2 for seen_coord in seen_coords):
                            new_type = data1['type']
                            # Accumula le chiavi da rimuovere
                            to_remove_drone1.append(point_id_drone1)
                            to_remove_drone2.append(point_id_drone2)
                            to_remove_balls.add((drone1_id, point_id_drone1))
                            to_remove_balls.add((drone2_id, point_id_drone2))
                            # Incrementa i contatori
                            next_point_id_drone1 = self.increment_point_id(next_point_id_drone1, k)
                            next_point_id_drone2 = self.increment_point_id(next_point_id_drone2, k)
                            k += 1
                            # Accumula le chiavi con i rispettivi valori da aggiungere
                            addition_type1 = 'last' if data1['addition'] == 'last' else 'previous'
                            addition_type2 = 'last' if data2['addition'] == 'last' else 'previous'
                            to_add_drone1.append((next_point_id_drone1, {'coords': new_coords, 'type': new_type, 'addition': addition_type1}))
                            to_add_drone2.append((next_point_id_drone2, {'coords': new_coords, 'type': new_type, 'addition': addition_type2}))
                            # SOSTITUZIONE EDGE NELLA ADJACENCY MATRIX
                            self.adjacency_matrices[drone1_id] = self.replace_node_in_adjacency_matrix(self.adjacency_matrices[drone1_id], point_id_drone1, next_point_id_drone1)
                            self.adjacency_matrices[drone2_id] = self.replace_node_in_adjacency_matrix(self.adjacency_matrices[drone2_id], point_id_drone2, next_point_id_drone2)
                            # Aggiungi le nuove coordinate a seen_coords
                            seen_coords.add(new_coords_tuple)

            # Rimuovi le vecchie istanze e le vecchie palline
            to_remove_drone1 = set(to_remove_drone1) # evita ripetizioni
            to_remove_drone2 = set(to_remove_drone2)
            for point_id in to_remove_drone1:
                del self.drones_db[drone1_id][point_id]
            for point_id in to_remove_drone2:
                del self.drones_db[drone2_id][point_id]
            for drone_id, point_id in to_remove_balls:
                self.remove_visual_ball(drone_id, point_id)

            # Aggiungi i nuovi nodi e visualizzazioni
            for point_id, data in to_add_drone1:
                self.drones_db[drone1_id][point_id] = data
                self.add_visual_ball(drone1_id, point_id, data['coords'])
            for point_id, data in to_add_drone2:
                self.drones_db[drone2_id][point_id] = data
                self.add_visual_ball(drone2_id, point_id, data['coords'])

    def merge_similar_points_2_drones2(self,
                                       drone1_id,
                                       drone2_id,
                                       threshold=0.58,
                                       threshold_2=0.01):
        """ confronto incrociato tra i nodi di 2 droni diversi e sostituisce con la media
        Parameters
        -------
        drone1_id: int 
        drone2_id: int
        threshold: float - optional
        threshold_2: float - optional
            La distanza minima tra i nuovi punti aggiunti per evitare densità eccessive di nodi.
        """
        drone1_points = self.drones_db[drone1_id]
        drone2_points = self.drones_db[drone2_id]

        if drone1_id != drone2_id: # se tolta i droni mergiano anche con sè stessi
            to_remove_drone1 = []
            to_remove_drone2 = []
            to_add_drone1 = []
            to_add_drone2 = []
            to_remove_balls = set()
            seen_coords = set()

            # Inizializza i contatori per il prossimo point_id
            k = 0
            for point_id_drone1, data1 in drone1_points.items():
                for point_id_drone2, data2 in drone2_points.items():
                    next_point_id_drone1 = self.get_next_point_id(drone1_id)
                    next_point_id_drone2 = self.get_next_point_id(drone2_id)
                    if 0.01 < self.euclidean_distance(data1['coords'], data2['coords']) < threshold and data1['type']!='start' and data2['type']!='start':
                        new_coords = np.mean([data1['coords'], data2['coords']], axis=0).tolist()
                        new_coords_tuple = tuple(new_coords)
                        # Accumula le chiavi da rimuovere a prescindere dall'aggiunta o no di un nuovo nodo mediato
                        to_remove_drone1.append(point_id_drone1)
                        to_remove_drone2.append(point_id_drone2)
                        to_remove_balls.add((drone1_id, point_id_drone1))
                        to_remove_balls.add((drone2_id, point_id_drone2))
                        # Verifica che le nuove coordinate non siano troppo vicine a quelle già viste
                        # ora se non ho già messo un nodo da quelle parti lo aggiungo
                        if not any(self.euclidean_distance(new_coords, seen_coord) < threshold_2 for seen_coord in seen_coords):
                            new_type = data1['type']
                            if data1['type'] == 'junction' or data2['type'] == 'junction':
                                new_type = 'junction'
                            # Incrementa i contatori
                            next_point_id_drone1 = self.increment_point_id(next_point_id_drone1, k)
                            next_point_id_drone2 = self.increment_point_id(next_point_id_drone2, k)
                            k += 1
                            # Accumula le chiavi con i rispettivi valori da aggiungere
                            addition_type1 = 'last' if data1['addition'] == 'last' else 'previous'
                            addition_type2 = 'last' if data2['addition'] == 'last' else 'previous'
                            to_add_drone1.append((next_point_id_drone1, {'coords': new_coords, 'type': new_type, 'addition': addition_type1}))
                            to_add_drone2.append((next_point_id_drone2, {'coords': new_coords, 'type': new_type, 'addition': addition_type2}))
                            # SOSTITUZIONE EDGE NELLA ADJACENCY MATRIX
                            self.adjacency_matrices[drone1_id] = self.replace_node_in_adjacency_matrix(self.adjacency_matrices[drone1_id], point_id_drone1, next_point_id_drone1)
                            self.adjacency_matrices[drone2_id] = self.replace_node_in_adjacency_matrix(self.adjacency_matrices[drone2_id], point_id_drone2, next_point_id_drone2)
                            # Aggiungi le nuove coordinate a seen_coords
                            seen_coords.add(new_coords_tuple)

            # Rimuovi le vecchie istanze e le vecchie palline
            to_remove_drone1 = set(to_remove_drone1) # evita ripetizioni
            to_remove_drone2 = set(to_remove_drone2)
            for point_id in to_remove_drone1:
                del self.drones_db[drone1_id][point_id]
            for point_id in to_remove_drone2:
                del self.drones_db[drone2_id][point_id]
            for drone_id, point_id in to_remove_balls:
                self.remove_visual_ball(drone_id, point_id)

            # Aggiungi i nuovi nodi e visualizzazioni
            for point_id, data in to_add_drone1:
                self.drones_db[drone1_id][point_id] = data
                self.add_visual_ball(drone1_id, point_id, data['coords'])
            for point_id, data in to_add_drone2:
                self.drones_db[drone2_id][point_id] = data
                self.add_visual_ball(drone2_id, point_id, data['coords'])


    def replace_node_in_adjacency_matrix(self,
                                         adjacency_matrix,
                                         old_node_id,
                                         new_node_id):
        """
        Sostituisce un nodo in una matrice di adiacenza con un nuovo nodo, trasferendo tutti gli edge.

        Parameters
        ----------
        adjacency_matrix : np.ndarray
            La matrice di adiacenza.
        old_node : str
            L'ID 'xxxx' del nodo da sostituire.
        new_node : str
            L'ID 'xxxx' del nuovo nodo.

        Returns
        -------
        np.ndarray
            La nuova matrice di adiacenza con il nodo sostituito.
        """
        old_node = self.point_id_to_index(old_node_id)
        new_node = self.point_id_to_index(new_node_id)
        
        num_nodes = adjacency_matrix.shape[0]

        # Creiamo una nuova matrice di adiacenza con una dimensione aumentata di 1
        new_matrix = np.zeros((num_nodes + 1, num_nodes + 1), dtype=int)

        # Copiamo la matrice di adiacenza originale nella nuova matrice
        new_matrix[:num_nodes, :num_nodes] = adjacency_matrix

        # Trasferiamo tutti gli edge dal vecchio nodo al nuovo nodo
        new_matrix[new_node, :-1] = adjacency_matrix[old_node, :]
        new_matrix[:-1, new_node] = adjacency_matrix[:, old_node]

        # Impostiamo a zero la riga e la colonna del vecchio nodo
        new_matrix[old_node, :] = 0
        new_matrix[:, old_node] = 0

        return new_matrix

    def replace_2nodes_in_adjacency_matrix(self,
                                           adjacency_matrix,
                                           node1_id,
                                           node2_id,
                                           new_node_id):
        """
        Unisce due nodi in una matrice di adiacenza in un nuovo nodo, trasferendo tutti gli edge.

        Parameters
        ----------
        adjacency_matrix : np.ndarray
            La matrice di adiacenza.
        node1_id : str
            L'ID 'xxxx' del primo nodo da unire.
        node2_id : str
            L'ID 'xxxx' del secondo nodo da unire.
        new_node_id : str
            L'ID 'xxxx' del nuovo nodo che sostituirà i due nodi.

        Returns
        -------
        np.ndarray
            La nuova matrice di adiacenza con i nodi uniti.
        """
        node1 = self.point_id_to_index(node1_id)
        node2 = self.point_id_to_index(node2_id)
        new_node = self.point_id_to_index(new_node_id)

        num_nodes = adjacency_matrix.shape[0]

        # Creiamo una nuova matrice di adiacenza con una dimensione aumentata di 1
        new_matrix = np.zeros((num_nodes + 1, num_nodes + 1), dtype=int)

        # Copiamo la matrice di adiacenza originale nella nuova matrice
        new_matrix[:num_nodes, :num_nodes] = adjacency_matrix

        # Trasferiamo tutti gli edge dai vecchi nodi al nuovo nodo
        new_matrix[new_node, :-1] = np.maximum(adjacency_matrix[node1, :], adjacency_matrix[node2, :])
        new_matrix[:-1, new_node] = np.maximum(adjacency_matrix[:, node1], adjacency_matrix[:, node2])

        # Impostiamo a zero le righe e le colonne dei vecchi nodi
        new_matrix[node1, :] = 0
        new_matrix[:, node1] = 0
        new_matrix[node2, :] = 0
        new_matrix[:, node2] = 0

        return new_matrix

    def increment_point_id(self,string,quantity):
        """Incrementa una stringa di point_id del tipo 'xxxx' di quantityù
        Params
        string : str
            stringa di tipo 'xxxx'
        quantity : int
            quantità di cui deve crescere la stringa
        """
        return f'{int(string) + quantity:04}'

    def distance_between_newpoint_and_oldpoints(self,
                                                drone_id,
                                                coords,
                                                ):
        """ da runnare in add_point. distanza minima tra l'eventuale nuovo nodo e nodo più vicino già nel dizionario#TODO: aggiusta sensibilità
        Parameters
        -------
        drone_id: int 
            id del drone alla cui lista voglio aggiungere un punto
        coords : ndarray
            (3)-shaped array con le coord del punto
        
        Returns
        -------
        float
            distanza minima
        """
        drone_points = self.drones_db[drone_id]
        min_dist = 5.0
        for id, data in drone_points.items():
            if self.euclidean_distance(data['coords'], coords) < min_dist:
                min_dist = self.euclidean_distance(data['coords'], coords)
        return min_dist


    def remove_point(self,
                     drone_id,
                     point_id):
        """rimuove un punto al database del drone selezionato
        Parameters
        -------
        drone_id : int
            id del drone dalla cui lista voglio rimuovere un punto
        point_id : str  
            'xxxx' string 
        """
        if drone_id in self.drones_db and point_id in self.drones_db[drone_id]:
            del self.drones_db[drone_id][point_id]
            self.remove_visual_ball(drone_id,point_id)
            print(f'Removed point {point_id} from drone {drone_id}')
        else:
            print(f'Point {point_id} not found for drone {drone_id}')

    def display_drones_db(self):
        """mostra l'intero database
        """
        for drone_id, points in self.drones_db.items():
            print(f"Drone {drone_id}:")
            for point_id, details in points.items():
                print(f"  Point {point_id}: {details}")

    def add_visual_ball(self,
                        drone_id,
                        point_id,
                        position,
                        color=[1, 0, 0, 1],
                        radius=0.03,
                        ):
        """crea un pallino colorato senza collisioni in un punto
        --------
        Parametres
        drone_id : int
            id del drone dalla cui lista voglio rimuovere un punto
        point_id : str  
            'xxxx' string 
        position: ndarray (3) 
            position of the pallino
        color: ndarray (4) 
            RGBA-format color
        radius: float - optional
        """
        if drone_id == 0:
            color = [1, 0, 0, 1] # red
        elif drone_id == 1:
            color = [0, 1, 0, 1] # green
        elif drone_id == 2:
            color = [0, 0, 1, 1] # blue
        elif drone_id == 3:
            color = [1, 1, 0, 1] # yellow
        elif drone_id == 4:
            color = [0.5, 0, 0.5, 1] # purple
        elif drone_id == 5:
            color = [0, 1, 1, 1] # ciano

        # Aggiungi un offset verticale alla posizione in base al drone_id
        z_offset = drone_id * 2*radius  # Regola il moltiplicatore per la spaziatura desiderata
        new_position = [position[0], position[1], position[2] + z_offset]

        visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=radius, rgbaColor=color)
        ball_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape_id, basePosition=new_position)
        # salva id pallino
        custom_id = f"{drone_id:03}_{point_id}"
        self.custom_id_balls_map[custom_id] = ball_id # copy?
        return ball_id
    
    def remove_visual_ball(self,
                           drone_id,
                           point_id,
                           ):
        """rimuove un pallino colorato di un certo drone
        --------
        Parametres
        drone_id : int
            id del drone dalla cui lista voglio rimuovere un punto
        point_id : str  
            'xxxx' string 
        """
        custom_id = f"{drone_id:03}_{point_id}"
        if custom_id in self.custom_id_balls_map:
            ball_id = self.custom_id_balls_map[custom_id]
            p.removeBody(ball_id)
            del self.custom_id_balls_map[custom_id]
        else:
            print(f"Custom ID {custom_id} not found in the list of balls.")

    def add_visual_line(self,
                        drone_id,
                        point_a,
                        point_b,
                        color=[1, 0, 0],
                        width=2):
        """crea una linea colorata senza collisioni da un punto a a un punto b
        --------
        Parametres
        drone_id : int
            id del drone dalla cui lista voglio rimuovere un punto
        point_a: ndarray (3) 
            posizione di partenza
        point_b: ndarray (3) 
            posizione di fine 
        color: ndarray (3) 
            RGB-format color
        width: float - optional
        """
        if drone_id == 0:
            color = [1, 0, 0] # red
        elif drone_id == 1:
            color = [0, 1, 0] # green
        elif drone_id == 2:
            color = [0, 0, 1] # blue
        elif drone_id == 3:
            color = [1, 1, 0] # yellow
        elif drone_id == 4:
            color = [0.5, 0, 0.5] # purple
        elif drone_id == 5:
            color = [0, 1, 1] # ciano
        ball_radius = 0.03 # copiare il valore da add_visual_ball
        # Aggiungi un offset verticale alle posizioni in base al drone_id
        z_offset = drone_id * 2 * ball_radius  # Regola il moltiplicatore per la spaziatura desiderata
        new_point_a = [point_a[0], point_a[1], point_a[2] + z_offset]
        new_point_b = [point_b[0], point_b[1], point_b[2] + z_offset]

        line_id = p.addUserDebugLine(new_point_a, new_point_b, color, width)
        return line_id


    def show_edges(self):
        """da fare: far si che rimuova tutti gli item di debug tranne le terne orientate solidali ai droni
        """
        p.removeAllUserDebugItems() # TODO: def remove_non_persistent_debug_items(persistent_debug_items):
        for drone_id, adjacency_matrix in enumerate(self.adjacency_matrices):
            drone_points = self.drones_db[drone_id]  
            point_ids = list(drone_points.keys())
            # al momento si basa sull'elenco punti ma si deve basare sulla matrice
            num_points = len(point_ids)
            point_indexes = []
            for i in range(num_points):
                point_indexes.append(self.point_id_to_index(point_ids[i])) 
            z=0
            for i in point_indexes:
                for j in point_indexes:
                    if adjacency_matrix[i][j] == 1:
                        point_a_id = self.index_to_point_id(i)
                        point_b_id = self.index_to_point_id(j)
                        point_a_coords = drone_points[point_a_id]['coords']
                        point_b_coords = drone_points[point_b_id]['coords']
                        self.add_visual_line(drone_id,point_a_coords,point_b_coords)
  
    def point_id_to_index(self, point_id:str):
        """
        Converte una stringa di point_id del tipo 'xxxx' nel corrispondente indice Python (intero).
        Params:
        point_id : str
            Stringa del tipo 'xxxx'.

        Returns:
        int
            Indice Python corrispondente (0-based).
        """
        return int(point_id) - 1
    
    def index_to_point_id(self, index: int):
        """
        Converte un indice Python (intero) nel corrispondente stringa di point_id del tipo 'xxxx'.
        Params:
        index : int
            Indice Python (0-based).

        Returns:
        str
            Stringa del tipo 'xxxx' corrispondente.
        """
        return f'{index + 1:04}'

################ VALUTAZIONE MISSIONE ########################

    def calculate_coverage(self, radius):
        """
        Calcola la percentuale di copertura dell'area da parte dei droni.

        Parameters
        ----------
        radius : float
            Il raggio degli intorni circolari intorno a ciascun nodo.

        Returns
        -------
        float
            La percentuale di copertura dell'area.
        """
        covered_area = 0
        buffers = []

        for drone_id, points in self.drones_db.items():
            for point_id, data in points.items():
                coords = data['coords']
                point = Point(coords[0], coords[1])
                buffer = point.buffer(radius).intersection(self.total_area_polygon)
                buffers.append(buffer)

        union_buffers = unary_union(buffers)
        covered_area = union_buffers.area
        coverage_percent = (covered_area / self.total_area_polygon.area) * 100
        ############# OBSOLETO ##############
        # faccio la verifica di fine missione sulle singole percentuali non sul totale
        ## if coverage_percent >= self.TARGET_COVERAGE and self.COVERAGE_IS_ENOUGH == False:
        ##     self.efficiency = self.step_counter*self.PYB_TIMESTEP
        ##     self.COVERAGE_IS_ENOUGH = True
        ##     self.plot_coverage(self.total_area_polygon, radius) #TODO: vedere come deve esssere l'input EVALUATOR
        ##     self.returning_phase_paths_planning()
        return coverage_percent

    def calculate_coverage_individually(self, radius):
        """
        Calcola la percentuale di copertura dell'area da parte di ciascun drone.

        Parameters
        ----------
        radius : float
            Il raggio degli intorni circolari intorno a ciascun nodo.

        Returns
        -------
        list of float
            Una lista contenente la percentuale di copertura dell'area per ciascun drone.
        """
        coverage_per_drone = []

        for drone_id, points in self.drones_db.items():
            buffers = []
            for point_id, data in points.items():
                coords = data['coords']
                point = Point(coords[0], coords[1])
                buffer = point.buffer(radius).intersection(self.total_area_polygon)
                buffers.append(buffer)

            union_buffers = unary_union(buffers)
            covered_area = union_buffers.area
            coverage_percent = (covered_area / self.total_area_polygon.area) * 100
            coverage_per_drone.append(coverage_percent)
        ############# OBSOLETO ##############
        # 17 / 07 - il fine missione si basa sul tempo trascorso
        # per evitare questo basta settare TARGET_COVERAGE = 101
        if all(coverage >= self.TARGET_COVERAGE for coverage in coverage_per_drone) \
                and not self.COVERAGE_IS_ENOUGH:
            self.efficiency = self.step_counter * self.PYB_TIMESTEP
            self.COVERAGE_IS_ENOUGH = True
            self.plot_coverage(self.total_area_polygon, radius)  # Plotta la copertura totale
            self.returning_phase_paths_planning()

        return coverage_per_drone


    def plot_coverage(self, total_area_polygon, radius):
        """
        Visualizza l'area coperta dai droni all'interno del labirinto.

        Parameters
        ----------
        evaluator : DroneMissionEvaluator
            Un'istanza della classe DroneMissionEvaluator contenente le informazioni sui droni e il poligono dell'area totale.
        radius : float
            Il raggio degli intorni circolari intorno a ciascun nodo.

        Returns
        -------
        None
        """
        fig, ax = plt.subplots()

        # Traccia l'area totale del labirinto
        rotated_total_area = rotate(total_area_polygon, 90, origin=(0, 0))
        x, y = rotated_total_area.exterior.xy
        ax.plot(x, y, label='Total Area')

        for drone_id, points in self.drones_db.items():
            for point_id, data in points.items():
                coords = data['coords']
                point = Point(coords[0], coords[1])
                buffer = point.buffer(radius).intersection(total_area_polygon)
                rotated_buffer = rotate(buffer, 90, origin=(0, 0))
                if rotated_buffer.geom_type == 'Polygon':
                    x, y = rotated_buffer.exterior.xy
                    ax.plot(x, y, label=f'Buffer {point_id}')
                elif rotated_buffer.geom_type == 'MultiPolygon':
                    for poly in rotated_buffer:
                        x, y = poly.exterior.xy
                        ax.plot(x, y, label=f'Buffer {point_id}')
                else:
                    rotated_point = rotate(point, 90, origin=(0, 0))
                    ax.plot(rotated_point.x, rotated_point.y, 'o', label=f'Buffer {point_id}')

            # Imposta le etichette degli assi
        plt.xlabel('y')
        plt.ylabel('x')
        plt.title('Coverage Area')
        # Ottieni i limiti degli assi
        xlim = ax.get_xlim()
        # Imposta le etichette invertite per l'asse x
        xticks = np.linspace(xlim[0], xlim[1], num=5)
        ax.set_xticks(xticks)
        ax.set_xticklabels([f'{int(label):.0f}' for label in xticks[::-1]])
        #plt.legend()
        plt.show(block=self.BLOCK_SIMULATION_WHEN_RETURNING_PHASE_STARTS)
            
############ WAYPOINT FINE MISSIONE ################

    def get_closest_node(self,
                         drone_id):
        """
        trova il nodo più vicino da cui iniziare il tracciato

        Parameters
        ----------
        drone_id : int
            id del drone

        Returns
        ----------
        str
            L'ID del punto più vicino da cui partire per il ritorno
        """
        actual_position = self.pos[drone_id]
        drone_points = self.drones_db[drone_id]
        min_dist = np.inf
        closest_point_id = []
        for point_id, data in drone_points.items():
            distance = self.euclidean_distance(actual_position,data['coords'])
            if distance < min_dist:
                min_dist = distance
                closest_point_id = point_id
        return closest_point_id

    def dijkstra(self,
                 drone_id,
                 start_node):
        """
        Implementa l'algoritmo di Dijkstra per trovare il percorso più breve da start_node a tutti gli altri nodi.

        Params:
        drone_id : int
            ID del drone.
        start_node : str
            ID del nodo di partenza 'start'

        Returns:
        dict
            Distanze minime dal nodi di partenza a tutti gli altri nodi.
        dict
            Nodo precedente nel percorso più breve per ciascun nodo.
        """
        nodes = list(self.drones_db[drone_id].keys())
        adjacency_matrix = self.adjacency_matrices[drone_id]
        num_nodes = len(nodes)

        visited = set()
        distances = {node: float('inf') for node in nodes}
        previous_nodes = {node: None for node in nodes}
        distances[start_node] = 0

        while len(visited) < num_nodes:
            current_node = min((node for node in nodes if node not in visited), key=lambda node: distances[node])
            visited.add(current_node)
            #current_index = self.point_id_to_index(current_node)

            for neighbor_index, weight in enumerate(adjacency_matrix[self.point_id_to_index(current_node)]):
                neighbor_node = self.index_to_point_id(neighbor_index)
                if weight > 0 and neighbor_node in nodes and neighbor_node not in visited:
                    new_distance = distances[current_node] + weight
                    if new_distance < distances[neighbor_node]:
                        distances[neighbor_node] = new_distance
                        previous_nodes[neighbor_node] = current_node

        return distances, previous_nodes

    def find_path_to_start(self,
                           drone_id):
        """
        Trova il percorso più breve dal nodo più vicino alla posizione corrente del drone al nodo di partenza.

        Params:
        drone_id : int
            ID del drone.

        Returns:
        list
            Lista ordinata di nodi da raggiungere per tornare al nodo di partenza.
        """
        # Trova il nodo più vicino alla posizione corrente da cui partire
        closest_node_id = self.get_closest_node(drone_id)

        # Prende il nodo con il tag 'start'
        start_node_id = None
        for node_id, data in self.drones_db[drone_id].items():
            if data['type'] == 'start':
                start_node_id = node_id
                break
        if start_node_id is None:
            raise ValueError(f"Start node not found for drone {drone_id}")

        # Trova il path più veloce: closest node >>> start node
        distances, previous_nodes = self.dijkstra(drone_id, start_node_id)

        # Ricostruisce il percorso dal nodo più vicino al nodo di partenza
        path = []
        current_node = closest_node_id
        while current_node is not None:
            path.append(current_node)
            current_node = previous_nodes[current_node]

        return path
    
    def returning_phase_paths_planning(self):
        """
        funzione che runna find_path_to_start per ogni drone nel momento in cui la missione
        ha coperto un coverage sufficiente

        updates
        ------- 
        self.returning_paths:
            dizionario di liste, ognuna contiene l'elenco ordinato di id dei nodi del percorso di ritorno (WAYPOINTS)
        self.NUM_WP :  ndarray (NUM_DRONES, 1)
            numero di WAYPOINTS del percorso di ritorno per ogni drone
        """
        for drone_id in self.drones_db.keys():
            path = self.find_path_to_start(drone_id)
            self.returning_paths[drone_id] = path
            self.NUM_WP[drone_id][0] = len(path)
